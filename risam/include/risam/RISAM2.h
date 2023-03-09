/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/** @brief ISAM2 extension "robust incremental Smoothing and Mapping" (riSAM) for incremental graduated non-convexity.
 *
 * This class inherits from gtsam's ISAM2 class, replacing a small amount of its re-linearization logic to extend the
 * algorithm to do the following...
 *      1. Convexify all factors associated with variables affected by the re-elimination of the top of the bayes tree
 *      2. Update the convexifying parameter for all variables that are relinearized due to a change in their estimate
 *
 *  @author Dan McGann
 *  @date Mar 2022
 */
#pragma once
#include <gtsam/nonlinear/ISAM2.h>

#include <deque>

#include "risam/DoglegLineSearch.h"
#include "risam/GraduatedFactor.h"

#define RISAM_DEBUG false

using namespace gtsam;
namespace risam {
class RISAM2 : public ISAM2 {
  /** TYPES **/
 public:
  enum RelinType { QUADRATIC, ROBUST, CACHED };
  struct RiSAMUpdateInfo {
    FastMap<size_t, RelinType> relinearization_types;
    KeySet updateInvolvedKeys;
    KeySet affectedKeysConvex;
    FastMap<size_t, double> mu;
  };
  RiSAMUpdateInfo last_update_info_;

  struct RISAM2Params {
    /// @brief The base ISAM2 params
    ISAM2Params isam2_params;
    /// @brief Optional params for custom line-search optimization method
    boost::optional<DoglegLineSearchParams> optimization_params{boost::none};
    // Constrain Variables involved because of convexity to be farther from the root than update variables
    bool use_custom_ordering_constraint{true};
    // Iterate until all mu's have converged before adding new GNC factors
    bool converge_after_new_gnc{true};    // Iterate until some form of convergence after update with GNC factors
    bool converge_mu{true};               // Iterate until mu convergence
    bool converge_values{true};           // Iterate until variable estimates converge
    size_t value_converge_max_iters{20};  // Max iterations if converging values
    double value_converge_abs_tol{1e-2};  // Absolute Error decrease convergence tolerance
    double value_converge_rel_tol{1e-2};  // Relative Error decrease convergence tolerance

    // Increment mu_init when we converge in param values for outlier factors
    bool increment_outlier_mu{true};
    double outlier_mu_chisq_upper_bound{0.9};            // Increment mu_init if chi^2 > upper
    double outlier_mu_chisq_lower_bound{0.25};           // Decrement mu_init if chi^2 < lower
    double outlier_mu_avg_var_convergence_thresh{0.01};  // Average variable delta norm
  };

  /** FIELDS **/
 protected:
  RISAM2Params risam_params_;
  /// @brief The convexifying parameter associated with each factor in nonlinearFactors_
  FastVector<double> mu_;
  /// @brief The mu_init value to use for each factor
  FastVector<double> mu_inits_;
  /// @brief The Factors for which we should check status next time the solution converges
  FastSet<size_t> factors_to_check_status_;

  /// @brief Indicies of factors that are known to be inliers. (i.e. Odometry measurements)
  FastSet<size_t> known_inliers_;
  /// @brief All factors for which mu has not converged
  FastSet<size_t> convex_factors_;
  /// @brief Flag used to indicate that the current update involves factors that are potential outliers
  bool new_factors_include_gnc_;
  /// @brief The ordering of the entire factor graph, 0=leaf, \infty = root
  std::deque<Key> variable_ordering_;

  /** TERMS AND DEFINITIONS
   *
   * NOTE: The following definitions were used during early riSAM implementation. They enable flexibility and through
   * hyper parameters can be used to run variants of the riSAM algorithm that are not presented in the currently
   * published work.
   *
   * ~~Definitions for Factors / Keys used in Update~~
   * For the below each set has two parallel definitions.
   * 1. Set of all factors as described below
   * 2. Set of all keys affected by the set of factors described below.
   *
   * Note: Plus is used to indicate a non-mutually exclusive set
   *
   * "Update-Directly-Affected"
   * Keys that are directly affected in the update, this is keys that are touched by new factors.
   *
   * "Update-Involved":
   * The set of all factors that would be involved in the re-elimination of the top of the bayes tree
   * in the original ISAM 2 algorithm. These are factors in cliques who's frontal variables are involved with the
   *new factors, and factors from cliques along paths to the root.
   *
   * "Convex-Involved-Plus":
   * The set of all factors that riSAM additionally includes in elimination. These are all factors that are still
   *convex (mu parameter has not converged), and factors of cliques on the paths to the root.
   *
   * "Convex-Involved":
   * The "Convex-Involved+" set, minus any factor that is "Update-Involved"
   *
   * "Involved":
   * Set of factors defined by the union of "Update-Involved" and "Convex-Involved"
   * This is the set of variables/factors that we preform re-elimination/relinearization on each update.
   *
   * "Relin-Plus":
   * The set of factors that are scheduled for relinearization because one of their variable deltas has exceed the
   * relinearization threshold.
   *
   * "Relin":
   * The "Relin-Plus" set minus any "Involved" factor/variable
   *
   *
   * ~~Categorical Definitions~~
   * These definitions do not have a parallel variable set. Rather these definitions are strict as written
   *
   * "Convex":
   * The set of all factors that are "Convex". i.e. who's \mu parameter has not yet converged.
   * Elements may overlap with many of the other sets as defined above.
   *
   * "Converged":
   * The set of all factors that are not known-inliers, and not convex. These are factors that were once convex, and
   * have converged with respect to their mu parameters.
   *
   * "Known-Inliers":
   * The set of all factors that the user has marked to be inliers (usually odometry).
   *
   **/

  /** INTERFACE **/
 public:
  /// @brief Constructor
  RISAM2(RISAM2Params params) : ISAM2(params.isam2_params), risam_params_(params) {}

  /** @brief Update Interface. See ISAM2 docs for details
   * @param known_inliers: flags to solve that all factors in newFactors are inliers, and should be always be considered
   * with quadratic cost.
   */
  ISAM2Result update(const NonlinearFactorGraph& newFactors = NonlinearFactorGraph(), const Values& newTheta = Values(),
                     bool known_inliers = false, const FactorIndices& removeFactorIndices = FactorIndices(),
                     const boost::optional<FastMap<Key, int> >& constrainedKeys = boost::none,
                     const boost::optional<FastList<Key> >& noRelinKeys = boost::none,
                     const boost::optional<FastList<Key> >& extraReelimKeys = boost::none,
                     bool force_relinearize = false);

  /** @brief Update Interface. See ISAM2 docs for details
   * @param known_inliers: flags to solve that all factors in newFactors are inliers, and should be always be considered
   * with quadratic cost.
   */
  ISAM2Result update(const NonlinearFactorGraph& newFactors, const Values& newTheta, bool known_inliers,
                     const ISAM2UpdateParams& updateParams);

  /// @brief Recalculate Interface. See ISAM2 docs for details
  void recalculate(const ISAM2UpdateParams& updateParams, const KeySet& relinKeys, ISAM2Result* result,
                   const KeySet& updateDirectlyInvolvedKeys);

  /** @brief recalculateBatch Interface. See ISAM2 docs for details
   * @param updateInvolvedKeys: The set of "Update-Involved" keys. (see Terms and Defs.)
   */
  void recalculateBatch(const ISAM2UpdateParams& updateParams, const KeySet& updateInvolvedKeys,
                        KeySet* affectedKeysSet, ISAM2Result* result);

  /** @brief recalculateIncremental Interface. See ISAM2 docs for details
   * @param updateInvolvedKeys: The set of "Update-Involved" keys. (see Terms and Defs.)
   */
  void recalculateIncremental(const ISAM2UpdateParams& updateParams, const KeySet& relinKeys,
                              const FastList<Key>& affectedKeys, const KeySet& updateInvolvedKeys,
                              KeySet* affectedKeysSet, Cliques* orphans, ISAM2Result* result);

  /** @brief relinearizes a factor based on solver state.
   * @param factor_idx: the index of the factor to relinearize
   * @param updateInvolvedKeys: The set of "Update-Involved" keys. (see Terms and Defs.)
   * @param relinKeys: The set of keys marked for relinearization due to delta threshold.
   *
   * Relinearization is preformed as follows for every factor:
   * - If "Update-Involved":                Quadratic
   * - If "Known-Inliers":                  Quadratic
   * - If "Convex-Involved" & "Convex":     Robust-Cost
   * - If "Convex-Involved" & "Converged":  Cached-Cost
   * - If "Relin" & not "Involved":         Robust-Cost
   */
  GaussianFactor::shared_ptr relinearizeFactor(const size_t factor_idx, const KeySet& updateInvolvedKeys,
                                               const KeySet& relinKeys);

  /** @brief relinearizeAffectedFactors Interface. See ISAM2Impl docs for details
   * @param updateInvolvedKeys: The set of "Update-Involved" keys. (see Terms and Defs.)
   */
  GaussianFactorGraph relinearizeAffectedFactors(const ISAM2UpdateParams& updateParams,
                                                 const FastList<Key>& affectedKeys, const KeySet& relinKeys,
                                                 const KeySet& updateInvolvedKeys);

  /** @brief pushBackFactors. See ISAM2Impl docs for details
   * @param known_inliers: flags to solve that all factors in newFactors are inliers, and should be always be considered
   * with quadratic cost.
   */
  void pushBackFactors(const NonlinearFactorGraph& newFactors, bool known_inliers,
                       NonlinearFactorGraph* nonlinearFactors, GaussianFactorGraph* linearFactors,
                       VariableIndex* variableIndex, FactorIndices* newFactorsIndices, KeySet* keysWithRemovedFactors,
                       ISAM2UpdateParams updateParams);

  /** @brief Computes the next delta
   * @param forceFullSolve: ignore wildfire thresh and compute the delta for all variables
   */
  void updateDelta(bool forceFullSolve = false) const;

  double robustError(boost::optional<VectorValues> delta = boost::none) const;
  double robustError(Values vals) const;
  void iterateToConvergence();
  void updateMuInitOnConvergence();

  VectorValues& getDelta() const;
  Values calculateEstimate() const;

  /** @brief Bayes Tree Traversal Functions
   * These functions are used to traverse a bayes tree starting at cliques that include any key in param keys in
   * their frontal set, and along all paths to the root.
   *
   * This traversal accumulates all keys in the frontal set of touched cliques.
   *
   * This traversal is necessary for computing the "Update-Involved" set without modifying the bayes tree.
   */
  KeySet traverseTop(const KeyVector& keys);
  void traversePath(KeySet& traversedKeys, ISAM2::sharedClique clique);
  void traverseClique(KeySet& traversedKeys, ISAM2::sharedClique clique);

  /// @brief Writes bayes tree starting at clique as dot file to given stream
  void dot(std::ostream& s, sharedClique clique, const KeyFormatter& keyFormatter = DefaultKeyFormatter,
           int parentnum = 0) const;

  /// @brief Writes bayes tree as dot file to given stream
  void dot(std::ostream& os, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /// @brief Writes bayes tree as dot file to file
  void saveGraph(const std::string& filename, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /// @brief Writes bayes tree as dot to string
  std::string dot(const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;
};
}  // namespace risam