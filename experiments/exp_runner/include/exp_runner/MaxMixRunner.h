/** @brief Experiment Interface for MaxMixture SLAM
 *
 * @author Dan McGann
 * @date January 2022
 */
#pragma once
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "exp_runner/Runner.h"

namespace exp_runner {

class MaxMixtureFactor : public gtsam::NonlinearFactor {
  /** Public Types **/
 public:
  typedef boost::shared_ptr<MaxMixtureFactor> shared_ptr;
  typedef gtsam::FastVector<NonlinearFactor::shared_ptr> MixtureModel;

  /** Protected Fields **/
 protected:
  /// @brief The mixture model of Factors making up this MaxMixtureFactor.
  MixtureModel mixture_model_;
  /// @brief log(weight) assigned to each of the mixture components
  gtsam::FastVector<double> log_weights_;
  /// @brief the Negative Log Normalization Constants for each Model @see computeNegLogNormalizationConstant
  gtsam::FastVector<double> negative_log_normalizers_;

 public:
  /** @brief MaxMixtureFactor Constructor
   *  @param keys: All keys referenced by this factor. The set-union of keys of all its components.
   *  @param mixture_components: The factors describing the modes of this max mixture model. Mixture Components must be
   * NoiseModelFactors with Gaussian Noise Models. Additionally, all Mixture Components are expected to linearize to the
   * same number of rows.
   *  @param weights: the importance weights assigned to each mode
   */
  MaxMixtureFactor(const gtsam::KeyVector& keys, const MixtureModel& mixture_components,
                   const gtsam::FastVector<double>& weights);

  /* PUBLIC INTERFACE */
 public:
  /** @brief Computes the negative log normalization constant of for the linearized (gaussian) component.
   * Computes: -\log\left(\frac{1}{(2\pi)^{d/2}|\Sigma_j|^{1/2}} \exp \left\{ -\frac{1}{2}||h(x_i) -
   * z_i||^2_{\Sigma_j} \right\} \right)
   *  @param component: The mode for which to compute its constant.
   */
  static double computeNegLogNormalizationConstant(const gtsam::NonlinearFactor::shared_ptr& component);

  /** @brief Computes the Index and normalized error for the component that is most probable given
   * the estimate.
   *  @param current_estimate: The current estimate at which to evaluate component probabilities
   */
  std::pair<size_t, double> computeMaxIndexAndNormalizedError(const gtsam::Values& current_estimate) const;

  /// @brief Returns the number of modes in this factor
  size_t nrModes() const;

  /// @brief Returns the dimensionality of the factor (rows on linearization)
  size_t dim() const override;

  /** @brief Computes the normalized error for the factor 0.5 the squared mahalanobis distance
   *  @param current_estimate: the variable estimate at which to compute the error
   */
  double error(const gtsam::Values& current_estimate) const override;

  /** @brief Linearize this factor at a specific point
   *  @param current_estimate: the variable estimate at which to linearize the Factor
   */
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& current_estimate) const override;

  /// @brief Creates a deep copy of this Factor
  gtsam::NonlinearFactor::shared_ptr clone() const override;

  /// @brief Prints to stdout information about this factor for params see gtsam print function interface
  void print(const std::string& s = "", const gtsam::KeyFormatter& kf = gtsam::DefaultKeyFormatter) const override;
};

template <class POSE_TYPE>
class MaxMixRunner : public Method<POSE_TYPE> {
  /** FIELDS **/
 private:
  /// @brief the actual solver
  boost::shared_ptr<gtsam::ISAM2> solver_;
  /// @brief internal tracker for current estimate
  gtsam::Values current_estimate_;
  /// @brief Current Mode Sequence
  ModeSequence current_mode_sequence_;
  /// @brief Flag to get the mode from the max mixture factor
  gtsam::FastVector<bool> get_mode_from_factor_;

  /** PARAMETERS **/
 private:
  /// @brief Weight (0,1] for the null hypo mode.
  const double null_hypo_weight_{1e-7};
  /// @brief Scale factor applied to measurement covariance for null hypothesis factors
  const double null_hypo_cov_scale_{1.6e7};

  /** HELPERS **/
 protected:
  /// @brief Internal Handler used for both Odometry and Loop Closures
  void handleBetweenPose(gtsam::Key start_key, gtsam::Key end_key, size_t num_modes,
                         std::vector<irl::Measurement::shared_ptr> measurements, bool use_best_initialization);

  /** INTERFACE IMPL **/
 public:
  void init() override;
  void handlePrior(irl::Prior<POSE_TYPE>& prior) override;
  void handleOdometry(irl::Odometry<POSE_TYPE>& odom) override;
  void handleLoop(irl::Loop<POSE_TYPE>& loop) override;
  MethodEstimateResults getEstimate() override;
  MethodModeSeqResults getModeSequence() override;
  std::string getName() override;
};
}  // namespace exp_runner

#include "exp_runner/MaxMixRunner-inl.h"