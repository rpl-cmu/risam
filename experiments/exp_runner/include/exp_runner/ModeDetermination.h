/** Helpers to figure out what modes are engaged and dis-engaged for different methods
 *
 * Author: Dan McGann
 * Date: April 2022
 */
#pragma once
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "exp_runner/Runner.h"

namespace exp_runner {
/// @brief Structure for storing information about one ambiguous loop closure event
struct MHInfo {
  MHInfo(size_t correct_mode, size_t seq_idx, gtsam::NonlinearFactorGraph& fg)
      : correct_mode_idx(correct_mode), seq_idx(seq_idx), factors(fg) {}
  size_t correct_mode_idx;              /// @brief the index of the correct mode
  size_t seq_idx;                       /// @brief The index in the Mode sequence that this loop closure corresponds to
  gtsam::NonlinearFactorGraph factors;  /// @brief the factors assciated with this loop closure event
};

/// @brief functor type for determining if a factor is in inlier
typedef std::function<bool(const gtsam::NonlinearFactor::shared_ptr&, const gtsam::Values&)> InlierCheckFunc;

/** Computes a mode sequence by looking at active factors and factors listed in a history associated with loop closures
 * @Mutates mode_seq
 */
void computeModeSequence(ModeSequence& mode_seq, const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
                         const std::vector<MHInfo>& mh_info, const InlierCheckFunc& inlier_check_func) {
  // Compute all inlier factors
  gtsam::NonlinearFactorGraph inlier_factors;
  for (size_t i = 0; i < graph.size(); i++) {
    auto factor = graph[i];
    if (factor && inlier_check_func(factor, values)) {
      inlier_factors.push_back(factor);
    }
  }

  // Update mode for all loop closure events
  for (auto& lci : mh_info) {
    // Accumulate all the modes from this event that are active
    std::vector<int> indices_of_active_factors;
    for (int i = 0; i < lci.factors.size(); i++) {
      // If the mode factor is found accumulate its index
      if (std::find_if(inlier_factors.begin(), inlier_factors.end(),
                       [&lci, &i](const gtsam::NonlinearFactor::shared_ptr& a) {
                         return lci.factors[i]->equals(*a);
                       }) != inlier_factors.end()) {
        indices_of_active_factors.push_back(i);
      }
    }

    // Determine if the result is any of the special cases
    int mode_selection;

    // SPECIAL CASE: Correct mode is null hypothesis (nh is always last mode)
    if (indices_of_active_factors.size() == 0) {
      mode_selection = lci.factors.size();
    }
    // SPECIAL CASE: More than one mode active
    else if (indices_of_active_factors.size() > 1) {
      // CASE 1 multiple active modes and correct mode is one of them (-1)
      if (std::find(indices_of_active_factors.begin(), indices_of_active_factors.end(), lci.correct_mode_idx) !=
          indices_of_active_factors.end()) {
        mode_selection = -1;
      }
      // CASE 2 multiple active modes and correct mode is NOT one of them (-2)
      else {
        mode_selection = -2;
      }
    }
    // standard: one active mode
    else {
      mode_selection = indices_of_active_factors.front();
    }

    // Finally update the mode sequence for this event
    mode_seq[lci.seq_idx] = mode_selection;
  }
}

/** Common Inlier Check Functions **/
bool chiSqInlierCheck(const gtsam::NonlinearFactor::shared_ptr& factor, const gtsam::Values& values) {
  auto nmfactor = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(factor);
  auto err = sqrt(nmfactor->noiseModel()->squaredMahalanobisDistance(nmfactor->unwhitenedError(values))); // Mahalanobis Distance
  //auto err = std::sqrt(2.0 * nmfactor->error(values)); 
  double thresh = boost::math::quantile(boost::math::chi_squared_distribution<double>(factor->dim()), 0.95);
  return err < thresh;
}

bool trueInlierCheck(const gtsam::NonlinearFactor::shared_ptr& factor, const gtsam::Values& values) { return true; }

}  // namespace exp_runner