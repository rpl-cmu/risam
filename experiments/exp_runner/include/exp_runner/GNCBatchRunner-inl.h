/** @brief Implementation for GNC solver
 *
 * @author Dan McGann
 * @date Feb 2022
 */
#pragma once
#include "exp_runner/GNCBatchRunner.h"

namespace exp_runner {
/***************************************************************************/
template <class POSE_TYPE>
void GNCBatchRunner<POSE_TYPE>::init() {}

/***************************************************************************/
template <class POSE_TYPE>
void GNCBatchRunner<POSE_TYPE>::handleUpdate() {
  auto params = gtsam::GncParams<gtsam::GaussNewtonParams>(optimizer_params_);
  params.setKnownInliers(known_inliers_);
  gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>> optimizer(graph_, initial_estimates_, params);
  current_estimate_ = optimizer.optimize();
  initial_estimates_.update(current_estimate_);
}

/***************************************************************************/
template <class POSE_TYPE>
void GNCBatchRunner<POSE_TYPE>::handlePrior(irl::Prior<POSE_TYPE>& prior) {
  if (prior.num_modes == 1) {
    known_inliers_.push_back(graph_.nrFactors());
    graph_.push_back(prior.asFactor(0));
    initial_estimates_.insert(prior.getGenerator(0)(initial_estimates_));
    current_mode_sequence_.push_back(0);
    handleUpdate();
  } else {
    gtsam::NonlinearFactorGraph prior_factors;
    for (size_t i = 0; i < prior.num_modes; i++) {
      prior_factors.push_back(prior.asFactor(i));
      initial_estimates_.insert(prior.getGenerator(i)(initial_estimates_));
    }
    // Update the loop closure info
    mh_infos_.push_back(MHInfo(prior.correct_mode_idx, current_mode_sequence_.size(), prior_factors));
    current_mode_sequence_.push_back(0);  // Push back temp index this will get updated in the next call
    graph_.push_back(prior_factors);

    handleUpdate();
  }
}

/***************************************************************************/
template <class POSE_TYPE>
void GNCBatchRunner<POSE_TYPE>::handleOdometry(irl::Odometry<POSE_TYPE>& odom) {
  if (odom.num_modes == 1) {
    known_inliers_.push_back(graph_.nrFactors());
    graph_.push_back(odom.asFactor(0));
    initial_estimates_.insert(odom.getGenerator(0)(initial_estimates_));
    current_mode_sequence_.push_back(0);
    handleUpdate();
  } else {
    gtsam::NonlinearFactorGraph odom_factors;
    for (size_t i = 0; i < odom.num_modes; i++) {
      auto factor = odom.asFactor(i);
      if (factor) {
        odom_factors.push_back(factor);
        initial_estimates_.insert(odom.getGenerator(i)(initial_estimates_));
      }
    }
    // Update the loop closure info
    mh_infos_.push_back(MHInfo(odom.correct_mode_idx, current_mode_sequence_.size(), odom_factors));
    current_mode_sequence_.push_back(0);  // Push back temp index this will get updated in the next call
    graph_.push_back(odom_factors);
    handleUpdate();
  }
}

/***************************************************************************/
template <class POSE_TYPE>
void GNCBatchRunner<POSE_TYPE>::handleLoop(irl::Loop<POSE_TYPE>& loop) {
  gtsam::NonlinearFactorGraph loop_factors;
  for (size_t i = 0; i < loop.num_modes; i++) {
    auto factor = loop.asFactor(i);
    if (factor) {
      loop_factors.push_back(factor);
      initial_estimates_.insert(loop.getGenerator(i)(initial_estimates_));
    }
  }
  mh_infos_.push_back(MHInfo(loop.correct_mode_idx, current_mode_sequence_.size(), loop_factors));
  current_mode_sequence_.push_back(0);  // Push back temp index this will get updated in the next call
  graph_.push_back(loop_factors);
  handleUpdate();
}

/***************************************************************************/
template <class POSE_TYPE>
void GNCBatchRunner<POSE_TYPE>::handleFinal() {
  auto params = gtsam::GncParams<gtsam::GaussNewtonParams>(optimizer_params_);
  params.setKnownInliers(known_inliers_);
  gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>> optimizer(graph_, initial_estimates_, params);
  current_estimate_ = optimizer.optimize();
}

/***************************************************************************/
template <class POSE_TYPE>
MethodEstimateResults GNCBatchRunner<POSE_TYPE>::getEstimate() {
  MethodEstimateResults result;
  result.push_back(current_estimate_);
  return result;
}

/***************************************************************************/
template <class POSE_TYPE>
MethodModeSeqResults GNCBatchRunner<POSE_TYPE>::getModeSequence() {
  MethodModeSeqResults result;
  ModeSequence seq;
  computeModeSequence(current_mode_sequence_, graph_, current_estimate_, mh_infos_, &chiSqInlierCheck);
  seq.assign(current_mode_sequence_.begin(), current_mode_sequence_.end());
  result.push_back(seq);
  return result;
}

/***************************************************************************/
template <class POSE_TYPE>
std::string GNCBatchRunner<POSE_TYPE>::getName() {
  return "gnc-batch";
}
}  // namespace exp_runner