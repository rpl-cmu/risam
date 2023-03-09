/** @brief Implementation for PCM solver
 *
 * @note ONLY WORKS FOR POSE GRAPH OPTIMIZATION
 *
 * @author Dan McGann
 * @date January 2022
 */
#pragma once
#include "exp_runner/PCMRunner.h"

namespace exp_runner {
/***************************************************************************/
template <class POSE_TYPE>
void PCMRunner<POSE_TYPE>::init() {
  KimeraRPGO::RobustSolverParams params;
  if (std::is_same<POSE_TYPE, gtsam::Pose2>::value) {
    params.setPcm2DParams(odom_threshold_, lc_threshold_, KimeraRPGO::Verbosity::VERBOSE);
    // params.setIncremental();
  } else {
    params.setPcm3DParams(odom_threshold_, lc_threshold_, KimeraRPGO::Verbosity::VERBOSE);
    // params.setIncremental();
  }
  params.use_gnc_ = false;
  solver_ = boost::make_shared<KimeraRPGO::RobustSolver>(params);
}

/***************************************************************************/
template <class POSE_TYPE>
void PCMRunner<POSE_TYPE>::handleUpdate() {
  solver_->update(new_factors_, new_values_);
  current_estimate_ = solver_->calculateEstimate();
  new_factors_ = gtsam::NonlinearFactorGraph();
  new_values_ = gtsam::Values();
}

/***************************************************************************/
template <class POSE_TYPE>
void PCMRunner<POSE_TYPE>::handlePrior(irl::Prior<POSE_TYPE>& prior) {
  // ASSUME INLIER PRIOR !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  new_factors_.push_back(prior.asFactor(0));
  gtsam::Values merged(current_estimate_);
  merged.insert(new_values_);
  new_values_.insert(prior.getGenerator(0)(merged));

  // Update the underlying solver
  current_mode_sequence_.push_back(0);
  handleUpdate();
}

/***************************************************************************/
template <class POSE_TYPE>
void PCMRunner<POSE_TYPE>::handleOdometry(irl::Odometry<POSE_TYPE>& odom) {
  // ASSUME INLIER ODOM !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  new_factors_.push_back(odom.asFactor(0));
  gtsam::Values merged(current_estimate_);
  merged.insert(new_values_);
  new_values_.insert(odom.getGenerator(0)(merged));

  // Update the underlying solver
  current_mode_sequence_.push_back(0);
  handleUpdate();
}

/***************************************************************************/
template <class POSE_TYPE>
void PCMRunner<POSE_TYPE>::handleLoop(irl::Loop<POSE_TYPE>& loop) {
  gtsam::NonlinearFactorGraph loop_factors;
  // PCM cannot handle "Either-Or" ambiguity, so add all potential loop closures
  for (size_t i = 0; i < loop.num_modes; i++) {
    auto factor = loop.asFactor(i);
    if (factor) {
      loop_factors.push_back(factor);
    }
  }

  // Update the loop closure info
  mh_infos_.push_back(MHInfo(loop.correct_mode_idx, current_mode_sequence_.size(), loop_factors));
  current_mode_sequence_.push_back(0);  // Push back temp index this will get updated in the next call
  new_factors_.push_back(loop_factors);
  handleUpdate();
}

/***************************************************************************/
template <class POSE_TYPE>
void PCMRunner<POSE_TYPE>::handleFinal() {
  solver_->update(new_factors_, new_values_);
  current_estimate_ = solver_->calculateEstimate();
}

/***************************************************************************/
template <class POSE_TYPE>
MethodEstimateResults PCMRunner<POSE_TYPE>::getEstimate() {
  MethodEstimateResults result;
  result.push_back(current_estimate_);
  return result;
}

/***************************************************************************/
template <class POSE_TYPE>
MethodModeSeqResults PCMRunner<POSE_TYPE>::getModeSequence() {
  MethodModeSeqResults result;
  ModeSequence seq;
  auto inlier_factors = solver_->getFactorsUnsafe();
  computeModeSequence(current_mode_sequence_, inlier_factors, current_estimate_, mh_infos_, &trueInlierCheck);
  seq.assign(current_mode_sequence_.begin(), current_mode_sequence_.end());
  result.push_back(seq);
  return result;
}

/***************************************************************************/
template <class POSE_TYPE>
std::string PCMRunner<POSE_TYPE>::getName() {
  return name_;
}
}  // namespace exp_runner