/** @brief Implementation for Baseline Experiment Runner
 * This runner always uses the correct mode, and is intendend to be treatede as a reference solution
 *
 * @author Dan McGann
 * @date January 2022
 */
#pragma once
#include <boost/math/distributions/chi_squared.hpp>

namespace exp_runner {

/***************************************************************************/
template <class POSE_TYPE>
void PseudoGtRunner<POSE_TYPE>::init() {
  gtsam::ISAM2Params params;
  params.enableDetailedResults = true;
  params.relinearizeThreshold = 0.01;
  params.setOptimizationParams(gtsam::ISAM2DoglegParams());
  solver_ = boost::make_shared<gtsam::ISAM2>(params);
}

/***************************************************************************/
template <class POSE_TYPE>
void PseudoGtRunner<POSE_TYPE>::handlePrior(irl::Prior<POSE_TYPE>& prior) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  new_factors.push_back(prior.asFactor(prior.correct_mode_idx));
  new_values = prior.getGenerator(prior.correct_mode_idx)(current_estimate_);
  auto result = solver_->update(new_factors, new_values);
  current_estimate_ = solver_->calculateEstimate();
  current_mode_sequence_.push_back(prior.correct_mode_idx);
}

/***************************************************************************/
template <class POSE_TYPE>
void PseudoGtRunner<POSE_TYPE>::handleOdometry(irl::Odometry<POSE_TYPE>& odom) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  new_factors.push_back(odom.asFactor(odom.correct_mode_idx));
  new_values = odom.getGenerator(odom.correct_mode_idx)(current_estimate_);
  auto result = solver_->update(new_factors, new_values);
  current_estimate_ = solver_->calculateEstimate();
  current_mode_sequence_.push_back(odom.correct_mode_idx);
}

/***************************************************************************/
template <class POSE_TYPE>
void PseudoGtRunner<POSE_TYPE>::handleLoop(irl::Loop<POSE_TYPE>& loop) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  new_factors.push_back(loop.asFactor(loop.correct_mode_idx));
  new_values = loop.getGenerator(loop.correct_mode_idx)(current_estimate_);
  auto result = solver_->update(new_factors, new_values);
  current_estimate_ = solver_->calculateEstimate();
  current_mode_sequence_.push_back(loop.correct_mode_idx);
}

/***************************************************************************/
template <class POSE_TYPE>
MethodEstimateResults PseudoGtRunner<POSE_TYPE>::getEstimate() {
  MethodEstimateResults result;
  result.push_back(current_estimate_);
  return result;
}

/***************************************************************************/
template <class POSE_TYPE>
MethodModeSeqResults PseudoGtRunner<POSE_TYPE>::getModeSequence() {
  MethodModeSeqResults result;
  ModeSequence seq;
  seq.assign(current_mode_sequence_.begin(), current_mode_sequence_.end());
  result.push_back(seq);
  return result;
}

/***************************************************************************/
template <class POSE_TYPE>
std::string PseudoGtRunner<POSE_TYPE>::getName() {
  return "pseudo-gt";
}

}  // namespace exp_runner
