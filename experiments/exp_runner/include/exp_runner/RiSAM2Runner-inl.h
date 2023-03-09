/** @brief Implementation for Baseline Experiment Runner
 * This runner always uses the correct mode, and is intendend to be treatede as a reference solution
 *
 * @author Dan McGann
 * @date January 2022
 */
#pragma once
#include <boost/math/distributions/chi_squared.hpp>

#include "risam/RISAM2.h"
namespace exp_runner {

/***************************************************************************/
template <class POSE_TYPE>
void RiSAM2Runner<POSE_TYPE>::init() {
  solver_ = boost::make_shared<risam::RISAM2>(riparams_);
}

/***************************************************************************/
template <class POSE_TYPE>
void RiSAM2Runner<POSE_TYPE>::handleFinal() {
  for (size_t i = 0; i < 20; i++) {
    solver_->update(NonlinearFactorGraph(), Values(), false, ISAM2UpdateParams());
  }
  current_estimate_ = solver_->calculateEstimate();
}

/***************************************************************************/
template <class POSE_TYPE>
void RiSAM2Runner<POSE_TYPE>::handlePrior(irl::Prior<POSE_TYPE>& prior) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  if (prior.num_modes == 1) {
    new_factors.push_back(prior.asFactor(0));
    new_values.insert(prior.getGenerator(0)(current_estimate_));
    current_mode_sequence_.push_back(0);
    auto result = solver_->update(new_factors, new_values, true);
    current_estimate_ = solver_->calculateEstimate();
  } else {
    for (size_t i = 0; i < prior.num_modes; i++) {
      new_factors.push_back(prior.asGNCFactor(i, kernel_shape_param_));
    }
    new_values.insert(prior.getGenerator(0)(current_estimate_));  // TODO (dan) better method?
    mh_infos_.push_back(MHInfo(prior.correct_mode_idx, current_mode_sequence_.size(), new_factors));
    current_mode_sequence_.push_back(0);
    auto result = solver_->update(new_factors, new_values, false);
    current_estimate_ = solver_->calculateEstimate();
  }
}

/***************************************************************************/
template <class POSE_TYPE>
void RiSAM2Runner<POSE_TYPE>::handleOdometry(irl::Odometry<POSE_TYPE>& odom) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;

  if (odom.num_modes == 1) {
    new_factors.push_back(odom.asFactor(0));
    new_values.insert(odom.getGenerator(0)(current_estimate_));
    current_mode_sequence_.push_back(0);
    auto result = solver_->update(new_factors, new_values, true);
    current_estimate_ = solver_->calculateEstimate();
  } else {
    for (size_t i = 0; i < odom.num_modes; i++) {
      auto factor = odom.asGNCFactor(i, kernel_shape_param_);
      if (factor) {
        new_factors.push_back(factor);
      }
    }
    new_values.insert(odom.getGenerator(0)(current_estimate_));  // TODO (dan) better method?
    mh_infos_.push_back(MHInfo(odom.correct_mode_idx, current_mode_sequence_.size(), new_factors));
    current_mode_sequence_.push_back(0);
    auto result = solver_->update(new_factors, new_values, false);
    current_estimate_ = solver_->calculateEstimate();
  }
}

/***************************************************************************/
template <class POSE_TYPE>
void RiSAM2Runner<POSE_TYPE>::handleLoop(irl::Loop<POSE_TYPE>& loop) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  for (size_t i = 0; i < loop.num_modes; i++) {
    auto factor = loop.asGNCFactor(i, kernel_shape_param_);
    if (factor) {
      new_factors.push_back(factor);
      new_values.insert(loop.getGenerator(i)(current_estimate_));
    }
  }
  mh_infos_.push_back(MHInfo(loop.correct_mode_idx, current_mode_sequence_.size(), new_factors));
  current_mode_sequence_.push_back(0);  // Push back temp index this will get updated in the next call
  auto result = solver_->update(new_factors, new_values, false);
  current_estimate_ = solver_->calculateEstimate();
}

/***************************************************************************/
template <class POSE_TYPE>
MethodEstimateResults RiSAM2Runner<POSE_TYPE>::getEstimate() {
  MethodEstimateResults result;
  result.push_back(current_estimate_);
  return result;
}

/***************************************************************************/
template <class POSE_TYPE>
MethodModeSeqResults RiSAM2Runner<POSE_TYPE>::getModeSequence() {
  MethodModeSeqResults result;
  computeModeSequence(current_mode_sequence_, solver_->getFactorsUnsafe(), current_estimate_, mh_infos_,
                      &chiSqInlierCheck);
  result.push_back(current_mode_sequence_);
  return result;
}

/***************************************************************************/
template <class POSE_TYPE>
std::string RiSAM2Runner<POSE_TYPE>::getName() {
  return name_;
}

}  // namespace exp_runner
