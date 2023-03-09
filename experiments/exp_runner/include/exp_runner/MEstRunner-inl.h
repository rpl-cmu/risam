/** @brief Implementation for MEstimator Runner
 *
 * @author Dan McGann
 * @date May 2022
 */
#pragma once

#include "exp_runner/MEstRunner.h"
namespace exp_runner {

/***************************************************************************/
template <class POSE_TYPE>
void MEstRunner<POSE_TYPE>::init() {
  gtsam::ISAM2Params params;
  params.setOptimizationParams(gtsam::ISAM2DoglegParams(0.1, 1e-10, gtsam::DoglegOptimizerImpl::TrustRegionAdaptationMode::ONE_STEP_PER_ITERATION));
  solver_ = boost::make_shared<gtsam::ISAM2>(params);
}

/***************************************************************************/
template <class POSE_TYPE>
void MEstRunner<POSE_TYPE>::handleFinal() {
  for (size_t i = 0; i < 20; i++) {
    solver_->update();
  }
  current_estimate_ = solver_->calculateBestEstimate();
}

/***************************************************************************/
template <class POSE_TYPE>
void MEstRunner<POSE_TYPE>::handlePrior(irl::Prior<POSE_TYPE>& prior) {
  gtsam::NonlinearFactorGraph new_factors;
  if (prior.num_modes == 1) {
    new_factors.push_back(prior.asFactor(0));
  } else {
    for (size_t i = 0; i < prior.num_modes; i++) {
      auto factor = prior.asFactor(i);
      if (factor) {
        auto nm_factor = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(factor);
        new_factors.push_back(nm_factor->cloneWithNewNoiseModel(
            gtsam::noiseModel::Robust::Create(robust_kernel_, nm_factor->noiseModel())));
      }
    }
    // Update the loop closure info
    mh_infos_.push_back(MHInfo(prior.correct_mode_idx, current_mode_sequence_.size(), new_factors));
  }

  gtsam::Values new_values = prior.getGenerator(0)(current_estimate_);
  solver_->update(new_factors, new_values);
  current_estimate_ = solver_->calculateEstimate();
  current_mode_sequence_.push_back(0);
}

/***************************************************************************/
template <class POSE_TYPE>
void MEstRunner<POSE_TYPE>::handleOdometry(irl::Odometry<POSE_TYPE>& odom) {
  gtsam::NonlinearFactorGraph new_factors;
  if (odom.num_modes == 1) {
    new_factors.push_back(odom.asFactor(0));
  } else {
    for (size_t i = 0; i < odom.num_modes; i++) {
      auto factor = odom.asFactor(i);
      if (factor) {
        auto nm_factor = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(factor);
        new_factors.push_back(nm_factor->cloneWithNewNoiseModel(
            gtsam::noiseModel::Robust::Create(robust_kernel_, nm_factor->noiseModel())));
      }
    }
    // Update the loop closure info
    mh_infos_.push_back(MHInfo(odom.correct_mode_idx, current_mode_sequence_.size(), new_factors));
  }

  gtsam::Values new_values = odom.getGenerator(0)(current_estimate_);
  solver_->update(new_factors, new_values);
  current_estimate_ = solver_->calculateEstimate();
  current_mode_sequence_.push_back(0);
}

/***************************************************************************/
template <class POSE_TYPE>
void MEstRunner<POSE_TYPE>::handleLoop(irl::Loop<POSE_TYPE>& loop) {
  gtsam::NonlinearFactorGraph new_factors;
  if (loop.num_modes == 1) {
    new_factors.push_back(loop.asFactor(0));
  } else {
    for (size_t i = 0; i < loop.num_modes; i++) {
      auto factor = loop.asFactor(i);
      if (factor) {
        auto nm_factor = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(factor);
        new_factors.push_back(nm_factor->cloneWithNewNoiseModel(
            gtsam::noiseModel::Robust::Create(robust_kernel_, nm_factor->noiseModel())));
      }
    }
    // Update the loop closure info
    mh_infos_.push_back(MHInfo(loop.correct_mode_idx, current_mode_sequence_.size(), new_factors));
  }

  gtsam::Values new_values = loop.getGenerator(0)(current_estimate_);
  solver_->update(new_factors, new_values);
  current_estimate_ = solver_->calculateEstimate();
  current_mode_sequence_.push_back(0);
}

/***************************************************************************/
template <class POSE_TYPE>
MethodEstimateResults MEstRunner<POSE_TYPE>::getEstimate() {
  MethodEstimateResults result;
  result.push_back(current_estimate_);
  return result;
}

/***************************************************************************/
template <class POSE_TYPE>
MethodModeSeqResults MEstRunner<POSE_TYPE>::getModeSequence() {
  MethodModeSeqResults result;
  ModeSequence seq;
  computeModeSequence(current_mode_sequence_, solver_->getFactorsUnsafe(), current_estimate_, mh_infos_,
                      &chiSqInlierCheck);
  seq.assign(current_mode_sequence_.begin(), current_mode_sequence_.end());
  result.push_back(seq);
  return result;
}

/***************************************************************************/
template <class POSE_TYPE>
std::string MEstRunner<POSE_TYPE>::getName() {
  return kernel_name_;
}

}  // namespace exp_runner
