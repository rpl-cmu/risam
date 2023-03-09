/** @brief Implementation for DC solver
 *
 *
 * @author Dan McGann
 * @date Feb 2022
 */
#pragma once
#include "dcsam/DCMixtureFactor.h"
#include "dcsam/DiscretePriorFactor.h"
#include "exp_runner/DCRunner.h"

using gtsam::symbol_shorthand::D;

namespace exp_runner {
/***************************************************************************/
template <class POSE_TYPE>
void DCRunner<POSE_TYPE>::init() {
  gtsam::ISAM2Params params;
  params.setOptimizationParams(gtsam::ISAM2DoglegParams());
  solver_ = boost::make_shared<dcsam::DCSAM>(params);
}

/***************************************************************************/
template <class POSE_TYPE>
void DCRunner<POSE_TYPE>::handlePrior(irl::Prior<POSE_TYPE>& prior) {
  // ASSUME INLIER PRIORS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  dcsam::HybridFactorGraph new_factors;
  gtsam::Values new_values;
  dcsam::DiscreteValues new_discrete_values;
  // Initialize the factor based on the selected mode
  new_factors.push_nonlinear(prior.asFactor(0));
  new_values = prior.getGenerator(0)(current_estimate_.continuous);

  // Update the underlying solver
  solver_->update(new_factors, new_values);
  current_estimate_ = solver_->calculateEstimate();
  current_mode_sequence_.push_back(0);
}

/***************************************************************************/
template <class POSE_TYPE>
void DCRunner<POSE_TYPE>::handleOdometry(irl::Odometry<POSE_TYPE>& odom) {
  // ASSUME INLIER ODOMETRY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  dcsam::HybridFactorGraph new_factors;
  gtsam::Values new_values;
  new_factors.push_nonlinear(odom.asFactor(0));
  solver_->update(new_factors, odom.getGenerator(0)(current_estimate_.continuous));
  current_mode_sequence_.push_back(0);
  // Update current estimate info
  current_estimate_ = solver_->calculateEstimate();
}

/***************************************************************************/
template <class POSE_TYPE>
void DCRunner<POSE_TYPE>::handleLoop(irl::Loop<POSE_TYPE>& loop) {
  // Handle Single Hypothesis case
  if (loop.num_modes == 1) {
    dcsam::HybridFactorGraph new_factors;
    new_factors.push_nonlinear(loop.asFactor(0));
    solver_->update(new_factors, loop.getGenerator(0)(current_estimate_.continuous));
    current_mode_sequence_.push_back(0);
  }
  // Handle MultiHypothesis Case
  else {
    dcsam::HybridFactorGraph new_factors;
    gtsam::Values new_values;
    DiscreteValues new_discrete_values;
    gtsam::DiscreteKey discrete_key(D(discrete_var_counter_++), 2);
    gtsam::FastVector<std::function<gtsam::Values(const gtsam::Values&)>> generators;

    for (size_t i = 0; i < loop.num_modes; i++) {
      auto loop_measure = boost::dynamic_pointer_cast<irl::LoopMeasure<POSE_TYPE>>(loop.measurements[i]);
      if (loop_measure) {
        // Add discrete variable for this mode
        new_discrete_values[discrete_key.first] = 0;  // Assume inlier at first
        // Add the Loop factor for this mode

        std::vector<gtsam::BetweenFactor<POSE_TYPE>> components;
        components.push_back(  // Inlier Measurement
            gtsam::BetweenFactor<POSE_TYPE>(loop.pose_a_key, loop_measure->pose_b_key, loop_measure->rel_pose,
                                            gtsam::noiseModel::Gaussian::Covariance(loop_measure->covariance)));
        components.push_back(  // Outlier Measurement
            gtsam::BetweenFactor<POSE_TYPE>(loop.pose_a_key, loop_measure->pose_b_key, loop_measure->rel_pose,
                                            gtsam::noiseModel::Isotropic::Sigma(loop_measure->covariance.rows(), 1.6e7)));

        new_factors.push_dc(dcsam::DCMixtureFactor<gtsam::BetweenFactor<POSE_TYPE>>(components[0].keys(), discrete_key,
                                                                                    components, false));
        new_factors.push_discrete(dcsam::DiscretePriorFactor(discrete_key, {1 - 10e-7, 10e-7}));
      }
      generators.push_back(loop.getGenerator(i));
    }
    // Determine linearization point
    new_values.insert(generators[0](current_estimate_.continuous));

    // Preform update
    solver_->update(new_factors, new_values, new_discrete_values);

    mh_measures_infos_.push_back(std::make_pair(current_mode_sequence_.size(), discrete_key));
    current_mode_sequence_.push_back(0);
  }

  // Update current estimate info
  current_estimate_ = solver_->calculateEstimate();
}

/***************************************************************************/
template <class POSE_TYPE>
MethodEstimateResults DCRunner<POSE_TYPE>::getEstimate() {
  MethodEstimateResults result;
  // current_estimate_.discrete.print();
  result.push_back(current_estimate_.continuous);
  return result;
}

/***************************************************************************/
template <class POSE_TYPE>
MethodModeSeqResults DCRunner<POSE_TYPE>::getModeSequence() {
  MethodModeSeqResults result;
  ModeSequence seq;
  for (auto& pair : mh_measures_infos_) {
    current_mode_sequence_[pair.first] = current_estimate_.discrete.at(pair.second.first);
  }
  seq.assign(current_mode_sequence_.begin(), current_mode_sequence_.end());
  result.push_back(seq);
  return result;
}

/***************************************************************************/
template <class POSE_TYPE>
std::string DCRunner<POSE_TYPE>::getName() {
  return "dcsam";
}
}  // namespace exp_runner