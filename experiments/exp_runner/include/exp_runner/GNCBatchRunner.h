/** @brief Experiment Interface for Graduated Non-Convexity
 *
 * @author Dan McGann
 * @date Feb 2022
 */
#pragma once
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "exp_runner/ModeDetermination.h"
#include "exp_runner/Runner.h"

namespace exp_runner {

template <class POSE_TYPE>
class GNCBatchRunner : public Method<POSE_TYPE> {
  /** FIELDS **/
 private:
  /// @brief iteration_counter
  size_t iter{0};
  /// @brief The graph that we are optimizing
  gtsam::NonlinearFactorGraph graph_;
  /// @brief Initial Conditions used at each batch solve
  gtsam::Values initial_estimates_;

  /// @brief internal tracker for current estimate
  gtsam::Values current_estimate_;
  /// @brief the current assignment of modes
  ModeSequence current_mode_sequence_;
  /// @brief list of known inlier factors (i.e. odometry) that we should not convexify
  gtsam::GncParams<gtsam::GaussNewtonParams>::IndexVector known_inliers_;
  /// @brief info about each loop closure
  std::vector<MHInfo> mh_infos_;

  /** PARAMETERS **/
 private:
  /// @brief the parameters to use for Optimization
  gtsam::GaussNewtonParams optimizer_params_;

  /** HELPERS **/
 protected:
  /** @brief Checks iteration count and runs optimizer if needed
   */
  void handleUpdate();

  /** INTERFACE IMPL **/
 public:
  void init() override;
  void handlePrior(irl::Prior<POSE_TYPE>& prior) override;
  void handleOdometry(irl::Odometry<POSE_TYPE>& odom) override;
  void handleLoop(irl::Loop<POSE_TYPE>& loop) override;
  void handleFinal() override;
  MethodEstimateResults getEstimate() override;
  MethodModeSeqResults getModeSequence() override;
  std::string getName() override;
};
}  // namespace exp_runner

#include "exp_runner/GNCBatchRunner-inl.h"