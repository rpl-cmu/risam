/** @brief Experiment Interface for pseudo gt runner
 *
 * @author Dan McGann
 * @date January 2022
 */
#pragma once
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "exp_runner/Runner.h"

namespace exp_runner {

template <class POSE_TYPE>
class PseudoGtRunner : public Method<POSE_TYPE> {
  /** FIELDS **/
 private:
  /// @brief the solver
  boost::shared_ptr<gtsam::ISAM2> solver_;
  /// @brief internal tracker for current estimate
  gtsam::Values current_estimate_;
  /// @brief Current Mode Sequence
  ModeSequence current_mode_sequence_;
  size_t count = 0;

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

#include "exp_runner/PseudoGtRunner-inl.h"