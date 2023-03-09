/** @brief Runner that adds all measurements with a robust estimator
 *
 * @author Dan McGann
 * @date January 2022
 */
#pragma once
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "exp_runner/Runner.h"

namespace exp_runner {

template <class POSE_TYPE>
class MEstRunner : public Method<POSE_TYPE> {
  /** FIELDS **/
 private:
  /// @brief the solver
  boost::shared_ptr<gtsam::ISAM2> solver_;
  /// @brief internal tracker for current estimate
  gtsam::Values current_estimate_;
  /// @brief Current Mode Sequence
  ModeSequence current_mode_sequence_;
  /// @brief information about mh measurements
  std::vector<MHInfo> mh_infos_;
  /// @brief The robust kernel to use
  gtsam::noiseModel::mEstimator::Base::shared_ptr robust_kernel_;
  std::string kernel_name_;

  /** INTERFACE IMPL **/
 public:
  MEstRunner(gtsam::noiseModel::mEstimator::Base::shared_ptr robust_kernel, std::string kernel_name)
      : robust_kernel_(robust_kernel), kernel_name_(kernel_name) {}
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

#include "exp_runner/MEstRunner-inl.h"