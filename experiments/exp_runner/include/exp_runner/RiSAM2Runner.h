/** @brief Experiment Interface for pseudo gt runner
 *
 * @author Dan McGann
 * @date January 2022
 */
#pragma once
#include "exp_runner/ModeDetermination.h"
#include "exp_runner/PseudoGtRunner.h"
#include "exp_runner/Runner.h"
#include "risam/RISAM2.h"

namespace exp_runner {

template <class POSE_TYPE>
class RiSAM2Runner : public Method<POSE_TYPE> {
  /** FIELDS **/
 private:
  /// @brief The Parameters to use for RISAM
  risam::RISAM2::RISAM2Params riparams_;
  /// @brief The shape parameter used for SIG Kernels
  double kernel_shape_param_;
  /// @brief The name of this method (may change for factory presets)
  std::string name_;
  /// @brief the solver
  boost::shared_ptr<risam::RISAM2> solver_;
  /// @brief internal tracker for current estimate
  gtsam::Values current_estimate_;
  /// @brief Current Mode Sequence
  ModeSequence current_mode_sequence_;
  /// @brief information about mh measurements
  std::vector<MHInfo> mh_infos_;

  /** INTERFACE IMPL **/
 public:
  RiSAM2Runner(risam::RISAM2::RISAM2Params& params, double kernel_shape_param, std::string name = "risam")
      : riparams_(params), kernel_shape_param_(kernel_shape_param), name_(name) {}
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

#include "exp_runner/RiSAM2Runner-inl.h"