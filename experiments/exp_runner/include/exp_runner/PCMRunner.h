/** @brief Experiment Interface for Incremental Single Robot Pairwise Consistency Measure
 * Implementation provided by `thirdpart/Kimera-RPGO`
 *
 * @note ONLY WORKS FOR POSE GRAPH OPTIMIZATION
 *
 * @author Dan McGann
 * @date January 2022
 */
#pragma once
#define SLOW_BUT_CORRECT_BETWEENFACTOR
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "KimeraRPGO/RobustSolver.h"
#include "KimeraRPGO/SolverParams.h"
#include "exp_runner/ModeDetermination.h"
#include "exp_runner/Runner.h"

namespace exp_runner {

template <class POSE_TYPE>
class PCMRunner : public Method<POSE_TYPE> {
  /** FIELDS **/
 private:
  /// @brief iteration_counter
  size_t iter{0};
  /// @brief The factors to add to the solver on the next update iteration
  gtsam::NonlinearFactorGraph new_factors_;
  /// @brief The new values to add to the solver on the next update iteration
  gtsam::Values new_values_;
  /// @brief the actual solver
  boost::shared_ptr<KimeraRPGO::RobustSolver> solver_;
  /// @brief internal tracker for current estimate
  gtsam::Values current_estimate_;
  /// @brief the current assignment of modes
  ModeSequence current_mode_sequence_;
  /// @brief tracks information about loop closures
  std::vector<MHInfo> mh_infos_;

  /** PARAMETERS **/
 private:
  std::string name_;
  /// @brief max allowable M distance deviation from odometry
  double odom_threshold_;
  /// @brief max allowable M distance deviation between pairs of measurements
  double lc_threshold_;

  /** HELPERS **/
 protected:
  /** @brief Checks iteration count and runs optimizer if needed
   */
  void handleUpdate();

  /** INTERFACE IMPL **/
 public:
  PCMRunner(double odom_threshold = 3.0, double loop_threshold = 3.0, std::string name = "pcm-o3-l3")
      : odom_threshold_(odom_threshold), lc_threshold_(loop_threshold), name_(name) {}
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

#include "exp_runner/PCMRunner-inl.h"