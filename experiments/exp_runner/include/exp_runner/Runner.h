/** @brief Provides interface used to run experiments as well as interface required by each methood.
 *
 * @author Dan McGann
 * @date January 2022
 */
#pragma once
#include <gtsam/nonlinear/Values.h>

#include <chrono>

#include "irl/irl.h"

namespace exp_runner {

/** TYPES **/
typedef std::vector<gtsam::Values> MethodEstimateResults;
/** @brief indicates the mode selection for each entry
 * There are two special values
 * - (-1): Indicates that more than one mode from a single measurement
 * were selected and that the correct mode was one of them
 * - (-2): Indicates that more than one mode from a single measurement
 * were selected and that the correct mode was NOT one of them
 *
 * These special cases are required by the "add-all" prior works like PCM, or DCS
 */
typedef std::vector<int> ModeSequence;
typedef std::vector<ModeSequence> MethodModeSeqResults;

/// @brief Interface that each method must provide to run the experiments
template <class POSE_TYPE>
class Method {
  /** INTERFACE **/
 public:
  /// @brief Run initialization code, potentially using non-default parameters from file
  virtual void init() = 0;

  /// @brief Updates the method with a new Prior measurement
  virtual void handlePrior(irl::Prior<POSE_TYPE>& prior) = 0;

  /// @brief Updates the method with a new Odometry measurement
  virtual void handleOdometry(irl::Odometry<POSE_TYPE>& odom) = 0;

  /// @brief Updates the method with a new loop closure measurement
  virtual void handleLoop(irl::Loop<POSE_TYPE>& loop) = 0;

  /// @brief Finished up any compute for the methods
  virtual void handleFinal() {}  // This is used primarily by batch runners to ensure final estimates are correct

  /// @brief returns the current estimate(s) from the method (if >1 ordered by hypothesis score)
  virtual MethodEstimateResults getEstimate() = 0;

  /// @brief returns the current mode sequence from the method (if >1 order by hypothesis score)
  virtual MethodModeSeqResults getModeSequence() = 0;

  /// @brief returns the name of the method
  virtual std::string getName() = 0;
};

/// @brief Class used to run an experiment with a configured method
template <class POSE_TYPE>
class ExperimentRunner {
  /** FIELDS **/
 protected:
  /// @brief the method that this runner is executing
  boost::shared_ptr<Method<POSE_TYPE>> method_;

  /// @brief The estimate provided by the method after each iteration (new measurement)
  MethodEstimateResults iteration_values_;
  /// @brief The interation time of the method required for each iteration (new measurement)
  std::vector<double> iteration_times_ms_;
  /// @brief The modes tracked after each iteration
  MethodModeSeqResults iteration_modes_;
  /// @brief The total execution time for this method
  double total_execution_time_s_{0};
  /// @brief Index increment at which to save intermediate results
  size_t save_every_n_{10};

  /** INTERFACE **/
 public:
  /// @brief Constructs a runner for the given method
  ExperimentRunner(boost::shared_ptr<Method<POSE_TYPE>> method, size_t save_every_n);

  /// @brief Executes the method on the given log file accumulating metric data for each iteration
  void run(irl::Log& log, std::string output_dir);

 private:
  std::string serialize_values(gtsam::Values& values);
  std::string serialize_modes(ModeSequence& modes);
};
}  // namespace exp_runner

#include "exp_runner/Runner-inl.h"
