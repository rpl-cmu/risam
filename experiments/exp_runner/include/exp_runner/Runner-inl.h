/** @brief Implementation for Runner Functions
 *
 * @author Dan McGann
 * @date January 2022
 */
#pragma once
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <ctime>
#include <iomanip>
#include <sstream>

#include "exp_runner/Runner.h"

namespace bfs = boost::filesystem;

namespace exp_runner {

/***************************************************************************/
template <class POSE_TYPE>
ExperimentRunner<POSE_TYPE>::ExperimentRunner(boost::shared_ptr<Method<POSE_TYPE>> method, size_t save_every_n)
    : method_(method), save_every_n_(save_every_n) {}

/***************************************************************************/
template <class POSE_TYPE>
void ExperimentRunner<POSE_TYPE>::run(irl::Log& log, std::string output_dir) {
  method_->init();
  // Ensure that the output location exists
  bfs::create_directory(output_dir);
  // Generate the output dir
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::stringstream time_ss;
  time_ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
  std::string result_dir = output_dir + "/" + log.header.name + "_" + method_->getName() + "_" + time_ss.str();
  bfs::create_directory(result_dir);

  // Generate Interation output folder
  std::string interation_result_dir = result_dir + "/iterations";
  bfs::create_directory(interation_result_dir);

  // Iterate through all entries and call appropriate method handler
  size_t iteration_index = 0;
  for (auto& entry : log.entries) {
    // Try cast to all entry types
    auto prior = boost::dynamic_pointer_cast<irl::Prior<POSE_TYPE>>(entry);
    auto odom = boost::dynamic_pointer_cast<irl::Odometry<POSE_TYPE>>(entry);
    auto loop = boost::dynamic_pointer_cast<irl::Loop<POSE_TYPE>>(entry);

    // Run handler for the proper entry type
    std::cout << std::endl << "ITER: " << iteration_index << std::endl;
    auto exe_time_start = std::chrono::high_resolution_clock::now();
    if (prior) {
      method_->handlePrior(*prior);
    } else if (odom) {
      method_->handleOdometry(*odom);
    } else if (loop) {
      method_->handleLoop(*loop);
    } else {
      throw std::runtime_error("Experiment Runner: Encountered Entry of unknown type in IRL.");
    }
    auto exe_time_end = std::chrono::high_resolution_clock::now();
    auto iter_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(exe_time_end - exe_time_start).count() * 1e-6;

    // After we have executed the code, accumulate metric information
    iteration_values_ = method_->getEstimate();
    iteration_modes_ = method_->getModeSequence();
    iteration_times_ms_.push_back(iter_ms);
    total_execution_time_s_ += iter_ms / 1000.0;

    if ((iteration_index % save_every_n_) == 0) {
      // Write iteration info "Values" format
      std::string prefix = (boost::format("%05d") % iteration_index).str();
      auto values_file = std::make_shared<std::ofstream>(interation_result_dir + "/" + prefix + "_values.txt");
      auto modes_file = std::make_shared<std::ofstream>(interation_result_dir + "/" + prefix + "_modes.txt");
      for (size_t hypo_idx = 0; hypo_idx < iteration_values_.size(); hypo_idx++) {
        *values_file << serialize_values(iteration_values_[hypo_idx]) << std::endl;
        *modes_file << serialize_modes(iteration_modes_[hypo_idx]) << std::endl;
      }
    }

    iteration_index++;
  }

  // Handle any final compute before writing final values
  method_->handleFinal();
  iteration_values_ = method_->getEstimate();
  iteration_modes_ = method_->getModeSequence();

  // Write the iteration times
  auto iteration_times_file = std::make_shared<std::ofstream>(result_dir + "/iteration_times.txt");
  for (auto t_ms : iteration_times_ms_) *iteration_times_file << t_ms << std::endl;

  // Write out convience final values and modes
  auto final_values_file = std::make_shared<std::ofstream>(result_dir + "/" + "final_values.txt");
  auto final_modes_file = std::make_shared<std::ofstream>(result_dir + "/" + "final_modes.txt");
  for (size_t hypo_idx = 0; hypo_idx < iteration_values_.size(); hypo_idx++) {
    *final_values_file << serialize_values(iteration_values_[hypo_idx]) << std::endl;
    *final_modes_file << serialize_modes(iteration_modes_[hypo_idx]) << std::endl;
  }
}

/***************************************************************************/
template <class POSE_TYPE>
std::string ExperimentRunner<POSE_TYPE>::serialize_values(gtsam::Values& values) {
  bool first = false;
  std::stringstream ss;
  ss << std::setprecision(5) << std::fixed;

  for (auto kvp : values) {
    if (first) {
      first = false;
    } else {
      ss << " ";
    }

    auto symbol = gtsam::Symbol(kvp.key);
    if (symbol.chr() == 'x') {
      if (std::is_same<POSE_TYPE, gtsam::Pose2>::value) {
        auto pose = kvp.value.cast<gtsam::Pose2>();
        ss << "POSE2 " << symbol.index() << " " << pose.x() << " " << pose.y() << " " << pose.theta();
      } else if (std::is_same<POSE_TYPE, gtsam::Point2>::value) {
        auto pose = kvp.value.cast<gtsam::Point2>();
        ss << "POINT2 " << symbol.index() << " " << pose.x() << " " << pose.y();
      } else if (std::is_same<POSE_TYPE, gtsam::Point3>::value) {
        auto pose = kvp.value.cast<gtsam::Point3>();
        ss << "POINT3 " << symbol.index() << " " << pose.x() << " " << pose.y() << " " << pose.z();
      } else if (std::is_same<POSE_TYPE, gtsam::Pose3>::value) {
        auto pose = kvp.value.cast<gtsam::Pose3>();
        auto rxryrz = pose.rotation().xyz();
        ss << "POSE3 " << symbol.index() << " " << pose.x() << " " << pose.y() << " " << pose.z() << " " << rxryrz[0]
           << " " << rxryrz[1] << " " << rxryrz[2];
      } else {
        throw std::runtime_error("INVALID VALUE TYPE TO SERIALIZE");
      }
    } else {
      std::cout << "Warning: Unknown Symbol found in Values, only poses 'X' allowed. Unknown value not serialized";
    }
  }
  return ss.str();
}

/***************************************************************************/
template <class POSE_TYPE>
std::string ExperimentRunner<POSE_TYPE>::serialize_modes(ModeSequence& modes) {
  bool first = false;
  std::stringstream ss;

  for (auto& mode : modes) {
    if (first) {
      first = false;
    } else {
      ss << " ";
    }

    ss << mode;
  }
  return ss.str();
}

}  // namespace exp_runner