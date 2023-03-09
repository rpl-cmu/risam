/** @brief Entry Point for running MH-Inference experiments
 * @param cli_args: for cli arguments run program with --help or look at `handle_args`
 *
 * @author Dan McGann
 * @date January 2022
 */

#include <gtsam/inference/Key.h>

#include <boost/program_options.hpp>

#include "exp_runner/Factory.h"
#include "exp_runner/MaxMixRunner.h"
#include "exp_runner/PseudoGtRunner.h"
#include "exp_runner/Runner.h"
#include "irl/irl.h"

/**
 *    ###    ########   ######    ######
 *   ## ##   ##     ## ##    ##  ##    ##
 *  ##   ##  ##     ## ##        ##
 * ##     ## ########  ##   ####  ######
 * ######### ##   ##   ##    ##        ##
 * ##     ## ##    ##  ##    ##  ##    ##
 * ##     ## ##     ##  ######    ######
 */

namespace po = boost::program_options;
po::variables_map handle_args(int argc, const char *argv[]) {
  // Define the options
  po::options_description options("Allowed options");
  // clang-format off
  options.add_options()
      ("help,h",                                                "Produce help message")
      ("irl_file,i",    po::value<std::string>()->required(),   "(Required) Path to Incremental Robot Log file.")
      ("method,m",      po::value<std::string>()->required(),   "(Required) The name of the method to run (e.g. pseudo-gt, see exp_runners/include/Factory.h for more options).")
      ("save_every_n,n",po::value<size_t>()->required(),        "(Required) Runner will save intermediate result files every N iterations.")
      ("output_dir,o",  po::value<std::string>()->required(),   "(Required) Directory to which the results will be saved.");
  // clang-format on

  // Parse and return the options
  po::variables_map var_map;
  po::store(po::parse_command_line(argc, argv, options), var_map);

  // Handle help special case
  if (var_map.count("help") || argc == 1) {
    std::cout << "run-experiment: Main entry point to run robust SLAM methods on IRL datasets. Please provide required arguments: " << std::endl;
    std::cout << options << "\n";
    exit(1);
  }
  
  // Handle all other arguments
  po::notify(var_map);

  return var_map;
}

/**
 * ##     ##    ###    #### ##    ##
 * ###   ###   ## ##    ##  ###   ##
 * #### ####  ##   ##   ##  ####  ##
 * ## ### ## ##     ##  ##  ## ## ##
 * ##     ## #########  ##  ##  ####
 * ##     ## ##     ##  ##  ##   ###
 * ##     ## ##     ## #### ##    ##
 */

int main(int argc, const char *argv[]) {
  auto args = handle_args(argc, argv);
  irl::Log log = irl::parse_irl_file(args["irl_file"].as<std::string>());

  std::cout << "Running " << args["method"].as<std::string>() << " on ..." << std::endl;
  std::cout << "Dataset" << std::endl;
  std::cout << "----------------------------------" << std::endl;
  std::cout << log.header.name << std::endl;
  std::cout << log.header.date << std::endl;
  std::cout << "Pose Dim: " << log.header.global_dim << std::endl;
  std::cout << log.header.user_string << std::endl;
  std::cout << "----------------------------------" << std::endl;
  std::cout << "Status: Starting" << std::endl;
  if (log.header.linearity == "nonlinear") {
    if (log.header.global_dim == 2) {
      auto method = exp_runner::method_factory<gtsam::Pose2>(args["method"].as<std::string>());
      exp_runner::ExperimentRunner<gtsam::Pose2> runner(method, args["save_every_n"].as<size_t>());
      runner.run(log, args["output_dir"].as<std::string>());

    } else {
      auto method = exp_runner::method_factory<gtsam::Pose3>(args["method"].as<std::string>());
      exp_runner::ExperimentRunner<gtsam::Pose3> runner(method, args["save_every_n"].as<size_t>());
      runner.run(log, args["output_dir"].as<std::string>());
    }
  } else {
    if (log.header.global_dim == 2) {
      auto method = exp_runner::method_factory<gtsam::Point2>(args["method"].as<std::string>());
      exp_runner::ExperimentRunner<gtsam::Point2> runner(method, args["save_every_n"].as<size_t>());
      runner.run(log, args["output_dir"].as<std::string>());

    } else {
      auto method = exp_runner::method_factory<gtsam::Point3>(args["method"].as<std::string>());
      exp_runner::ExperimentRunner<gtsam::Point3> runner(method, args["save_every_n"].as<size_t>());
      runner.run(log, args["output_dir"].as<std::string>());
    }
  }
  std::cout << "Status: Done" << std::endl;

  return 0;
}