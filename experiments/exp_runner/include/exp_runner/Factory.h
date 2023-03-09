/** @brief Factor for Runner Methods
 *
 * @author Dan McGann
 * @date Jan 2022
 */
#pragma once
#include <gtsam/linear/LossFunctions.h>

#include <boost/shared_ptr.hpp>

#include "exp_runner/DCRunner.h"
#include "exp_runner/GNCBatchRunner.h"
#include "exp_runner/MEstRunner.h"
#include "exp_runner/MaxMixRunner.h"
#include "exp_runner/PCMRunner.h"
#include "exp_runner/PseudoGtRunner.h"
#include "exp_runner/RiSAM2Runner.h"
#include "exp_runner/Runner.h"

namespace exp_runner {

/// @brief Factor for Pose2 methods
template <class POSE_TYPE>
boost::shared_ptr<exp_runner::Method<POSE_TYPE>> method_factory(std::string method_name) {
  if (method_name == "pseudo-gt") {
    return boost::make_shared<exp_runner::PseudoGtRunner<POSE_TYPE>>();
  } else if (method_name == "gnc-batch") {
    return boost::make_shared<exp_runner::GNCBatchRunner<POSE_TYPE>>();
  } else if (method_name == "pcm") {
    return boost::make_shared<exp_runner::PCMRunner<POSE_TYPE>>(3.0, 3.0, "pcm");
  } else if (method_name == "maxmix") {
    return boost::make_shared<exp_runner::MaxMixRunner<POSE_TYPE>>();
  } else if (method_name == "dcsam") {
    return boost::make_shared<exp_runner::DCRunner<POSE_TYPE>>();
  } else if (method_name == "huber") {
    return boost::make_shared<exp_runner::MEstRunner<POSE_TYPE>>(gtsam::noiseModel::mEstimator::Huber::Create(3),
                                                                 "huber");
  } else if (method_name == "gm") {
    return boost::make_shared<exp_runner::MEstRunner<POSE_TYPE>>(gtsam::noiseModel::mEstimator::GemanMcClure::Create(3),
                                                                 "gm");
  } else if (method_name == "gauss") {
    return boost::make_shared<exp_runner::MEstRunner<POSE_TYPE>>(gtsam::noiseModel::mEstimator::Null::Create(),
                                                                 "gauss");
  } else if (method_name == "risam") {  // riSAM Paper Experimental Params
    risam::DoglegLineSearchParams opt_params;
    opt_params.search_type = risam::DoglegLineSearchType::OUTWARD;
    opt_params.init_delta = 1.0;
    opt_params.min_delta = 1.0;
    opt_params.max_delta = 50;
    risam::RISAM2::RISAM2Params riparams;
    riparams.converge_after_new_gnc = true;
    riparams.converge_mu = true;
    riparams.converge_values = false;
    riparams.increment_outlier_mu = true;
    riparams.optimization_params = opt_params;
    return boost::make_shared<exp_runner::RiSAM2Runner<POSE_TYPE>>(riparams, 3, method_name);
  } else {
    throw std::runtime_error("Unknown Method Name");
  }
}
}  // namespace exp_runner
