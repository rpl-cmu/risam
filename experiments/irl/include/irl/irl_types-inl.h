/** @brief Implementation for irl-types.h
 *
 * @author Dan McGann
 * @date Jan 2022
 */

#pragma once

#include "irl/irl_types.h"
namespace irl {

/**
 * ##     ## ######## ##       ########  ######## ########   ######
 * ##     ## ##       ##       ##     ## ##       ##     ## ##    ##
 * ##     ## ##       ##       ##     ## ##       ##     ## ##
 * ######### ######   ##       ########  ######   ########   ######
 * ##     ## ##       ##       ##        ##       ##   ##         ##
 * ##     ## ##       ##       ##        ##       ##    ##  ##    ##
 * ##     ## ######## ######## ##        ######## ##     ##  ######
 */

/***************** PARSING INTERNAL *****************/
namespace parsing {
template <>
int next<int>(std::deque<std::string>& c) {
  assert(c.size() > 0);
  std::string comp = c.front();
  c.pop_front();
  return std::stoi(comp);
}

template <>
double next<double>(std::deque<std::string>& c) {
  assert(c.size() > 0);
  std::string comp = c.front();
  c.pop_front();
  return std::stod(comp);
}

template <>
size_t next<size_t>(std::deque<std::string>& c) {
  assert(c.size() > 0);
  std::string comp = c.front();
  c.pop_front();
  return std::stol(comp);
}

template <>
std::string next<std::string>(std::deque<std::string>& c) {
  assert(c.size() > 0);
  std::string comp = c.front();
  c.pop_front();
  return comp;
}
}  // namespace parsing

/***************** PARSE POSES *****************/
template <typename POSE_TYPE>
POSE_TYPE parse_pose(std::deque<std::string>& in) {
  throw std::runtime_error("parse_pose can only handle Pose2 or Pose3");
}

template <>
gtsam::Pose2 parse_pose<gtsam::Pose2>(std::deque<std::string>& in) {
  double x = parsing::next<double>(in);
  double y = parsing::next<double>(in);
  double theta = parsing::next<double>(in);
  return gtsam::Pose2(x, y, theta);  // x, y, theta
}

template <>
gtsam::Point2 parse_pose<gtsam::Point2>(std::deque<std::string>& in) {
  double x = parsing::next<double>(in);
  double y = parsing::next<double>(in);
  return gtsam::Point2(x, y);
}

template <>
gtsam::Point3 parse_pose<gtsam::Point3>(std::deque<std::string>& in) {
  double x = parsing::next<double>(in);
  double y = parsing::next<double>(in);
  double z = parsing::next<double>(in);
  return gtsam::Point3(x, y, z);
}

template <>
gtsam::Pose3 parse_pose<gtsam::Pose3>(std::deque<std::string>& in) {
  double x = parsing::next<double>(in);
  double y = parsing::next<double>(in);
  double z = parsing::next<double>(in);
  gtsam::Point3 p(x, y, z);
  double rx = parsing::next<double>(in);
  double ry = parsing::next<double>(in);
  double rz = parsing::next<double>(in);
  gtsam::Rot3 r = gtsam::Rot3::RzRyRx(rx, ry, rz);  // Rx, Ry, Rz
  return gtsam::Pose3(r, p);
}

/***************** PARSE COVARIANCE *****************/

gtsam::Matrix parse_cov_size(size_t dim, std::deque<std::string>& in) {
  gtsam::Matrix cov = gtsam::Matrix::Zero(dim, dim);
  for (size_t r = 0; r < dim; r++) {
    for (size_t c = 0; c < dim; c++) {
      cov(r, c) = parsing::next<double>(in);
    }
  }
  return cov;
}

template <typename POSE_TYPE>
gtsam::Matrix parse_covariance(std::deque<std::string>& in) {
  if (std::is_same<POSE_TYPE, gtsam::Pose2>::value) {
    return parse_cov_size(3, in);
  } else if (std::is_same<POSE_TYPE, gtsam::Pose3>::value) {
    return parse_cov_size(6, in);
  } else if (std::is_same<POSE_TYPE, gtsam::Point2>::value) {
    return parse_cov_size(2, in);
  } else if (std::is_same<POSE_TYPE, gtsam::Point3>::value) {
    return parse_cov_size(3, in);
  } else {
    throw std::runtime_error("parse_pose can only handle Pose2 or Pose3");
  }
}

/**
 *  ######   ######## ##    ## ######## ########     ###    ########  #######  ########   ######
 * ##    ##  ##       ###   ## ##       ##     ##   ## ##      ##    ##     ## ##     ## ##    ##
 * ##        ##       ####  ## ##       ##     ##  ##   ##     ##    ##     ## ##     ## ##
 * ##   #### ######   ## ## ## ######   ########  ##     ##    ##    ##     ## ########   ######
 * ##    ##  ##       ##  #### ##       ##   ##   #########    ##    ##     ## ##   ##         ##
 * ##    ##  ##       ##   ### ##       ##    ##  ##     ##    ##    ##     ## ##    ##  ##    ##
 *  ######   ######## ##    ## ######## ##     ## ##     ##    ##     #######  ##     ##  ######
 */
/***************************************************************************/
EmptyGenerator::EmptyGenerator() {}

gtsam::Values EmptyGenerator::operator()(const gtsam::Values& vals) { return gtsam::Values(); }

/***************************************************************************/
template <class POSE_TYPE>
PriorGen<POSE_TYPE>::PriorGen(gtsam::Key key, POSE_TYPE pose) : key(key), pose(pose) {}

template <class POSE_TYPE>
gtsam::Values PriorGen<POSE_TYPE>::operator()(const gtsam::Values& vals) {
  gtsam::Values result;
  if (!vals.exists(key)) {
    result.insert(key, pose);
  }
  return result;
}

/***************************************************************************/
template <class POSE_TYPE>
BetweenPoseGen<POSE_TYPE>::BetweenPoseGen(gtsam::Key start_key, gtsam::Key end_key, POSE_TYPE rel_pose)
    : start_key(start_key), end_key(end_key), rel_pose(rel_pose) {}

template <class POSE_TYPE>
gtsam::Values BetweenPoseGen<POSE_TYPE>::operator()(const gtsam::Values& vals) {
  gtsam::Values result;
  if (!vals.exists(end_key)) {
    result.insert(end_key, gtsam::traits<POSE_TYPE>::Compose(vals.at<POSE_TYPE>(start_key), rel_pose));
  }
  return result;
}

/**
 * ########  ########  ####  #######  ########
 * ##     ## ##     ##  ##  ##     ## ##     ##
 * ##     ## ##     ##  ##  ##     ## ##     ##
 * ########  ########   ##  ##     ## ########
 * ##        ##   ##    ##  ##     ## ##   ##
 * ##        ##    ##   ##  ##     ## ##    ##
 * ##        ##     ## ####  #######  ##     ##
 */
/***************************************************************************/
template <class POSE_TYPE>
Prior<POSE_TYPE>::Prior(std::deque<std::string>& in) {
  num_modes = parsing::next<size_t>(in);
  correct_mode_idx = parsing::next<size_t>(in);
  pose_key = X(parsing::next<size_t>(in));
  for (size_t i = 0; i < num_modes; i++) {
    if (in.front() == "NULL") {
      measurements.push_back(boost::make_shared<NullHypothesis>());
      in.pop_front();
    } else {
      auto m = boost::make_shared<PoseMeasure<POSE_TYPE>>();
      m->pose = parse_pose<POSE_TYPE>(in);
      m->covariance = parse_covariance<POSE_TYPE>(in);
      measurements.push_back(m);
    }
  }
}

/***************************************************************************/
template <class POSE_TYPE>
gtsam::NonlinearFactor::shared_ptr Prior<POSE_TYPE>::asFactor(size_t mode_idx) {
  auto measure = measurements[mode_idx];
  auto prior_measure = boost::dynamic_pointer_cast<PoseMeasure<POSE_TYPE>>(measure);
  if (prior_measure) {
    return boost::make_shared<gtsam::PriorFactor<POSE_TYPE>>(
        pose_key, prior_measure->pose, gtsam::noiseModel::Gaussian::Covariance(prior_measure->covariance));
  } else {
    return gtsam::NonlinearFactor::shared_ptr();
  }
}

/***************************************************************************/
template <class POSE_TYPE>
typename risam::GenericGraduatedFactor<gtsam::PriorFactor<POSE_TYPE>>::shared_ptr Prior<POSE_TYPE>::asGNCFactor(
    size_t mode_idx, double shape_param) {
  auto measure = measurements[mode_idx];
  auto prior_measure = boost::dynamic_pointer_cast<PoseMeasure<POSE_TYPE>>(measure);
  return risam::make_shared_graduated<gtsam::PriorFactor<POSE_TYPE>>(
      boost::make_shared<risam::SIGKernel>(shape_param), pose_key, prior_measure->pose,
      gtsam::noiseModel::Gaussian::Covariance(prior_measure->covariance));
}

/***************************************************************************/
template <class POSE_TYPE>
Generator Prior<POSE_TYPE>::getGenerator(size_t mode_idx) {
  auto measure = measurements[mode_idx];
  auto prior_measure = boost::dynamic_pointer_cast<PoseMeasure<POSE_TYPE>>(measure);
  return PriorGen<POSE_TYPE>(pose_key, prior_measure->pose);
}

/***************************************************************************/
template <class POSE_TYPE>
gtsam::KeyVector Prior<POSE_TYPE>::keys() {
  return gtsam::KeyVector({pose_key});
}

/***************************************************************************/
template <class POSE_TYPE>
void Prior<POSE_TYPE>::print() {
  std::cout << "Prior Entry --------------- " << gtsam::DefaultKeyFormatter(pose_key) << std::endl;
  for (auto& meas : measurements) {
    std::cout << "Measure:" << std::endl;
    auto measure = boost::dynamic_pointer_cast<PoseMeasure<POSE_TYPE>>(meas);
    // measure->pose.print();
    std::cout << measure->covariance << std::endl;
  }
}

/**
 *  #######  ########   #######  ##     ## ######## ######## ########  ##    ##
 * ##     ## ##     ## ##     ## ###   ### ##          ##    ##     ##  ##  ##
 * ##     ## ##     ## ##     ## #### #### ##          ##    ##     ##   ####
 * ##     ## ##     ## ##     ## ## ### ## ######      ##    ########     ##
 * ##     ## ##     ## ##     ## ##     ## ##          ##    ##   ##      ##
 * ##     ## ##     ## ##     ## ##     ## ##          ##    ##    ##     ##
 *  #######  ########   #######  ##     ## ########    ##    ##     ##    ##
 */
/***************************************************************************/
template <class POSE_TYPE>
Odometry<POSE_TYPE>::Odometry(std::deque<std::string>& in) {
  num_modes = parsing::next<size_t>(in);
  correct_mode_idx = parsing::next<size_t>(in);
  start_pose_key = X(parsing::next<size_t>(in));
  end_pose_key = X(parsing::next<size_t>(in));

  for (size_t i = 0; i < num_modes; i++) {
    auto m = boost::make_shared<PoseMeasure<POSE_TYPE>>();
    m->pose = parse_pose<POSE_TYPE>(in);
    m->covariance = parse_covariance<POSE_TYPE>(in);
    measurements.push_back(m);
  }
}

/***************************************************************************/
template <class POSE_TYPE>
gtsam::NonlinearFactor::shared_ptr Odometry<POSE_TYPE>::asFactor(size_t mode_idx) {
  auto measure = measurements[mode_idx];
  auto odom_measure = boost::dynamic_pointer_cast<PoseMeasure<POSE_TYPE>>(measure);
  return boost::make_shared<gtsam::BetweenFactor<POSE_TYPE>>(
      start_pose_key, end_pose_key, odom_measure->pose,
      gtsam::noiseModel::Gaussian::Covariance(odom_measure->covariance));
}

/***************************************************************************/
template <class POSE_TYPE>
typename risam::GenericGraduatedFactor<gtsam::BetweenFactor<POSE_TYPE>>::shared_ptr Odometry<POSE_TYPE>::asGNCFactor(
    size_t mode_idx, double shape_param) {
  auto measure = measurements[mode_idx];
  auto odom_measure = boost::dynamic_pointer_cast<PoseMeasure<POSE_TYPE>>(measure);
  return risam::make_shared_graduated<gtsam::BetweenFactor<POSE_TYPE>>(
      boost::make_shared<risam::SIGKernel>(shape_param), start_pose_key, end_pose_key, odom_measure->pose,
      gtsam::noiseModel::Gaussian::Covariance(odom_measure->covariance));
}

/***************************************************************************/
template <class POSE_TYPE>
Generator Odometry<POSE_TYPE>::getGenerator(size_t mode_idx) {
  auto measure = measurements[mode_idx];
  auto between_measure = boost::dynamic_pointer_cast<PoseMeasure<POSE_TYPE>>(measure);
  if (between_measure) {
    return BetweenPoseGen<POSE_TYPE>(start_pose_key, end_pose_key, between_measure->pose);
  } else {
    return EmptyGenerator();
  }
}

/***************************************************************************/
template <class POSE_TYPE>
gtsam::KeyVector Odometry<POSE_TYPE>::keys() {
  return gtsam::KeyVector({start_pose_key, end_pose_key});
}

/***************************************************************************/
template <class POSE_TYPE>
void Odometry<POSE_TYPE>::print() {
  std::cout << "Odometry Entry --------------- " << gtsam::DefaultKeyFormatter(start_pose_key) << " -> "
            << gtsam::DefaultKeyFormatter(end_pose_key) << std::endl;
  for (auto& meas : measurements) {
    std::cout << "Measure:" << std::endl;
    auto measure = boost::dynamic_pointer_cast<PoseMeasure<POSE_TYPE>>(meas);
    // measure->pose.print();
    std::cout << measure->covariance << std::endl;
  }
}

/**
 * ##        #######   #######  ########
 * ##       ##     ## ##     ## ##     ##
 * ##       ##     ## ##     ## ##     ##
 * ##       ##     ## ##     ## ########
 * ##       ##     ## ##     ## ##
 * ##       ##     ## ##     ## ##
 * ########  #######   #######  ##
 */
/***************************************************************************/
template <class POSE_TYPE>
Loop<POSE_TYPE>::Loop(std::deque<std::string>& in) {
  num_modes = parsing::next<size_t>(in);
  correct_mode_idx = parsing::next<size_t>(in);
  pose_a_key = X(parsing::next<size_t>(in));

  for (size_t i = 0; i < num_modes; i++) {
    if (in.front() == "NULL") {
      measurements.push_back(boost::make_shared<NullHypothesis>());
      in.pop_front();
    } else {
      auto m = boost::make_shared<LoopMeasure<POSE_TYPE>>();
      m->pose_b_key = X(parsing::next<size_t>(in));
      m->rel_pose = parse_pose<POSE_TYPE>(in);
      m->covariance = parse_covariance<POSE_TYPE>(in);
      measurements.push_back(m);
    }
  }
}

/***************************************************************************/
template <class POSE_TYPE>
gtsam::NonlinearFactor::shared_ptr Loop<POSE_TYPE>::asFactor(size_t mode_idx) {
  auto measure = measurements[mode_idx];
  auto loop_measure = boost::dynamic_pointer_cast<LoopMeasure<POSE_TYPE>>(measure);
  if (loop_measure) {
    return boost::make_shared<gtsam::BetweenFactor<POSE_TYPE>>(
        pose_a_key, loop_measure->pose_b_key, loop_measure->rel_pose,
        gtsam::noiseModel::Gaussian::Covariance(loop_measure->covariance));
  } else {
    return gtsam::NonlinearFactor::shared_ptr();
  }
}

/***************************************************************************/
template <class POSE_TYPE>
typename risam::GenericGraduatedFactor<gtsam::BetweenFactor<POSE_TYPE>>::shared_ptr Loop<POSE_TYPE>::asGNCFactor(
    size_t mode_idx, double shape_param) {
  auto measure = measurements[mode_idx];
  auto loop_measure = boost::dynamic_pointer_cast<LoopMeasure<POSE_TYPE>>(measure);
  if (loop_measure) {
    return risam::make_shared_graduated<gtsam::BetweenFactor<POSE_TYPE>>(
        boost::make_shared<risam::SIGKernel>(shape_param), pose_a_key, loop_measure->pose_b_key,
        loop_measure->rel_pose, gtsam::noiseModel::Gaussian::Covariance(loop_measure->covariance));
  } else {
    return typename risam::GenericGraduatedFactor<gtsam::BetweenFactor<POSE_TYPE>>::shared_ptr();
  }
}

/***************************************************************************/
template <class POSE_TYPE>
Generator Loop<POSE_TYPE>::getGenerator(size_t mode_idx) {
  // Loop Closures are always between known poses
  return EmptyGenerator();
}

/***************************************************************************/
template <class POSE_TYPE>
gtsam::KeyVector Loop<POSE_TYPE>::keys() {
  auto keys = gtsam::KeyVector({pose_a_key});
  for (auto& measure : measurements) {
    auto loop_measure = boost::dynamic_pointer_cast<LoopMeasure<POSE_TYPE>>(measure);
    if (loop_measure) {
      if (std::find(keys.begin(), keys.end(), loop_measure->pose_b_key) == keys.end()) {
        keys.push_back(loop_measure->pose_b_key);
      }
    }
  }
  return keys;
}

/***************************************************************************/
template <class POSE_TYPE>
void Loop<POSE_TYPE>::print() {
  std::cout << "Loop Entry --------------- " << gtsam::DefaultKeyFormatter(pose_a_key) << std::endl;
  for (auto& meas : measurements) {
    auto measure = boost::dynamic_pointer_cast<LoopMeasure<POSE_TYPE>>(meas);
    if (measure) {
      std::cout << "Measure: -> " << gtsam::DefaultKeyFormatter(measure->pose_b_key) << std::endl;
      // measure->rel_pose.print();
      std::cout << measure->covariance << std::endl;

    } else {
      std::cout << "Measure: NullHypo" << std::endl;
    }
  }
}

}  // namespace irl