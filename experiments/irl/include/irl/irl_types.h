/** irl_types.cpp
 * This file defines the data types found in a Incremental Log File.
 *
 * @author Dan McGann
 * @date Jan 2022
 */
#pragma once
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/sam/BearingRangeFactor.h>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <deque>
#include <type_traits>

#include "risam/GraduatedFactor.h"

using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

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
/** @brief Internal namespace for templatized parsing of tokens from an std::deque<std::string>
 * @note no template forms for sto{i/l/d} so we have to use template specialization here
 * this template specialization requires being defined at the namespace level so
 * we create a internal namespace here
 */
namespace parsing {
/// @brief Parses the next token from the given entry components
/// advancing the container and returning popped element as type T
template <typename T>
T next(std::deque<std::string>& c);
}  // namespace parsing

/** @brief Parses a Pose of POSE_TYPE
 * @note Can handle gtsam::Pose2 and gtsam::Pose3 types
 * @param in: std::deque<std::string> that must be advanced to start of Pose parameters
 */
template <typename POSE_TYPE>
POSE_TYPE parse_pose(std::deque<std::string>& in);

/** @brief Parses a covariance matrix of specified size
 * @param dim: the dimensionality of the matrix to parse
 * @param in: std::deque<std::string> that must be advanced to start of matrix coefficients
 */
gtsam::Matrix parse_cov_size(size_t dim, std::deque<std::string>& in);

/** @brief Covariance parsing specialized for different POSE TYPES
 * @note Can handle gtsam::Pose2 and gtsam::Pose3 types
 * @param in: std::deque<std::string> that must be advanced to start of matrix coefficients
 */
template <typename POSE_TYPE>
gtsam::Matrix parse_covariance(std::deque<std::string>& in);

/**
 * ##     ## ########    ###    ########  ######## ########
 * ##     ## ##         ## ##   ##     ## ##       ##     ##
 * ##     ## ##        ##   ##  ##     ## ##       ##     ##
 * ######### ######   ##     ## ##     ## ######   ########
 * ##     ## ##       ######### ##     ## ##       ##   ##
 * ##     ## ##       ##     ## ##     ## ##       ##    ##
 * ##     ## ######## ##     ## ########  ######## ##     ##
 */
/// @brief container for header informaton in IRL files
struct Header {
  std::string name;         /// @brief the name of the dataset
  std::string date;         /// @brief the date the dataset was generated
  size_t global_dim;        /// @brief the global dim of the dataset (2 or 3)
  std::string linearity;    /// @brief The linearity of the problem (linear or nonlinear)
  std::string user_string;  /// @brief notes by the dataset creator
};

/**
 * ##     ## ########    ###     ######  ##     ## ########  ######## ##     ## ######## ##    ## ########  ######
 * ###   ### ##         ## ##   ##    ## ##     ## ##     ## ##       ###   ### ##       ###   ##    ##    ##    ##
 * #### #### ##        ##   ##  ##       ##     ## ##     ## ##       #### #### ##       ####  ##    ##    ##
 * ## ### ## ######   ##     ##  ######  ##     ## ########  ######   ## ### ## ######   ## ## ##    ##     ######
 * ##     ## ##       #########       ## ##     ## ##   ##   ##       ##     ## ##       ##  ####    ##          ##
 * ##     ## ##       ##     ## ##    ## ##     ## ##    ##  ##       ##     ## ##       ##   ###    ##    ##    ##
 * ##     ## ######## ##     ##  ######   #######  ##     ## ######## ##     ## ######## ##    ##    ##     ######
 */

/// @brief Generic interface for a measurement found within entries
struct Measurement {
  typedef boost::shared_ptr<Measurement> shared_ptr;
  /// @brief Returns true iff the measurement is a null hypothesis measurement
  virtual bool isNull() { return false; }
};

/// @brief the Null Hypothesis measurement
struct NullHypothesis : public Measurement {
  bool isNull() override { return true; }
};

/// @brief a non-null pose measure (priors/odometry)
template <class POSE_TYPE>
struct PoseMeasure : public Measurement {
  POSE_TYPE pose;
  gtsam::Matrix covariance;
};

/// @brief a non-null between pose Measurement
template <class POSE_TYPE>
struct LoopMeasure : public Measurement {
  gtsam::Key pose_b_key;
  POSE_TYPE rel_pose;
  gtsam::Matrix covariance;
};

/**
 *  ######   ######## ##    ## ######## ########     ###    ########  #######  ########   ######
 * ##    ##  ##       ###   ## ##       ##     ##   ## ##      ##    ##     ## ##     ## ##    ##
 * ##        ##       ####  ## ##       ##     ##  ##   ##     ##    ##     ## ##     ## ##
 * ##   #### ######   ## ## ## ######   ########  ##     ##    ##    ##     ## ########   ######
 * ##    ##  ##       ##  #### ##       ##   ##   #########    ##    ##     ## ##   ##         ##
 * ##    ##  ##       ##   ### ##       ##    ##  ##     ##    ##    ##     ## ##    ##  ##    ##
 *  ######   ######## ##    ## ######## ##     ## ##     ##    ##     #######  ##     ##  ######
 */
typedef std::function<gtsam::Values(const gtsam::Values&)> Generator;

/// @brief Generator for Prior Measurements
struct EmptyGenerator {
  EmptyGenerator();
  gtsam::Values operator()(const gtsam::Values& vals);
};

/// @brief Generator for Prior Measurements
template <class POSE_TYPE>
struct PriorGen {
  gtsam::Key key;
  POSE_TYPE pose;

  PriorGen(gtsam::Key key, POSE_TYPE pose);
  gtsam::Values operator()(const gtsam::Values& vals);
};

/// @brief Generator for Odometry and Loop Closure Measurements
template <class POSE_TYPE>
struct BetweenPoseGen {
  gtsam::Key start_key;
  gtsam::Key end_key;
  POSE_TYPE rel_pose;

  BetweenPoseGen(gtsam::Key start_key, gtsam::Key end_key, POSE_TYPE rel_pose);
  gtsam::Values operator()(const gtsam::Values& vals);
};

/**
 * ######## ##    ## ######## ########  #### ########  ######
 * ##       ###   ##    ##    ##     ##  ##  ##       ##    ##
 * ##       ####  ##    ##    ##     ##  ##  ##       ##
 * ######   ## ## ##    ##    ########   ##  ######    ######
 * ##       ##  ####    ##    ##   ##    ##  ##             ##
 * ##       ##   ###    ##    ##    ##   ##  ##       ##    ##
 * ######## ##    ##    ##    ##     ## #### ########  ######
 */

/// @brief Base class for all entries
struct Entry {
  typedef boost::shared_ptr<Entry> shared_ptr;
  /// @brief The number of modes within this measurement (>0)
  size_t num_modes;
  /// @brief The index of the correct mode of this measurement
  size_t correct_mode_idx;
  /// @brief The measurements
  std::vector<Measurement::shared_ptr> measurements;

  /** @brief returns the i'th mode as a GTSAM factor
   * @note pointer is null iff the mode is the null hypothesis
   */
  virtual gtsam::NonlinearFactor::shared_ptr asFactor(size_t mode_idx) = 0;

  /// @brief Returns list of all keys involved in the entry
  virtual gtsam::KeyVector keys() = 0;

  /// @brief prints entry info
  virtual void print() = 0;
};

/// @brief Prior Entry
template <class POSE_TYPE>
struct Prior : public Entry {
  /** FIELDS **/
  /// @brief the Pose that this prior is attached to
  gtsam::Key pose_key;

  /** INTERFACE **/
  /** @brief Parses and Constructs a Prior Entry
   * @param std::deque<std::string> advanced to start of Prior parameters
   */
  Prior(std::deque<std::string>& in);
  gtsam::NonlinearFactor::shared_ptr asFactor(size_t mode_idx);
  typename risam::GenericGraduatedFactor<gtsam::PriorFactor<POSE_TYPE>>::shared_ptr asGNCFactor(
      size_t mode_idx, double shape_param = 1.0);
  gtsam::KeyVector keys();
  Generator getGenerator(size_t mode_idx);
  void print();
};

/// @brief Odometry Entry
template <class POSE_TYPE>
struct Odometry : public Entry {
  /** FIELDS **/
  /// @brief The start pose key for this odometry segment
  gtsam::Key start_pose_key;
  /// @brief The end pose key for this odometry segment
  gtsam::Key end_pose_key;

  /** INTERFACE **/
  /** @brief Parses and Constructs a Odometry Entry
   * @param std::deque<std::string> advanced to start of Odometry parameters
   */
  Odometry(std::deque<std::string>& in);
  gtsam::NonlinearFactor::shared_ptr asFactor(size_t mode_idx);
  typename risam::GenericGraduatedFactor<gtsam::BetweenFactor<POSE_TYPE>>::shared_ptr asGNCFactor(
      size_t mode_idx, double shape_param = 1.0);
  gtsam::KeyVector keys();
  Generator getGenerator(size_t mode_idx);
  void print();
};

/// @brief Loop Closure Entry
template <class POSE_TYPE>
struct Loop : public Entry {
  /** FIELDS **/
  /// @brief Pose A key for loop closure measurement ^aT_b
  gtsam::Key pose_a_key;

  /** INTERFACE **/
  /** @brief Parses and Constructs a Loop Entry
   * @param std::deque<std::string> advanced to start of Loop parameters
   */
  Loop(std::deque<std::string>& in);
  gtsam::NonlinearFactor::shared_ptr asFactor(size_t mode_idx);
  typename risam::GenericGraduatedFactor<gtsam::BetweenFactor<POSE_TYPE>>::shared_ptr asGNCFactor(
      size_t mode_idx, double shape_param = 1.0);
  gtsam::KeyVector keys();
  Generator getGenerator(size_t mode_idx);
  void print();
};

/**
 * ##        #######   ######
 * ##       ##     ## ##    ##
 * ##       ##     ## ##
 * ##       ##     ## ##   ####
 * ##       ##     ## ##    ##
 * ##       ##     ## ##    ##
 * ########  #######   ######
 */

/// @brief contains the information from an entire IRL file
struct Log {
  /// @brief dataset header information
  Header header;
  /// @brief temporally order list of events
  std::vector<Entry::shared_ptr> entries;
};

}  // namespace irl
#include "irl/irl_types-inl.h"