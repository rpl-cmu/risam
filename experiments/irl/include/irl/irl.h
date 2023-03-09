/**
 * @brief Interface for using the Incremental Robot File Parsing functionality.
 *
 * @author Dan McGann
 * @date Jan 2022
 */
#pragma once
#include <fstream>

#include "irl_types.h"

namespace irl {
namespace internal {

/** @brief Parses header from an IRL file with specified global pose
 * @param text_file: ifstream completely unparsed
 */
Header parse_header(std::ifstream& text_file) {
  // Parse the header
  Header header;
  // Name
  std::getline(text_file, header.name);
  // Date
  std::getline(text_file, header.date);
  // Global Dim
  std::string dim_line;
  std::getline(text_file, dim_line);
  header.global_dim = std::stol(dim_line);
  // Linearity
  std::getline(text_file, header.linearity);
  // User notes
  std::getline(text_file, header.user_string);
  return header;
}

/** @brief Parses entries from an IRL file with specified global pose
 * @param text_file: ifstream with header already parsed
 */
template <class POSE_TYPE>
std::vector<Entry::shared_ptr> parse_entries(std::ifstream& text_file) {
  std::string current_line;
  std::vector<Entry::shared_ptr> entries;
  while (std::getline(text_file, current_line)) {
    std::deque<std::string> components;
    boost::split(components, current_line, boost::is_any_of(" "));
    std::string entry_type = parsing::next<std::string>(components);
    if (entry_type == "PRIOR") {
      entries.push_back(boost::make_shared<Prior<POSE_TYPE>>(components));
    } else if (entry_type == "ODOMETRY") {
      entries.push_back(boost::make_shared<Odometry<POSE_TYPE>>(components));
    } else if (entry_type == "LOOP") {
      entries.push_back(boost::make_shared<Loop<POSE_TYPE>>(components));
    } else {
      throw std::runtime_error("Unknown entry type encountered: \"" + entry_type + "\"");
    }
  }
  return entries;
}
}  // namespace internal

/** @brief Parses an IRL file into a log
 * @param file_path: Path to the IRL text file
 */
Log parse_irl_file(std::string file_path) {
  std::ifstream text_file(file_path);

  try {
    // construct the log
    Log log;
    log.header = internal::parse_header(text_file);
    if (log.header.linearity == "nonlinear") {
      if (log.header.global_dim == 2) {
        log.entries = internal::parse_entries<gtsam::Pose2>(text_file);
      } else if (log.header.global_dim == 3) {
        log.entries = internal::parse_entries<gtsam::Pose3>(text_file);
      } else {
        throw std::runtime_error("IRL Parse Error: Invalid Global Dim = " + std::to_string(log.header.global_dim));
      }
    } else if (log.header.linearity == "linear") {
      if (log.header.global_dim == 2) {
        log.entries = internal::parse_entries<gtsam::Point2>(text_file);
      } else if (log.header.global_dim == 3) {
        log.entries = internal::parse_entries<gtsam::Point3>(text_file);
      } else {
        throw std::runtime_error("IRL Parse Error: Invalid Global Dim = " + std::to_string(log.header.global_dim));
      }
    } else {
      throw std::runtime_error("IRL Parse Error: Invalid Linearity = " + log.header.linearity);
    }
    return log;
  } catch (std::exception& e) {
    std::cout << "Parse IRL File encountered a " << e.what() << " exception. Rethrowing for details " << std::endl;
    throw;
  }
}

}  // namespace irl
