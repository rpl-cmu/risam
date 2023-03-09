project(robust-inference CXX C)
cmake_minimum_required(VERSION 2.8.3)

message(STATUS "================  ROBUST-INFERENCE  ======================")

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

# build riSAM
add_subdirectory(risam)

# Build Experiments directory
add_subdirectory(experiments)
