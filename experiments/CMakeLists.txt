message(STATUS "================  EXPERIMENTS  ======================")

# Prior Works for Comparison
add_subdirectory(thirdparty)

# Subprojects
add_subdirectory(irl)
add_subdirectory(exp_runner)

# Extra Boost Libraries needed by experiments
FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# Declare and configure experiment runner executable
add_executable(run-experiment "run-experiment.cpp")
target_link_libraries(run-experiment gtsam ${Boost_LIBRARIES} exp_runner irl)