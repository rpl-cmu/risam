add_library(exp_runner INTERFACE)
target_link_libraries(exp_runner INTERFACE gtsam risam irl KimeraRPGO dcsam)
target_include_directories(exp_runner INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
