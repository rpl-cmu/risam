add_library(irl INTERFACE)
target_link_libraries(irl INTERFACE gtsam)
target_include_directories(irl INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")