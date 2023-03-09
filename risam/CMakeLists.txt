message(STATUS "================  riSAM  ======================")

file(GLOB_RECURSE risam_srcs "*.cpp" "*.h")
add_library(risam ${risam_srcs})
target_link_libraries(risam gtsam)
target_include_directories(risam PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")