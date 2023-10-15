cmake_minimum_required(VERSION 3.8)
project(voxel_grid_util)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -flto -march=native")

find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(voxel_grid_util src/voxel_grid.cpp src/raycast.cpp)

install(FILES "${PROJECT_NAME}-config.cmake" "${PROJECT_NAME}-config-version.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake")

install(TARGETS voxel_grid_util
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include/voxel_grid_util)
