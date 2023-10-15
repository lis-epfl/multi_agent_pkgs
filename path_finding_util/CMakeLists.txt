cmake_minimum_required(VERSION 3.8)
project(path_finding_util)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -flto -march=native")

find_package(voxel_grid_util REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${VOXEL_GRID_UTIL_INCLUDE_DIRS})

add_library(path_finding_util src/path_tools.cpp src/graph_search.cpp src/global_planner.cpp src/prm.cpp)
target_link_libraries(path_finding_util ${VOXEL_GRID_UTIL_LIBRARIES})

install(FILES "${PROJECT_NAME}-config.cmake" "${PROJECT_NAME}-config-version.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake")

install(TARGETS path_finding_util
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include/path_finding_util)
