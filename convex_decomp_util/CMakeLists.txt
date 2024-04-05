cmake_minimum_required(VERSION 3.8.0)
project(convex_decomp_util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fno-tree-ccp")

find_package(decomp_util)
find_package(Eigen3 REQUIRED)

include_directories(include ${DECOMP_UTIL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(convex_decomp_util src/convex_decomp.cpp)

install(FILES "${PROJECT_NAME}-config.cmake" "${PROJECT_NAME}-config-version.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake")

install(TARGETS convex_decomp_util
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include/convex_decomp_util)
