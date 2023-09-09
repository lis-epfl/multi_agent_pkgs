cmake_minimum_required(VERSION 3.8)
project(mapping_util)

add_compile_options(-g)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(voxel_grid_util REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(env_builder_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  pcl_conversions
  sensor_msgs
  env_builder_msgs
  visualization_msgs
)

include_directories(include/mapping_util ${VOXEL_GRID_UTIL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(map_builder src/map_builder.cpp)
target_link_libraries(map_builder ${VOXEL_GRID_UTIL_LIBRARIES} voxel_grid) 
ament_target_dependencies(map_builder ${DEPENDENCIES})

add_executable(map_builder_node src/map_builder_node.cpp)
target_link_libraries(map_builder_node map_builder)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  map_builder_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
