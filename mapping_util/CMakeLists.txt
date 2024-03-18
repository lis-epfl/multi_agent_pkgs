cmake_minimum_required(VERSION 3.8)
project(mapping_util)

# add_compile_options(-g) # compile options for debugging
add_compile_options(-O3 -flto -march=native)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(voxel_grid_util REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(env_builder_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(path_finding_util REQUIRED)
find_package(visualization_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  pcl_conversions
  sensor_msgs
  env_builder_msgs
  tf2_ros
  visualization_msgs
)

include_directories(include/mapping_util ${VOXEL_GRID_UTIL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${PATH_FINDING_UTIL_INCLUDE_DIRS})

add_library(map_builder src/map_builder.cpp)
target_link_libraries(map_builder ${PATH_FINDING_UTIL_LIBRARIES} ${VOXEL_GRID_UTIL_LIBRARIES}) 
ament_target_dependencies(map_builder ${DEPENDENCIES})

add_executable(map_builder_node src/map_builder_node.cpp)
target_link_libraries(map_builder_node map_builder)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  map_builder
  map_builder_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(map_builder)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
