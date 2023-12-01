cmake_minimum_required(VERSION 3.8)
project(multi_agent_planner)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fno-tree-ccp")

find_package(ament_cmake REQUIRED)
find_package(decomp_util)
find_package(Eigen3 REQUIRED)
find_package(jps3d REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(GUROBI REQUIRED)
find_package(env_builder_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
find_package(decomp_ros_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(multi_agent_planner_msgs REQUIRED)
find_package(convex_decomp_util REQUIRED)
find_package(voxel_grid_util REQUIRED)
find_package(path_finding_util REQUIRED)
find_package(mapping_util REQUIRED)
find_package(tf2_ros REQUIRED)

set(DEPENDENCIES
  pcl_conversions
  rclcpp
  env_builder_msgs
  multi_agent_planner_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  decomp_ros_msgs
  mapping_util
  decomp_ros_utils
  tf2_ros
)

set(GUROBI_LIBRARIES "$ENV{GUROBI_HOME}/lib/libgurobi_c++.a;$ENV{GUROBI_HOME}/lib/libgurobi100.so" )

include_directories(include/multi_agent_planner ${PCL_INCLUDE_DIRS} ${CONVEX_DECOMP_UTIL_INCLUDE_DIRS} ${GUROBI_INCLUDE_DIRS} ${DECOMP_UTIL_INCLUDE_DIRS} ${JPS3D_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${VOXEL_GRID_UTIL_INCLUDE_DIRS} ${PATH_FINDING_UTIL_INCLUDE_DIRS})

# message("decomp_ros_utils_INCLUDE_DIRECTORIES: " ${decomp_ros_utils_INCLUDE_DIRECTORIES})
add_library(agent_class src/agent_class.cpp)
# target_include_directories(agent_class PRIVATE ${decomp_ros_utils_INCLUDE_DIRECTORIES})
target_link_libraries(agent_class ${CONVEX_DECOMP_UTIL_LIBRARIES} ${JPS3D_LIBRARIES} ${GUROBI_LIBRARIES} ${PATH_FINDING_UTIL_LIBRARIES} ${VOXEL_GRID_UTIL_LIBRARIES})
ament_target_dependencies(agent_class ${DEPENDENCIES})

add_executable(agent_node src/agent_node.cpp)
target_link_libraries(agent_node agent_class ${CONVEX_DECOMP_UTIL_LIBRARIES})
ament_target_dependencies(agent_node ${DEPENDENCIES})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  agent_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
)

ament_package()
