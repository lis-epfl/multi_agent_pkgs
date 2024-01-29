cmake_minimum_required(VERSION 3.8)
project(planner_crazyswarm_bridge)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
find_package(multi_agent_planner_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(DEPENDENCIES
  rclcpp
  crazyflie_interfaces
  multi_agent_planner_msgs
  tf2_ros
)

include_directories(include/planner_crazyswarm_bridge)
add_library(bridge_class src/bridge_class.cpp)
ament_target_dependencies(bridge_class ${DEPENDENCIES})

add_executable(bridge_node src/bridge_node.cpp)
target_link_libraries(bridge_node bridge_class)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/tracking_error.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
)
ament_package()
