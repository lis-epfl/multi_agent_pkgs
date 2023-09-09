cmake_minimum_required(VERSION 3.8)
project(decomp_ros_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(MSG_FILES
  "msg/Ellipsoid.msg"
  "msg/EllipsoidArray.msg"
  "msg/Polyhedron.msg"
  "msg/PolyhedronArray.msg"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
