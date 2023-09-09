cmake_minimum_required(VERSION 3.8)
project(env_builder_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(MSG_FILES
  "msg/VoxelGrid.msg"
  "msg/VoxelGridStamped.msg"
)

set(SRV_FILES
  "srv/GetVoxelGrid.srv"
  "srv/GetVoxelGridLocal.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES std_msgs
)

ament_package()
