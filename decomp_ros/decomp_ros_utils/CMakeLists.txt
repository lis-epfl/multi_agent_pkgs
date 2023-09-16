cmake_minimum_required(VERSION 3.8.0)
project(decomp_ros_utils)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wno-deprecated-declarations")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include/decomp_ros_utils ${EIGEN3_INCLUDE_DIRS} ${DECOMP_UTIL_INCLUDE_DIRS})

add_definitions(-DQT_NO_KEYWORDS)

## This setting causes Qt's "MOC" generation to happen automatically.
## this does not moc things in include!!!!!!! only in src
set(CMAKE_AUTOMOC ON)

## We'll use the version that rviz used so they are compatible.
# if(rviz_QT_VERSION VERSION_LESS "5")
#   message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  # find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  # include(${QT_USE_FILE})
# else()
#   message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
# endif()

set(DEPENDENCIES
  rviz_rendering
  rviz_common
  rclcpp
  Eigen3
  decomp_util
  decomp_ros_msgs
  tf2
  tf2_ros
  nav_msgs
  pluginlib
  )

set(SOURCE_FILES
  src/bound_visual.cpp
  src/mesh_visual.cpp
  src/vector_visual.cpp
  src/ellipsoid_array_visual.cpp
  src/ellipsoid_array_display.cpp
  src/polyhedron_array_display.cpp
  ${MOC_FILES})

add_library(mesh_shape src/mesh_shape.cpp)
ament_target_dependencies(mesh_shape ${DEPENDENCIES})

add_library(decomp_rviz_plugins SHARED ${SOURCE_FILES})
target_link_libraries(decomp_rviz_plugins ${QT_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES} mesh_shape)
ament_target_dependencies(decomp_rviz_plugins ${DEPENDENCIES})

install(DIRECTORY include/
   DESTINATION include/
)

install(TARGETS
   decomp_rviz_plugins
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin
   INCLUDES DESTINATION include
)

target_compile_definitions(decomp_rviz_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
