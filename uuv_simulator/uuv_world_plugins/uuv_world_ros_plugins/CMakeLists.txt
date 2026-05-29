cmake_minimum_required(VERSION 3.8)
project(uuv_world_ros_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 for ROS2 + Harmonic
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------------------------------------------------------
# Find packages
# ------------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-math7 REQUIRED)

find_package(uuv_world_plugins REQUIRED)
find_package(uuv_world_ros_plugins_msgs REQUIRED)

# ------------------------------------------------------------------------------
# Include directories
# ------------------------------------------------------------------------------

include_directories(
  include
)

# ------------------------------------------------------------------------------
# UnderwaterCurrentROSPlugin
# ------------------------------------------------------------------------------

add_library(uuv_underwater_current_ros_plugin SHARED
  src/UnderwaterCurrentROSPlugin.cc
)

target_link_libraries(uuv_underwater_current_ros_plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
)

ament_target_dependencies(
  uuv_underwater_current_ros_plugin
  rclcpp
  geometry_msgs
  uuv_world_plugins
  uuv_world_ros_plugins_msgs
)

# ------------------------------------------------------------------------------
# SphericalCoordinatesROSInterfacePlugin
# ------------------------------------------------------------------------------

add_library(uuv_sc_ros_interface_plugin SHARED
  src/SphericalCoordinatesROSInterfacePlugin.cc
)

target_link_libraries(uuv_sc_ros_interface_plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
)

ament_target_dependencies(
  uuv_sc_ros_interface_plugin
  rclcpp
  geometry_msgs
  uuv_world_plugins
  uuv_world_ros_plugins_msgs
)

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------

install(
  TARGETS
    uuv_underwater_current_ros_plugin
    uuv_sc_ros_interface_plugin
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

# ------------------------------------------------------------------------------
# Export
# ------------------------------------------------------------------------------

ament_export_include_directories(include)

ament_export_libraries(
  uuv_underwater_current_ros_plugin
  uuv_sc_ros_interface_plugin
)

ament_package()
