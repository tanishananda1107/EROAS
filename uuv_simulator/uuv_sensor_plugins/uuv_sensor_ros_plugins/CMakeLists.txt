cmake_minimum_required(VERSION 3.8)
project(uuv_sensor_ros_plugins)

if(CMAKE_COMPILER_IS_GNUCXX || CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uuv_gazebo_plugins REQUIRED)
find_package(uuv_sensor_ros_plugins_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(image_transport REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)

# Gazebo Harmonic dependencies
find_package(gz-sim8 REQUIRED)
find_package(gz-transport12 REQUIRED)
find_package(gz-msgs9 REQUIRED)
find_package(gz-common5 REQUIRED)

include_directories(include)

# RPT Plugin
add_library(uuv_gazebo_ros_rpt_plugin SHARED
  src/RPTROSPlugin.cc
  src/ROSBasePlugin.cc
  src/ROSBaseModelPlugin.cc
)
ament_target_dependencies(uuv_gazebo_ros_rpt_plugin
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  uuv_gazebo_plugins
  uuv_sensor_ros_plugins_msgs
  tf2_ros
  tf2
  message_filters
)
target_link_libraries(uuv_gazebo_ros_rpt_plugin
  gz-sim8::core
  gz-transport12::core
  gz-msgs9::core
  gz-common5::core
)

# Camera Plugin
add_library(uuv_gazebo_ros_camera_plugin SHARED
  src/UnderwaterCameraROSPlugin.cc
)
ament_target_dependencies(uuv_gazebo_ros_camera_plugin
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  uuv_gazebo_plugins
  uuv_sensor_ros_plugins_msgs
  tf2_ros
  tf2
  image_transport
)
target_link_libraries(uuv_gazebo_ros_camera_plugin
  gz-sim8::core
  gz-transport12::core
  gz-msgs9::core
  gz-common5::core
)

# Sonar Plugin
add_library(image_sonar_ros_plugin SHARED
  src/gazebo_ros_image_sonar.cpp
)
ament_target_dependencies(image_sonar_ros_plugin
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  uuv_gazebo_plugins
  uuv_sensor_ros_plugins_msgs
  tf2_ros
  tf2
  image_transport
)
target_link_libraries(image_sonar_ros_plugin
  gz-sim8::core
  gz-transport12::core
  gz-msgs9::core
  gz-common5::core
)

# Install targets
install(TARGETS
  uuv_gazebo_ros_rpt_plugin
  uuv_gazebo_ros_camera_plugin
  image_sonar_ros_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# Install includes
install(DIRECTORY include/
  DESTINATION include
)

# Install meshes and urdf
install(DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
