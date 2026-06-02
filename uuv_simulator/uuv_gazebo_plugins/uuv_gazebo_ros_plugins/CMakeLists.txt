cmake_minimum_required(VERSION 3.8)
project(uuv_gazebo_ros_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── ROS 2 dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(uuv_gazebo_ros_plugins_msgs REQUIRED)
find_package(uuv_gazebo_plugins REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# ── Gazebo Harmonic (gz-sim8) ─────────────────────────────────────────────────
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Convenience alias targets supplied by gz-cmake
set(GZ_SIM_VER     ${gz-sim8_VERSION_MAJOR})
set(GZ_PLUGIN_VER  ${gz-plugin2_VERSION_MAJOR})

# ── Common dependency list for ament_target_dependencies ──────────────────────
set(COMMON_DEPS
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  uuv_gazebo_ros_plugins_msgs
  uuv_gazebo_plugins
  tf2_ros
  tf2
)

include_directories(
  include
)

add_library(uuv_fin_ros_plugin SHARED
  src/FinROSPlugin.cc
)
target_include_directories(uuv_fin_ros_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(uuv_fin_ros_plugin ${COMMON_DEPS})
target_link_libraries(uuv_fin_ros_plugin
  uuv_gazebo_plugins::uuv_fin_plugin
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::register
  gz-math7::gz-math7
)

install(TARGETS uuv_fin_ros_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(uuv_thruster_ros_plugin SHARED
  src/ThrusterROSPlugin.cc
)
target_include_directories(uuv_thruster_ros_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(uuv_thruster_ros_plugin ${COMMON_DEPS})
target_link_libraries(uuv_thruster_ros_plugin
  uuv_gazebo_plugins::uuv_thruster_plugin
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::register
  gz-math7::gz-math7
)

install(TARGETS uuv_thruster_ros_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(uuv_underwater_object_ros_plugin SHARED
  src/UnderwaterObjectROSPlugin.cc
)
target_include_directories(uuv_underwater_object_ros_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(uuv_underwater_object_ros_plugin ${COMMON_DEPS})
target_link_libraries(uuv_underwater_object_ros_plugin
  uuv_gazebo_plugins::uuv_underwater_object_plugin
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::register
  gz-math7::gz-math7
)

install(TARGETS uuv_underwater_object_ros_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(uuv_joint_state_publisher SHARED
  src/JointStatePublisher.cc
)
target_include_directories(uuv_joint_state_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(uuv_joint_state_publisher ${COMMON_DEPS})
target_link_libraries(uuv_joint_state_publisher
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::register
  gz-math7::gz-math7
)

install(TARGETS uuv_joint_state_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}/test
  OPTIONAL
)

ament_export_dependencies(${COMMON_DEPS})
ament_export_libraries(
  uuv_fin_ros_plugin
  uuv_thruster_ros_plugin
  uuv_underwater_object_ros_plugin
  uuv_joint_state_publisher
)

ament_package()
