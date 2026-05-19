cmake_minimum_required(VERSION 3.8)
project(uuv_gazebo_ros_plugins)

if(CMAKE_COMPILER_IS_GNUCXX || CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(uuv_gazebo_plugins REQUIRED)
find_package(uuv_gazebo_ros_plugins_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)

# Include directories
include_directories(include)
ament_export_include_directories(include)

# Define the plugins
set(PLUGINS
  AccelerationsTestPlugin
  CustomBatteryConsumerROSPlugin
  FinROSPlugin
  JointStatePublisher
  LinearBatteryROSPlugin
  ThrusterROSPlugin
  UnderwaterObjectROSPlugin
)

foreach(plugin ${PLUGINS})
  add_library(${plugin} SHARED src/${plugin}.cc)
  ament_target_dependencies(${plugin}
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
    visualization_msgs
    tf2_ros
    tf2
    uuv_gazebo_plugins
    uuv_gazebo_ros_plugins_msgs
    gazebo_dev
  )
  install(TARGETS ${plugin}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

ament_package()
