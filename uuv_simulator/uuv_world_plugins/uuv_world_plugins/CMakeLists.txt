cmake_minimum_required(VERSION 3.8)
project(uuv_world_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(ignition-physics10 REQUIRED)
find_package(ignition-gazebo8 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uuv_world_ros_plugins_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS algorithm bind shared_ptr)

# Include directories
include_directories(include ${GAZEBO_INCLUDE_DIRS})
ament_export_include_directories(include)
ament_export_dependencies(
  ignition-physics10
  ignition-gazebo8
  uuv_world_ros_plugins_msgs
)

# Define the plugins
set(PLUGINS
  GaussMarkovProcess
  UnderwaterCurrentPlugin
)

foreach(plugin ${PLUGINS})
  add_library(${plugin} SHARED src/${plugin}.cc)
  ament_target_dependencies(${plugin}
    gazebo_dev
    gazebo_msgs
    ignition-physics10
    ignition-gazebo8
    rclcpp
    geometry_msgs
    uuv_world_ros_plugins_msgs
  )
  target_link_libraries(${plugin}
    ${Boost_LIBRARIES}
  )
  target_include_directories(${plugin} PRIVATE
    ${GAZEBO_INCLUDE_DIRS}
  )
  ament_export_libraries(${plugin})
  install(TARGETS ${plugin}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

ament_package()
