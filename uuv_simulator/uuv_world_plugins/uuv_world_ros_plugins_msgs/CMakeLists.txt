cmake_minimum_required(VERSION 3.8)
project(uuv_world_ros_plugins_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# C++ Standard
# ------------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------------------------------------------------------
# Find packages
# ------------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ------------------------------------------------------------------------------
# Generate ROS2 services
# ------------------------------------------------------------------------------

rosidl_generate_interfaces(${PROJECT_NAME}

  "srv/SetCurrentModel.srv"
  "srv/GetCurrentModel.srv"
  "srv/SetCurrentVelocity.srv"
  "srv/SetCurrentDirection.srv"
  "srv/SetOriginSphericalCoord.srv"
  "srv/GetOriginSphericalCoord.srv"
  "srv/TransformToSphericalCoord.srv"
  "srv/TransformFromSphericalCoord.srv"

  DEPENDENCIES
    std_msgs
    geometry_msgs
)

# ------------------------------------------------------------------------------
# Export runtime dependencies
# ------------------------------------------------------------------------------

ament_export_dependencies(rosidl_default_runtime)

# ------------------------------------------------------------------------------
# Package
# ------------------------------------------------------------------------------

ament_package()
