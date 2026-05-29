cmake_minimum_required(VERSION 3.8)
project(uuv_tutorial_dp_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# ------------------------------------------------------------------------------
# Install Python scripts
# ------------------------------------------------------------------------------

install(
  PROGRAMS
    scripts/tutorial_dp_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Install launch files
# ------------------------------------------------------------------------------

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Install package resources
# ------------------------------------------------------------------------------

install(
  DIRECTORY
    config
    rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ------------------------------------------------------------------------------
# Ament package
# ------------------------------------------------------------------------------

ament_package()
