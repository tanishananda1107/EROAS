cmake_minimum_required(VERSION 3.8)
project(uuv_tutorial_dp_controller)

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/tutorial_dp_controller.py
  DESTINATION lib/${PROJECT_NAME}/scripts
)

ament_package()