cmake_minimum_required(VERSION 3.8)

project(uuv_control_cascaded_pid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  DIRECTORY
    src/PID
  DESTINATION
    lib/python3.12/site-packages
)

install(
  PROGRAMS
    scripts/AccelerationControl.py
    scripts/VelocityControl.py
    scripts/PositionControl.py
    scripts/PositionControlUnderactuated.py
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
