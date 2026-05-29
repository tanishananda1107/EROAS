cmake_minimum_required(VERSION 3.8)
project(uuv_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/vehicle_teleop.py
    scripts/finned_uuv_teleop.py
    scripts/vehicle_keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
