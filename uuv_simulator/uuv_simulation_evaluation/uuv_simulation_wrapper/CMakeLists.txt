cmake_minimum_required(VERSION 3.8)
project(uuv_simulation_wrapper)

find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)

find_package(rclpy REQUIRED)

install(
  PROGRAMS
    scripts/set_simulation_timer
    scripts/unpause_simulation
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# TESTING (ROS2 way)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    test_run_task
    test/test_run_task.py
  )
endif()

ament_package()
