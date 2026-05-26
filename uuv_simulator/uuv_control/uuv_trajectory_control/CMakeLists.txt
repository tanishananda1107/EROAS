cmake_minimum_required(VERSION 3.8)
project(uuv_trajectory_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(uuv_control_msgs REQUIRED)
find_package(uuv_assistants REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/rov_nl_pid_controller.py
    scripts/rov_mb_sm_controller.py
    scripts/rov_nmb_sm_controller.py
    scripts/rov_pid_controller.py
    scripts/rov_ua_pid_controller.py
    scripts/rov_mb_fl_controller.py
    scripts/auv_geometric_tracking_controller.py
    scripts/demo_wp_trajectory_generator.py
    scripts/rov_pd_grav_compensation_controller.py
    scripts/rov_sf_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)

  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    test_waypoint
    test/test_waypoint.py
  )

  ament_add_pytest_test(
    test_trajectory_point
    test/test_trajectory_point.py
  )

  ament_add_pytest_test(
    test_waypoint_set
    test/test_waypoint_set.py
  )

endif()

ament_package()
