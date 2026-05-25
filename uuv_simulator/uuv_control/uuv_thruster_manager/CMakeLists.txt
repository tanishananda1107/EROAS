cmake_minimum_required(VERSION 3.8)
project(uuv_thruster_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(uuv_gazebo_ros_plugins_msgs REQUIRED)

# Services
set(srv_files
  "srv/GetThrusterCurve.srv"
  "srv/ThrusterManagerInfo.srv"
  "srv/SetThrusterManagerConfig.srv"
  "srv/GetThrusterManagerConfig.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs
)

ament_python_install_package(uuv_thrusters)

install(
  PROGRAMS
    scripts/thruster_allocator.py
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
    test_thrusters
    test/test_thrusters.py
  )

  ament_add_pytest_test(
    test_thruster_allocator
    test/test_thruster_allocator.py
  )

  ament_add_pytest_test(
    test_thruster_manager
    test/test_thruster_manager_proportional_correct.py
  )
endif()

ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()
