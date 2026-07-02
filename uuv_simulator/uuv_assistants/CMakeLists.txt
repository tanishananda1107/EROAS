cmake_minimum_required(VERSION 3.8)

project(uuv_assistants)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/publish_footprints
    scripts/publish_footprints.py
    scripts/publish_vehicle_footprint.py
    scripts/publish_world_models
    scripts/publish_world_models.py
    scripts/publish_world_ned_frame
    scripts/set_simulation_timer.py
    scripts/unpause_simulation.py
    scripts/create_new_robot_model
    scripts/create_thruster_manager_configuration
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY src/tf_quaternion
  DESTINATION lib/python3/dist-packages
)

ament_package()
