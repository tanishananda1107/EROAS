cmake_minimum_required(VERSION 3.8)
project(uuv_sensor_ros_plugins)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE)

ament_package()
