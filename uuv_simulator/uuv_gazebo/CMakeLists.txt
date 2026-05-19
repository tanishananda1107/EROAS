cmake_minimum_required(VERSION 3.8)
project(uuv_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
