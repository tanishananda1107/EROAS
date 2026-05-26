cmake_minimum_required(VERSION 3.8)
project(uuv_gazebo_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  Media
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE
)

ament_package()
