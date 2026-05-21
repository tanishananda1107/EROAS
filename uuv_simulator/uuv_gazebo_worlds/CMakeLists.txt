cmake_minimum_required(VERSION 3.8)
project(uuv_gazebo_worlds)

find_package(ament_cmake REQUIRED)
find_package(python3_cmake_modules REQUIRED)

# Install Python launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
  PATTERN "*.launch" EXCLUDE
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
  PATTERN "*.launch.py"
)

ament_package()
