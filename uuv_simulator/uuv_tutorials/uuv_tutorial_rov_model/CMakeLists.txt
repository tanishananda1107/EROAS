cmake_minimum_required(VERSION 3.8)
project(uuv_tutorial_rov_model)

find_package(ament_cmake REQUIRED)

# Install xacro files
install(DIRECTORY
  robots
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install mesh files
install(DIRECTORY
  mesh
  DESTINATION share/${PROJECT_NAME}
)

ament_package()