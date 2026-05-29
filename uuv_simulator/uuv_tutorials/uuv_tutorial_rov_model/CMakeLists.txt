cmake_minimum_required(VERSION 3.8)

project(uuv_tutorial_rov_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)

# ------------------------------------------------------------------------------
# Install robot resources
# ------------------------------------------------------------------------------

install(
  DIRECTORY
    meshes
    mesh
    robots
    urdf
    launch
    config
    worlds
    rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ------------------------------------------------------------------------------
# Install xacro files separately (optional but recommended)
# ------------------------------------------------------------------------------

install(
  DIRECTORY
    robots/
  DESTINATION share/${PROJECT_NAME}/robots
  FILES_MATCHING PATTERN "*.xacro"
)

# ------------------------------------------------------------------------------
# Export package
# ------------------------------------------------------------------------------

ament_package()
