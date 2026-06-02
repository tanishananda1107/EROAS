cmake_minimum_required(VERSION 3.8)
project(rexrov2_description)

# =========================
# ROS2 build system
# =========================
find_package(ament_cmake REQUIRED)

# =========================
# Install robot assets
# =========================
install(
  DIRECTORY meshes robots urdf launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE
  PATTERN "*.launch" EXCLUDE
)

# (Removed duplicate install block - was redundant in ROS1 version)

# =========================
# Testing (ROS2 equivalent)
# =========================
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(xacro REQUIRED)

  # Example: Python or launch testing can go here
  # ROS1 rostest/rosunit -> replaced by pytest or launch_testing in ROS2
endif()

# =========================
# Export package
# =========================
ament_package()
