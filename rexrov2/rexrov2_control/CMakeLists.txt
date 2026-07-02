cmake_minimum_required(VERSION 3.8)
project(rexrov2_control)

# =========================
# ROS2 build system
# =========================
find_package(ament_cmake REQUIRED)

# =========================
# Install directories
# =========================
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE
  PATTERN "*.launch" EXCLUDE
)

# =========================
# Gazebo Harmonic (gz-sim 8) integration (optional but recommended)
# =========================
find_package(gz-sim8 REQUIRED)

# If you later add plugins, you will use:
# add_library(... SHARED ...)
# target_link_libraries(... gz-sim8::gz-sim8)

# =========================
# Export package
# =========================
ament_package()
