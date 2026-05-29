# ROS2 / Gazebo Harmonic (gz-sim 8) migration
# Changes from ROS1:
#   - cmake_minimum_required: raised to 3.8 (ament minimum for ROS2 Jazzy/Humble)
#   - Removed: find_package(catkin ...) — catkin is ROS1 only
#   - Added:   find_package(ament_cmake) + find_package(ament_cmake_python)
#   - Removed: catkin_python_setup() — replaced by ament_cmake_python
#   - Removed: catkin_package()       — replaced by ament_package()
#   - Replaced catkin_install_python() with ament_cmake_python install +
#     individual install(PROGRAMS ...) calls for the script files
#   - roslib / rospy are ROS1; their ROS2 equivalents (rclpy) are Python
#     dependencies declared in package.xml, not found via CMake.

cmake_minimum_required(VERSION 3.8)
project(uuv_smac_utils)

# ---------------------------------------------------------------------------
# Build-system dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ---------------------------------------------------------------------------
# Install the Python package (src/uuv_smac_utils/)
# ---------------------------------------------------------------------------
# ament_cmake_python installs the package and generates the necessary
# __init__.py / resource hooks so `import uuv_smac_utils` works at runtime.
ament_python_install_package(uuv_smac_utils
  PACKAGE_DIR src/uuv_smac_utils
)

# ---------------------------------------------------------------------------
# Install executable scripts
# ---------------------------------------------------------------------------
# In ROS2 scripts go to lib/<package_name>/ so `ros2 run` can find them.
install(PROGRAMS
  scripts/smac_wrapper
  scripts/run_smac
  scripts/create_smac_config_files
  scripts/evaluate_smac_best_results
  scripts/smac
  scripts/create_results_folder
  scripts/generate_motion_primitives
  scripts/sync_smac_files
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# (Optional) Testing
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ---------------------------------------------------------------------------
# Finalise the package
# ---------------------------------------------------------------------------
ament_package()
