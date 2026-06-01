cmake_minimum_required(VERSION 3.8)
project(navigator_auv)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/sonar_heading_node.py
  scripts/move_sonar.py
  scripts/local_planner_heading.py
  scripts/altitude_control.py
  scripts/contour_heading.py
  scripts/sonar_reconstruction.py
  scripts/image_saver.py
  scripts/euler_calculator.py
  scripts/move_obstacle.py
  scripts/cbf_implementation.py
  scripts/just_cbf.py
  scripts/obstacle_draw.py
  scripts/only_gap.py
  scripts/pose_plotter.py
  scripts/spawner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
