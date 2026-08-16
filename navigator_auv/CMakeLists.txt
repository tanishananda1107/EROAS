cmake_minimum_required(VERSION 3.8)
project(navigator_auv)


find_package(ament_cmake REQUIRED)


install(PROGRAMS
  scripts/sonar_reconstruction.py
  scripts/helper\ nodes/image_saver.py
  scripts/only_gap.py
  scripts/sonar_frame_converter.py
  scripts/hover_hold.py
  scripts/pose_gt_to_odom.py
  scripts/sonar_ray_markers.py
  scripts/spawner.py
  scripts/trail_marker.py
  scripts/velocity_cbf.py
  scripts/waypoint_hover.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(navigator_auv_paper_function_tests test/test_paper_functions.py)
endif()

ament_package()
