cmake_minimum_required(VERSION 3.5)
project(uuv_simulation_evaluation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/evaluate_bag
    scripts/run_best_worst_comparison
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_constraint test/test_constraint.py)
  ament_add_pytest_test(test_cost_function test/test_cost_function.py)
  ament_add_pytest_test(test_evaluate_trajectory test/test_evaluate_trajectory.py)
  ament_add_pytest_test(test_kpis test/test_kpis.py)

endif()

ament_package()
