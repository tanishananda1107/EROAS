cmake_minimum_required(VERSION 3.8)

project(uuv_descriptions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)

find_package(xacro REQUIRED)

install(
  DIRECTORY
    launch
    meshes
    robots
    urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.launch" EXCLUDE
)

if(BUILD_TESTING)

  find_package(
    ament_cmake_pytest
    REQUIRED
  )

  ament_add_pytest_test(
    test_urdf_files
    test/test_urdf_files.py
  )

endif()

ament_package()
