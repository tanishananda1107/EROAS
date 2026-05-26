cmake_minimum_required(VERSION 3.8)
project(uuv_gazebo_ros_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── ROS 2 dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(uuv_gazebo_ros_plugins_msgs REQUIRED)
find_package(uuv_gazebo_plugins REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# ── Gazebo Harmonic (gz-sim8) ─────────────────────────────────────────────────
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Convenience alias targets supplied by gz-cmake
set(GZ_SIM_VER     ${gz-sim8_VERSION_MAJOR})
set(GZ_PLUGIN_VER  ${gz-plugin2_VERSION_MAJOR})

# ── Common dependency list for ament_target_dependencies ──────────────────────
set(COMMON_DEPS
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  uuv_gazebo_ros_plugins_msgs
  uuv_gazebo_plugins
  tf2_ros
  tf2
)

include_directories(
  include
)

# ── Helper macro: build one gz-sim plugin shared library ─────────────────────
macro(add_gz_plugin TARGET SOURCE)
  add_library(${TARGET} SHARED ${SOURCE})
  target_include_directories(${TARGET} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${TARGET} ${COMMON_DEPS})
  target_link_libraries(${TARGET}
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    gz-plugin${GZ_PLUGIN_VER}::register
    gz-math7::gz-math7)
endmacro()

# ── Plugin targets ─────────────────────────────────────────────────────────────
add_gz_plugin(uuv_fin_ros_plugin               src/FinROSPlugin.cc)
add_gz_plugin(uuv_thruster_ros_plugin          src/ThrusterROSPlugin.cc)
add_gz_plugin(uuv_underwater_object_ros_plugin src/UnderwaterObjectROSPlugin.cc)
add_gz_plugin(uuv_joint_state_publisher        src/JointStatePublisher.cc)
add_gz_plugin(uuv_accelerations_test_plugin    src/AccelerationsTestPlugin.cc)
add_gz_plugin(linear_battery_ros_plugin        src/LinearBatteryROSPlugin.cc)
add_gz_plugin(custom_battery_consumer_ros_plugin src/CustomBatteryConsumerROSPlugin.cc)

set(UUV_GAZEBO_ROS_PLUGINS_LIST
  uuv_fin_ros_plugin
  uuv_thruster_ros_plugin
  uuv_underwater_object_ros_plugin
  uuv_joint_state_publisher
  uuv_accelerations_test_plugin
  linear_battery_ros_plugin
  custom_battery_consumer_ros_plugin
)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS ${UUV_GAZEBO_ROS_PLUGINS_LIST}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}/test
  OPTIONAL
)

# ── Tests ──────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  foreach(test_script
      test/test_default_fossen_vehicle.py
      test/test_sphere_vehicle.py
      test/test_thrusters.py
  )
    get_filename_component(test_name ${test_script} NAME_WE)
    ament_add_pytest_test(${test_name} ${test_script}
      TIMEOUT 120
    )
  endforeach()
endif()

# ── Export ─────────────────────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(${UUV_GAZEBO_ROS_PLUGINS_LIST})
ament_export_dependencies(${COMMON_DEPS})

ament_package()
