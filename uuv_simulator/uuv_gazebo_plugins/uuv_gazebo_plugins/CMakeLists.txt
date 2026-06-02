cmake_minimum_required(VERSION 3.8)
project(uuv_gazebo_plugins)

# ── C++ standard ────────────────────────────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Build-tool ───────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)

# ── Gazebo Harmonic (gz-sim 8) stack ─────────────────────────────────────────
find_package(gz-sim8       REQUIRED COMPONENTS all)   # gz::sim::System, ECM, etc.
find_package(gz-plugin2    REQUIRED COMPONENTS register)
find_package(gz-math7      REQUIRED)                  # gz::math::Vector3d, Pose3d
find_package(gz-msgs10     REQUIRED)                  # gz::msgs::Vector3d
find_package(gz-transport13 REQUIRED)                 # gz::transport::Node
find_package(gz-utils2     REQUIRED)                  # GZ_UTILS_ASSERT
find_package(gz-common5    REQUIRED)                  # gzmsg, GZ_ASSERT helpers
find_package(sdformat14    REQUIRED)                  # sdf::ElementPtr, sdf::SDF

# ── Third-party ───────────────────────────────────────────────────────────────
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)   # Only needed for uuv_gazebo_plugins_msgs

# ── Collect list of all plugin targets (used for install) ─────────────────────
set(UUV_GAZEBO_PLUGINS_LIST "")

# ── Convenience alias so every target can use the same link list ──────────────
set(GZ_LIBS
  gz-sim8::gz-sim8
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
  gz-utils2::gz-utils2
  sdformat14::sdformat14
)

# ── Global include dirs ───────────────────────────────────────────────────────
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}   # generated protobuf headers
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(uuv_dynamics SHARED
  src/Dynamics.cc
)
target_include_directories(uuv_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(uuv_dynamics PUBLIC sdformat14::sdformat14)

add_library(uuv_thruster_conversion SHARED
  src/ThrusterConversionFcn.cc
)
target_include_directories(uuv_thruster_conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(uuv_thruster_conversion PUBLIC
  gz-common5::gz-common5
  sdformat14::sdformat14
)

add_library(uuv_thruster_plugin SHARED
  src/ThrusterPlugin.cc
)
target_include_directories(uuv_thruster_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(uuv_thruster_plugin PUBLIC
  uuv_dynamics
  uuv_thruster_conversion
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
  gz-common5::gz-common5
  sdformat14::sdformat14
)

add_library(uuv_fin_plugin SHARED
  src/LiftDragModel.cc
  src/FinPlugin.cc
)
target_include_directories(uuv_fin_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(uuv_fin_plugin PUBLIC
  uuv_dynamics
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
  gz-common5::gz-common5
  sdformat14::sdformat14
)

add_library(uuv_underwater_object_plugin SHARED
  src/UnderwaterObjectPlugin.cc
)
target_include_directories(uuv_underwater_object_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(uuv_underwater_object_plugin PUBLIC
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
  gz-common5::gz-common5
  sdformat14::sdformat14
)

install(TARGETS
    uuv_dynamics
    uuv_thruster_conversion
    uuv_thruster_plugin
    uuv_fin_plugin
    uuv_underwater_object_plugin
  EXPORT uuv_gazebo_pluginsCoreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hh"
)

ament_export_include_directories(include)
ament_export_targets(uuv_gazebo_pluginsCoreTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  gz-sim8
  gz-plugin2
  gz-math7
  gz-msgs10
  gz-transport13
  gz-utils2
  gz-common5
  sdformat14
  Eigen3
)

ament_package()
