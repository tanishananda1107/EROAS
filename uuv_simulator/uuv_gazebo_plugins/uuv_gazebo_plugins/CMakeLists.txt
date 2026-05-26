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
find_package(gz-math7      REQUIRED)                  # gz::math::Vector3d, Pose3d
find_package(gz-msgs10     REQUIRED)                  # gz::msgs::Vector3d
find_package(gz-transport13 REQUIRED)                 # gz::transport::Node
find_package(gz-utils2     REQUIRED)                  # GZ_UTILS_ASSERT
find_package(sdformat13    REQUIRED)                  # sdf::ElementPtr, sdf::SDF

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
  sdformat13::sdformat13
)

# ── Global include dirs ───────────────────────────────────────────────────────
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}   # generated protobuf headers
  ${EIGEN3_INCLUDE_DIRS}
)

# ────────────────────────────────────────────────────────────────────────────
# Custom protobuf messages  (replaces the old Gazebo PROTOBUF_IMPORT_DIRS
# dance; gz-msgs10 supplies the .proto include path automatically)
# ────────────────────────────────────────────────────────────────────────────
file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/msgs)
file(GLOB UUV_PROTO_SRCS msgs/*.proto)

if(UUV_PROTO_SRCS)
  # gz-msgs ships its own .proto files; tell protoc where to find them.
  get_target_property(GZ_MSGS_INCLUDE gz-msgs10::gz-msgs10 INTERFACE_INCLUDE_DIRECTORIES)

  protobuf_generate_cpp(PROTO_CC PROTO_HH ${UUV_PROTO_SRCS})

  add_library(uuv_gazebo_plugins_msgs SHARED ${PROTO_CC})
  target_include_directories(uuv_gazebo_plugins_msgs PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}
    ${GZ_MSGS_INCLUDE}
  )
  target_link_libraries(uuv_gazebo_plugins_msgs
    PUBLIC
      protobuf::libprotobuf
      gz-msgs10::gz-msgs10
  )
  list(APPEND UUV_GAZEBO_PLUGINS_LIST uuv_gazebo_plugins_msgs)
endif()

# ────────────────────────────────────────────────────────────────────────────
# uuv_dynamics
# ────────────────────────────────────────────────────────────────────────────
add_library(uuv_dynamics SHARED
  src/Dynamics.cc
)
target_include_directories(uuv_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(uuv_dynamics PUBLIC ${GZ_LIBS})
list(APPEND UUV_GAZEBO_PLUGINS_LIST uuv_dynamics)

# ────────────────────────────────────────────────────────────────────────────
# uuv_fin_plugin
# ────────────────────────────────────────────────────────────────────────────
add_library(uuv_fin_plugin SHARED
  src/LiftDragModel.cc
  src/FinPlugin.cc
)
target_include_directories(uuv_fin_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(uuv_fin_plugin
  PUBLIC
    uuv_dynamics
    ${GZ_LIBS}
  PRIVATE
    $<$<TARGET_EXISTS:uuv_gazebo_plugins_msgs>:uuv_gazebo_plugins_msgs>
)
list(APPEND UUV_GAZEBO_PLUGINS_LIST uuv_fin_plugin)

# ────────────────────────────────────────────────────────────────────────────
# uuv_underwater_object_plugin
# ────────────────────────────────────────────────────────────────────────────
add_library(uuv_underwater_object_plugin SHARED
  src/HydrodynamicModel.cc
  src/BuoyantObject.cc
  src/UnderwaterObjectPlugin.cc
)
target_include_directories(uuv_underwater_object_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(uuv_underwater_object_plugin
  PUBLIC
    ${GZ_LIBS}
    Eigen3::Eigen
)
list(APPEND UUV_GAZEBO_PLUGINS_LIST uuv_underwater_object_plugin)

# ────────────────────────────────────────────────────────────────────────────
# uuv_thruster_plugin
# ────────────────────────────────────────────────────────────────────────────
add_library(uuv_thruster_plugin SHARED
  src/ThrusterConversionFcn.cc
  src/ThrusterPlugin.cc
)
target_include_directories(uuv_thruster_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(uuv_thruster_plugin
  PUBLIC
    uuv_dynamics
    ${GZ_LIBS}
  PRIVATE
    $<$<TARGET_EXISTS:uuv_gazebo_plugins_msgs>:uuv_gazebo_plugins_msgs>
)
list(APPEND UUV_GAZEBO_PLUGINS_LIST uuv_thruster_plugin)

# ────────────────────────────────────────────────────────────────────────────
# umbilical_plugin
# ────────────────────────────────────────────────────────────────────────────
add_library(umbilical_plugin SHARED
  src/UmbilicalModel.cc
  src/UmbilicalPlugin.cc
)
target_include_directories(umbilical_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(umbilical_plugin
  PUBLIC ${GZ_LIBS}
)
list(APPEND UUV_GAZEBO_PLUGINS_LIST umbilical_plugin)

# ────────────────────────────────────────────────────────────────────────────
# Export / Install
# ────────────────────────────────────────────────────────────────────────────
install(TARGETS ${UUV_GAZEBO_PLUGINS_LIST}
  EXPORT  uuv_gazebo_pluginsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hh"
)

# Install generated protobuf headers.
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
    PATTERN "*.pb.h"
    PATTERN "*~" EXCLUDE
)

# ament exports so downstream packages can find our targets.
ament_export_targets(uuv_gazebo_pluginsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  gz-sim8
  gz-math7
  gz-msgs10
  gz-transport13
  gz-utils2
  sdformat13
  Eigen3
)

# ────────────────────────────────────────────────────────────────────────────
# Testing
# ────────────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(UNIT_ThrusterConversionFcn_TEST
    src/ThrusterConversionFcn_TEST.cc
  )
  target_link_libraries(UNIT_ThrusterConversionFcn_TEST
    uuv_thruster_plugin
    ${GZ_LIBS}
  )

  ament_add_gtest(UNIT_ThrusterDynamics_TEST
    src/ThrusterDynamics_TEST.cc
  )
  target_link_libraries(UNIT_ThrusterDynamics_TEST
    uuv_dynamics
    ${GZ_LIBS}
  )
endif()

ament_package()
