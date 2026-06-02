cmake_minimum_required(VERSION 3.16)
project(nps_uw_multibeam_sonar)

# ── C++ / CUDA standard ───────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CUDA_COMPILER "nvcc")
enable_language(CUDA CXX)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=sm_60")
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set_property(GLOBAL PROPERTY CUDA_SEPARABLE_COMPILATION ON)

# ── GCC 13 ABI fix — MUST come before any find_package ───────────────────────
# Ogre-Next 2.3 on Ubuntu Noble was compiled with _GLIBCXX_USE_CXX11_ABI=1.
# GCC 13 removed _M_max_size() from __new_allocator — without this define,
# any TU that includes Ogre headers gets "has no member named '_M_max_size'".
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)
add_compile_options(
  -Wno-deprecated-declarations
  -Wno-deprecated
)

# ── ament / ROS 2 ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(pcl_conversions REQUIRED)

# ── Gazebo Harmonic (gz-sim 8) ────────────────────────────────────────────────
find_package(gz-sim8 REQUIRED)
find_package(gz-rendering8 REQUIRED COMPONENTS ogre2)
find_package(gz-sensors8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED)

# ── Third-party ───────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(CUDAToolkit REQUIRED)

# ── Ogre-Next 2.3 — use pkg-config with confirmed .pc name ───────────────────
# .pc file confirmed: /usr/lib/x86_64-linux-gnu/pkgconfig/OGRE-2.3.pc
# .so confirmed:      /usr/lib/x86_64-linux-gnu/OGRE-2.3/libOgreNextMain.so
find_package(PkgConfig REQUIRED)

pkg_check_modules(OGRE_NEXT REQUIRED OGRE-2.3)
# Also pull in gz-rendering8's ogre2 component flags — covers any
# extra defines gz uses when wrapping Ogre-Next
pkg_check_modules(GZ_OGRE2 REQUIRED gz-rendering8-ogre2)

message(STATUS "OGRE-Next include dirs : ${OGRE_NEXT_INCLUDE_DIRS}")
message(STATUS "OGRE-Next library dirs : ${OGRE_NEXT_LIBRARY_DIRS}")
message(STATUS "OGRE-Next libraries    : ${OGRE_NEXT_LIBRARIES}")
message(STATUS "gz-rendering8-ogre2    : ${GZ_OGRE2_LIBRARIES}")

# ── Global include dirs ───────────────────────────────────────────────────────
include_directories(
  include
  src
  ${OGRE_NEXT_INCLUDE_DIRS}               # /usr/include/OGRE-2.3
  ${GZ_OGRE2_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OGRE_NEXT_LIBRARY_DIRS}               # /usr/lib/x86_64-linux-gnu/OGRE-2.3
)

add_definitions(
  ${PCL_DEFINITIONS}
  ${OGRE_NEXT_CFLAGS_OTHER}               # any extra -D flags from the .pc file
  ${GZ_OGRE2_CFLAGS_OTHER}
)

# ── Common ROS 2 ament deps ───────────────────────────────────────────────────
set(ROS2_DEPS
  ament_index_cpp
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  image_transport
  cv_bridge
  marine_acoustic_msgs
)

# ── Common GZ link libraries ──────────────────────────────────────────────────
set(GZ_LIBS
  gz-sim8::gz-sim8
  gz-rendering8::gz-rendering8
  gz-sensors8::gz-sensors8
  gz-transport13::gz-transport13
  gz-math7::gz-math7
  gz-common5::gz-common5
  gz-plugin2::register
)

# ── Common compile options ────────────────────────────────────────────────────
set(PLUGIN_COMPILE_OPTIONS
  -D_GLIBCXX_USE_CXX11_ABI=1
  -DOGRE_IGNORE_UNKNOWN_DEBUG
  -Wno-deprecated-declarations
  -Wno-deprecated
)

# ── Common private include dirs (per-target, not global) ─────────────────────
set(PLUGIN_INCLUDE_DIRS
  include
  include/selection_buffer
  src
  ${OGRE_NEXT_INCLUDE_DIRS}
  ${GZ_OGRE2_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ── Common selection-buffer sources ──────────────────────────────────────────
set(SELECTION_BUFFER_SRCS
  src/MaterialSwitcher.cc
  src/SelectionBuffer.cc
  src/SelectionRenderListener.cc
)

# ─────────────────────────────────────────────────────────────────────────────
# TARGET 1 — raster-based (depth-camera) sonar plugin
# ─────────────────────────────────────────────────────────────────────────────
add_library(nps_multibeam_sonar_ros_plugin SHARED
  src/gazebo_multibeam_sonar_raster_based.cpp
  src/sonar_calculation_cuda.cu
  ${SELECTION_BUFFER_SRCS}
)

set_target_properties(nps_multibeam_sonar_ros_plugin PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
)

target_include_directories(nps_multibeam_sonar_ros_plugin PRIVATE
  ${PLUGIN_INCLUDE_DIRS}
)

target_compile_options(nps_multibeam_sonar_ros_plugin PRIVATE
  ${PLUGIN_COMPILE_OPTIONS}
)

ament_target_dependencies(nps_multibeam_sonar_ros_plugin
  ${ROS2_DEPS} pcl_conversions
)

target_link_libraries(nps_multibeam_sonar_ros_plugin
  ${GZ_LIBS}
  ${OGRE_NEXT_LIBRARIES}         # OgreNextMain — from OGRE-2.3.pc
  ${GZ_OGRE2_LIBRARIES}          # gz-rendering8 ogre2 wrapper
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  CUDA::cudart
  CUDA::cufft
)

# ─────────────────────────────────────────────────────────────────────────────
# TARGET 2 — ray-based (GPU lidar) sonar plugin
# ─────────────────────────────────────────────────────────────────────────────
add_library(nps_multibeam_sonar_ray_ros_plugin SHARED
  src/gazebo_multibeam_sonar_ray_based.cpp
  src/sonar_calculation_cuda.cu
  ${SELECTION_BUFFER_SRCS}
)

set_target_properties(nps_multibeam_sonar_ray_ros_plugin PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
)

target_include_directories(nps_multibeam_sonar_ray_ros_plugin PRIVATE
  ${PLUGIN_INCLUDE_DIRS}
)

target_compile_options(nps_multibeam_sonar_ray_ros_plugin PRIVATE
  ${PLUGIN_COMPILE_OPTIONS}
)

ament_target_dependencies(nps_multibeam_sonar_ray_ros_plugin
  ${ROS2_DEPS} pcl_conversions
)

target_link_libraries(nps_multibeam_sonar_ray_ros_plugin
  ${GZ_LIBS}
  ${OGRE_NEXT_LIBRARIES}         # OgreNextMain — from OGRE-2.3.pc
  ${GZ_OGRE2_LIBRARIES}          # gz-rendering8 ogre2 wrapper
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  CUDA::cudart
  CUDA::cufft
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  nps_multibeam_sonar_ros_plugin
  nps_multibeam_sonar_ray_ros_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  worlds
  urdf
  models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
