cmake_minimum_required(VERSION 3.8)
project(nps_uw_multibeam_sonar)

# CUDA setup
set(CMAKE_CUDA_COMPILER "nvcc")
enable_language(CUDA CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=sm_60")

# ── ROS 2 / ament ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(xacro REQUIRED)

# ── Gazebo Harmonic (Gz Sim 8) ─────────────────────────────────────────────────
find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-rendering8 REQUIRED)
find_package(gz-sensors8 REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

# ── Third-party ────────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(CUDAToolkit REQUIRED)       # replaces deprecated FindCUDA

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ── Helper: common ament deps list ────────────────────────────────────────────
set(COMMON_DEPS
  rclcpp
  std_msgs
  cv_bridge
  tf2
  tf2_ros
  marine_acoustic_msgs
)

# ── Plugin: raster-based ───────────────────────────────────────────────────────
add_library(nps_multibeam_sonar_ros_plugin SHARED
  src/gazebo_multibeam_sonar_raster_based.cpp
  src/sonar_calculation_cuda.cu
  src/MaterialSwitcher.cc
  src/SelectionBuffer.cc
  src/SelectionRenderListener.cc
)

set_target_properties(nps_multibeam_sonar_ros_plugin PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
)

target_include_directories(nps_multibeam_sonar_ros_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(nps_multibeam_sonar_ros_plugin
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-rendering8::gz-rendering8
  gz-sensors8::gz-sensors8
  CUDA::cudart
  CUDA::cufft
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(nps_multibeam_sonar_ros_plugin ${COMMON_DEPS})

# ── Plugin: ray-based ─────────────────────────────────────────────────────────
add_library(nps_multibeam_sonar_ray_ros_plugin SHARED
  src/gazebo_multibeam_sonar_ray_based.cpp
  src/sonar_calculation_cuda.cu
  src/MaterialSwitcher.cc
  src/SelectionBuffer.cc
  src/SelectionRenderListener.cc
)

set_target_properties(nps_multibeam_sonar_ray_ros_plugin PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
)

target_include_directories(nps_multibeam_sonar_ray_ros_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(nps_multibeam_sonar_ray_ros_plugin
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-rendering8::gz-rendering8
  gz-sensors8::gz-sensors8
  CUDA::cudart
  CUDA::cufft
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(nps_multibeam_sonar_ray_ros_plugin ${COMMON_DEPS})

# ── Install libraries ──────────────────────────────────────────────────────────
install(TARGETS
  nps_multibeam_sonar_ros_plugin
  nps_multibeam_sonar_ray_ros_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ── Install headers ────────────────────────────────────────────────────────────
install(DIRECTORY include/
  DESTINATION include
)

# ── Install resources ──────────────────────────────────────────────────────────
install(DIRECTORY launch worlds urdf models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE
)

# ── ament finalize ─────────────────────────────────────────────────────────────
ament_export_libraries(
  nps_multibeam_sonar_ros_plugin
  nps_multibeam_sonar_ray_ros_plugin
)
ament_export_dependencies(${COMMON_DEPS})
ament_package()
