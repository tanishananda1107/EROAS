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

# ── ament / ROS 2 ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
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
find_package(gz-rendering8 REQUIRED)
find_package(gz-sensors8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED)

# ── Third-party ───────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(CUDAToolkit REQUIRED)

# ── Global include dirs ───────────────────────────────────────────────────────
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ── Common ROS 2 ament deps (used by both targets) ───────────────────────────
set(ROS2_DEPS
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  image_transport
  cv_bridge
  marine_acoustic_msgs
)

# ─────────────────────────────────────────────────────────────────────────────
# TARGET 1 — raster-based (depth-camera) sonar plugin
# ─────────────────────────────────────────────────────────────────────────────
add_library(nps_multibeam_sonar_ros_plugin SHARED
  src/gazebo_multibeam_sonar_raster_based.cpp
  src/sonar_calculation_cuda.cu
  src/MaterialSwitcher.cc          # empty shim — Ogre removed, see header
  src/SelectionBuffer.cc           # gz::rendering::RayQuery implementation
  src/SelectionRenderListener.cc   # empty shim — no longer needed
)

set_target_properties(nps_multibeam_sonar_ros_plugin PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
)

# sonar_calculation_cuda.cuh lives in src/ — use quotes in .cu, add src here
target_include_directories(nps_multibeam_sonar_ros_plugin PRIVATE
  include
  src
)

# FIXED: Target name corrected to match add_library above
ament_target_dependencies(nps_multibeam_sonar_ros_plugin
  rclcpp sensor_msgs std_msgs geometry_msgs
  image_transport cv_bridge marine_acoustic_msgs
  pcl_conversions        
)

target_link_libraries(nps_multibeam_sonar_ros_plugin
  gz-sim8::gz-sim8
  gz-rendering8::gz-rendering8
  gz-sensors8::gz-sensors8
  gz-transport13::gz-transport13
  gz-math7::gz-math7
  gz-common5::gz-common5
  gz-plugin2::register
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
  src/MaterialSwitcher.cc          # empty shim
  src/SelectionBuffer.cc           # gz::rendering::RayQuery implementation
  src/SelectionRenderListener.cc   # empty shim
)

set_target_properties(nps_multibeam_sonar_ray_ros_plugin PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
)

target_include_directories(nps_multibeam_sonar_ray_ros_plugin PRIVATE
  include
  src
)

ament_target_dependencies(nps_multibeam_sonar_ray_ros_plugin ${ROS2_DEPS})

target_link_libraries(nps_multibeam_sonar_ray_ros_plugin
  gz-sim8::gz-sim8
  gz-rendering8::gz-rendering8
  gz-sensors8::gz-sensors8
  gz-transport13::gz-transport13
  gz-math7::gz-math7
  gz-common5::gz-common5
  gz-plugin2::register
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
