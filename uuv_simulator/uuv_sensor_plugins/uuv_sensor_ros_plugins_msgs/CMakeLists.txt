cmake_minimum_required(VERSION 3.8)
project(uuv_sensor_ros_plugins_msgs)

if(CMAKE_VERSION VERSION_LESS 3.10)
  set(CMAKE_CXX_STANDARD 11)
else()
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX || CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChemicalParticleConcentration.msg"
  "msg/Salinity.msg"
  "msg/DVLBeam.msg"
  "msg/DVL.msg"
  "msg/PositionWithCovariance.msg"
  "msg/PositionWithCovarianceStamped.msg"
  "srv/ChangeSensorState.srv"
)

ament_package()
