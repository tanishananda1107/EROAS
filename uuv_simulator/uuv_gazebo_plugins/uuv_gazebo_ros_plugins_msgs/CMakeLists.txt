cmake_minimum_required(VERSION 3.8)
project(uuv_gazebo_ros_plugins_msgs)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FloatStamped.msg"
  "msg/UnderwaterObjectModel.msg"
  "msg/ThrusterConversionFcn.msg"
  "srv/SetUseGlobalCurrentVel.srv"
  "srv/GetThrusterState.srv"
  "srv/GetFloat.srv"
  "srv/GetThrusterConversionFcn.srv"
  "srv/SetThrusterState.srv"
  "srv/GetListParam.srv"
  "srv/SetFloat.srv"
  "srv/GetModelProperties.srv"
  "srv/GetThrusterEfficiency.srv"
  "srv/SetThrusterEfficiency.srv"
)

ament_package()
