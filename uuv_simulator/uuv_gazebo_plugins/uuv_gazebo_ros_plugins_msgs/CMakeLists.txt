cmake_minimum_required(VERSION 3.8)
project(uuv_gazebo_ros_plugins_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
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
  "srv/SetThrusterEfficiency.srv"
  "srv/SetThrusterState.srv"
  "srv/GetThrusterEfficiency.srv"
  "srv/GetThrusterState.srv"
  "srv/SetUseGlobalCurrentVel.srv"
  "srv/GetModelProperties.srv"
  "srv/SetFloat.srv"
  "srv/GetFloat.srv"
  "srv/GetListParam.srv"
  "srv/GetThrusterConversionFcn.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
