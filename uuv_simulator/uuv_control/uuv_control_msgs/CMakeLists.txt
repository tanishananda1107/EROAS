cmake_minimum_required(VERSION 3.8)
project(uuv_control_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Trajectory.msg"
  "msg/TrajectoryPoint.msg"
  "msg/Waypoint.msg"
  "msg/WaypointSet.msg"
)

set(srv_files
  "srv/AddWaypoint.srv"
  "srv/ClearWaypoints.srv"
  "srv/InitCircularTrajectory.srv"
  "srv/InitHelicalTrajectory.srv"
  "srv/InitWaypointsFromFile.srv"
  "srv/GetWaypoints.srv"
  "srv/GoTo.srv"
  "srv/GoToIncremental.srv"
  "srv/Hold.srv"
  "srv/IsRunningTrajectory.srv"
  "srv/InitWaypointSet.srv"
  "srv/InitRectTrajectory.srv"
  "srv/StartTrajectory.srv"
  "srv/SwitchToAutomatic.srv"
  "srv/SwitchToManual.srv"
  "srv/SetPIDParams.srv"
  "srv/GetPIDParams.srv"
  "srv/SetSMControllerParams.srv"
  "srv/GetSMControllerParams.srv"
  "srv/SetMBSMControllerParams.srv"
  "srv/GetMBSMControllerParams.srv"
  "srv/ResetController.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
