cmake_minimum_required(VERSION 3.8)
project(uuv_control_msgs)

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
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/Trajectory.msg"
  "msg/TrajectoryPoint.msg"
  "msg/WaypointSet.msg"
  "srv/SetSMControllerParams.srv"
  "srv/IsRunningTrajectory.srv"
  "srv/InitCircularTrajectory.srv"
  "srv/ClearWaypoints.srv"
  "srv/SetMBSMControllerParams.srv"
  "srv/SwitchToManual.srv"
  "srv/GetMBSMControllerParams.srv"
  "srv/InitWaypointSet.srv"
  "srv/InitWaypointsFromFile.srv"
  "srv/GetSMControllerParams.srv"
  "srv/GoToIncremental.srv"
  "srv/InitHelicalTrajectory.srv"
  "srv/GetWaypoints.srv"
  "srv/AddWaypoint.srv"
  "srv/StartTrajectory.srv"
  "srv/InitRectTrajectory.srv"
  "srv/SwitchToAutomatic.srv"
  "srv/ResetController.srv"
  "srv/GetPIDParams.srv"
  "srv/GoTo.srv"
  "srv/SetPIDParams.srv"
  "srv/Hold.srv"
)

ament_package()
