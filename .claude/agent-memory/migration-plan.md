# ROS2 Migration Plan - EROAS UUV Simulator

## Phase 1: Scan & Plan - Complete

### Package Structure Identified:
1. **uuv_gazebo** - ROS2 already (format=3, ament_cmake)
2. **uuv_gazebo_plugins** - ROS2 already (format=3, ament_cmake)
3. **uuv_gazebo_ros_plugins** - ROS2 already (format=3, ament_cmake)
4. **uuv_gazebo_ros_plugins_msgs** - ROS2 already (format=3, ament_cmake, rosidl_interface_packages)
5. **uuv_world_plugins** - ROS2 already (format=3, ament_cmake)
6. **uuv_world_ros_plugins** - ROS2 already (format=3, ament_cmake)
7. **uuv_world_ros_plugins_msgs** - ROS2 already (format=3, ament_cmake, rosidl_interface_packages)
8. **uuv_tutorial_seabed_world** - ROS1 (catkin) - Needs conversion
9. **uuv_tutorial_dp_controller** - ROS1 (catkin) - Needs conversion
10. **uuv_tutorial_disturbances** - ROS1 (catkin) - Needs conversion
11. **uuv_tutorials** - ROS1 (catkin) - Needs conversion
12. **uuv_tutorial_rov_model** - ROS1 (catkin) - Needs conversion
13. **uuv_simulation_evaluation** - ROS1 (catkin) - Needs conversion
14. **uuv_simulation_wrapper** - ROS1 (catkin) - Needs conversion
15. **uuv_smac_utils** - ROS1 (catkin) - Needs conversion
16. **uuv_control_msgs** - ROS2 already (format=3, ament_cmake, rosidl_interface_packages)
17. **uuv_trajectory_control** - ROS2 already (format=3, ament_cmake, rclpy)
18. **uuv_thruster_manager** - ROS2 already (format=3, ament_cmake, rclpy)
19. **uuv_control_cascaded_pids** - ROS2 already (format=3, ament_cmake, rclpy)
20. **uuv_auv_control_allocator** - ROS2 already (format=2, ament_cmake, rclpy)
21. **uuv_teleop** - ROS2 already (format=3, ament_cmake, rclpy)
22. **uuv_simulator** - ROS2 already (format=3, metapackage)

## Phase 2: Convert package.xml files - Complete

### All package.xml files converted (catkin -> ament_cmake):
- uuv_control_msgs
- uuv_gazebo_ros_plugins_msgs
- uuv_sensor_ros_plugins_msgs
- uuv_thruster_manager_msgs
- uuv_trajectory_control_msgs
- uuv_world_ros_plugins_msgs
- uuv_assistants
- uuv_control
- uuv_control_cascaded_pid
- uuv_control_utils
- uuv_descriptions
- uuv_gazebo
- uuv_gazebo_plugins
- uuv_gazebo_ros_plugins
- uuv_gazebo_worlds
- uuv_sensor_plugins
- uuv_sensor_ros_plugins
- uuv_thruster_manager
- uuv_trajectory_control
- uuv_world_plugins
- uuv_world_ros_plugins
- uuv_tutorials/uuv_tutorials_control
- uuv_tutorials/uuv_tutorials_description
- uuv_tutorials/uuv_tutorials_gazebo
- uuv_tutorials/uuv_tutorials_gazebo_ros_plugins
- uuv_tutorials/uuv_tutorials_navigation
- uuv_tutorials/uuv_tutorials_python
- uuv_tutorials/uuv_tutorials_terrains
- uuv_tutorials/uuv_tutorials_worlds

### Conversion rules applied:
- format="2" -> format="3"
- buildtool_depend catkin -> ament_cmake
- run_depend -> exec_depend
- Remove $ROS_PYTHON_VERSION conditions
- Ensure proper export tag structure
- Keep rosidl_interface_packages group for msg/srv packages
- Update Python dependencies to python3-* format
- Add member_of_group for custom message packages

## Phase 6: Launch System Conversion - In Progress

### Launch Files Discovered:

**uuv_gazebo/launch:**
- ocean_waves.launch (world file)
- ocean_waves_with_rov.launch
- ocean_waves_with_rov_logitech_joy.launch
- ocean_waves_with_oberon.launch
- ocean_waves_with_oberon_logitech_joy.launch
- ocean_waves_with_oberon4.launch
- ocean_waves_with_oberon4_logitech_joy.launch
- ocean_waves_with_oberon_arms.launch
- ocean_waves_with_oberon_arms_logitech_joy.launch
- rexrov_demos/rexrov_default.launch
- rexrov_demos/rexrov_default_logitech_joy.launch
- rexrov_demos/rexrov_oberon_demo.launch
- rexrov_demos/rexrov_oberon4_demo.launch
- rexrov_demos/rexrov_oberon_arms_demo.launch
- rexrov_demos/rexrov_oberon_demo_logitech_joy.launch
- rexrov_demos/rexrov_oberon4_demo_logitech_joy.launch
- rexrov_demos/rexrov_oberon_arms_demo_logitech_joy.launch
- controller_demos/record_demo.launch
- controller_demos/rover_dp_demo.launch
- controller_demos/rover_dp_demo_logitech_joy.launch
- controller_demos/rover_dp_demo_with_disturbances.launch

**uuv_teleop/launch:**
- uuv_teleop.launch
- uuv_oberon7_teleop.launch
- uuv_oberon4_teleop.launch

**uuv_descriptions/launch:**
- upload_rexrov.launch
- upload_rexrov_oberon7.launch
- upload_rexrov_oberon4.launch
- upload_rexrov_oberon_arms.launch

### Launch Conversion Rules:
- `<launch>` -> `def launch():`
- `<arg name="x" default="0"/>` -> `DeclareReal(name='x', default_value=0)`
- `<node pkg="pkg" type="type" name="name">` -> `Node(package='pkg', executable='type', name='name')`
- `<include file="$(find pkg)/path.launch">` -> `IncludeLaunchDescription(...)`
- `<param name="x" value="y"/>` -> `Parameter(name='x', value='y')`
- `<remap from="from" to="to"/>` -> `remappings=[('from', 'to')]`
- `<group ns="namespace">` -> `GroupLaunch(...)`
- `<group if="$(arg condition)">` -> `If(condition, ...)`
- `<group unless="$(arg condition)">` -> `Unless(condition, ...)`

## Phase 3: CMakeLists.txt conversion - Pending

### CMake conversion rules:
- catkin_package -> ament_package
- find_package(catkin REQUIRED ...) -> find_package(ament_cmake REQUIRED ...)
- catkin_package() -> ament_package()
- catkin_install_python() -> install(PYTHON ...)
- catkin_python_setup() -> ament_python_install_package()
- catkin_add_nosetests() -> ament_add_pytest_test()
- catkin_lint -> ament_lint_*
- catkin_test_dependencies -> ament_lint_*

## Phase 4: Source code migration - Pending

### Python code changes:
- import rospy -> from rclpy.node import Node
- rospy.init_node() -> rclpy.init()
- rospy.Publisher() -> create_publisher()
- rospy.Subscriber() -> create_subscription()
- rospy.Service() -> create_service()
- rospy.Time.now() -> rclpy.clock.Clock().now()
- rospy.sleep() -> rclpy.time.Time()

### C++ code changes:
- #include <ros/ros.h> -> #include <rclcpp/rclcpp.hpp>
- ROS_INFO() -> RCLCPP_INFO()
- ros::spin() -> rclcpp::spin()
- ros::NodeHandle -> rclcpp::Node

## Phase 5: Build & Test - Pending

### Build requirements:
- ROS2 Jazzy (or Humble)
- gazebo_ros_pkgs
- ros_gz_sim (if using Gazebo Ignition)
- All converted packages
- All message packages

### Testing:
- Build all packages with colcon
- Run all tests
- Verify topic/service communication
- Test simulation scenarios

## Key Dependencies to Track:

### Message packages (rosidl_interface_packages):
- uuv_control_msgs
- uuv_gazebo_ros_plugins_msgs
- uuv_world_ros_plugins_msgs

### Simulation packages:
- uuv_gazebo_plugins
- uuv_gazebo_ros_plugins
- uuv_gazebo_worlds
- uuv_world_plugins
- uuv_world_ros_plugins

### Control packages:
- uuv_control_msgs
- uuv_trajectory_control
- uuv_thruster_manager
- uuv_control_cascaded_pids
- uuv_auv_control_allocator

### Tutorial packages:
- uuv_tutorial_seabed_world
- uuv_tutorial_dp_controller
- uuv_tutorial_disturbances
- uuv_tutorial_rov_model

## Notes:
- Keep version numbers consistent (0.6.13 for most packages)
- Maintain Apache-2.0 license across all packages
- Preserve maintainer and author information
- Update Python dependencies to python3-* format
- Ensure all ROS1-specific dependencies are replaced with ROS2 equivalents