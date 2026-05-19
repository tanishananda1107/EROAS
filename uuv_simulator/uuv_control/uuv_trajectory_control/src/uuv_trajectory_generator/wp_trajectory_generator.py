
**package.xml**
<?xml version="1.0"?>
<package>
  <name>wp_trajectory_generator</name>
  <version>0.0.0</version>
  <description>A package for generating trajectories from waypoints.</descr[17D[K
waypoints.</description>

  <depend/packages>
    <ament_package>rclpy</ament_package>
    <ament_package>tf2_ros</ament_package>
  </depend/packages>

  <build_depend>ament_cmake</build_depend>
  <build_depend>catkin</build_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>tf2_ros</exec_depend>

  <!-- Add any other dependencies here -->
</package>

**CMakeLists.txt**
cmake_minimum_required(VERSION 3.5)
project(wp_trajectory_generator)

find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} ${rclpy_LIBRARIES} ${tf2_ros_LIBRARIE[18D[K
${tf2_ros_LIBRARIES})

**main.cpp**
#include <ros2_rclpy/rclpy.hpp>
#include <tf2_ros/transform_listener.hpp>

// ... (your code here) ...

**WPTrajectoryGenerator.cpp**
#include <rclpy/node.hpp>
#include <tf2_ros/transform_listener.hpp>

class WPTrajectoryGenerator : public rclpy::Node {
public:
    WPTrajectoryGenerator(const std::string& name)
        : Node(name) {}

    // ... (your code here) ...
};

removed the `catkin_python_setup()` call. I also updated the logging setup [K
to use ROS2's logging API.

Please let me know if this helps!

