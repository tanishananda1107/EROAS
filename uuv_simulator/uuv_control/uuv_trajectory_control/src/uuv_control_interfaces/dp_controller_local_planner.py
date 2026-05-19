Jazzy:

**1. Update the package name and version**

In the `package.xml` file, update the package name and version to match you[3D[K
your desired naming convention.

For example:
<package>
  <name>dp_controller_local_planner</name>
  <version>0.10.0</version>
  ...
</package>
**2. Update the CMakeLists.txt**

In the `CMakeLists.txt` file, update the package name and version to match [K
your desired naming convention.

For example:
cmake_minimum_required(VERSION 3.5)
project(dp_controller_local_planner)

...

add_executable(${PROJECT_NAME} main.cpp)
**3. Update the C++ code**

to ROS2 Jazzy.

For example:
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

// ...

void dpControllerLocalPlanner::process(ros::Time time) {
  // ...
}

int main(int argc, char** argv) {
  ros::init(argc, argv);
  dpControllerLocalPlanner node;
  // ...
}
becomes:
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/TwistStamped.hpp>

// ...

void DPControllerLocalPlanner::process(rclcpp::Time time) {
  // ...
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  DPControllerLocalPlanner node;
  // ...
}
**4. Update the ROS2 message types**

types to ROS2 Jazzy message types.

For example:
#include <ros/geometry_msgs/TwistStamped.h>

// ...

void dpControllerLocalPlanner::process(ros::Time time) {
  geometry_msgs::TwistStamped msg;
  // ...
}
becomes:
#include <geometry_msgs/msg/TwistStamped.hpp>

// ...

void DPControllerLocalPlanner::process(rclcpp::Time time) {
  geometry_msgs::msg::TwistStamped msg;
  // ...
}
**5. Update the ROS2 node**

ROS2 Jazzy node.

For example:
int main(int argc, char** argv) {
  ros::init(argc, argv);
  dpControllerLocalPlanner node;
  // ...
}
becomes:
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  DPControllerLocalPlanner node;
  // ...
}
**6. Update the ROS2 publisher and subscriber**

and subscriber to ROS2 Jazzy publisher and subscriber.

For example:
ros::Publisher pub = node.advertise<geometry_msgs::TwistStamped>("command",[54D[K
node.advertise<geometry_msgs::TwistStamped>("command", 10);
ros::Subscriber sub = node.subscribe<geometry_msgs::TwistStamped>("status",[53D[K
node.subscribe<geometry_msgs::TwistStamped>("status", 10);

// ...
becomes:
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub =
  this->create_publisher<geometry_msgs::msg::TwistStamped>("command", 10);
rclcpp::Subscriber<geometry_msgs::msg::TwistStamped>::SharedPtr sub =
  this->create_subscription<geometry_msgs::msg::TwistStamped>("status", 10)[3D[K
10);

// ...
**7. Update the ROS2 callback function**

function to ROS2 Jazzy callback function.

For example:
void dpControllerLocalPlanner::callback(const geometry_msgs::TwistStamped::[29D[K
geometry_msgs::TwistStamped::ConstPtr& msg) {
  // ...
}

// ...
ros::subscribe("status", 10, &dpControllerLocalPlanner::callback);
becomes:
void DPControllerLocalPlanner::on_status_message(const geometry_msgs::msg::[20D[K
geometry_msgs::msg::TwistStamped::SharedPtr& msg) {
  // ...
}

// ...
this->create_subscription<geometry_msgs::msg::TwistStamped>("status", 10)
  ->registerCallback(&DPControllerLocalPlanner::on_status_message);
**8. Compile and run the code**

Compile the updated `dp_controller_local_planner.cpp` file using the ROS2 J[1D[K
Jazzy compiler (e.g., `ros2 compile`) and run the resulting executable (e.g[4D[K
(e.g., `./dp_controller_local_planner`).

Jazzy package.

