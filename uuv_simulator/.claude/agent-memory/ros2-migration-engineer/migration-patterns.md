---
name: migration-patterns
description: ROS1 to ROS2 migration patterns and conventions for the EROAS uuv_simulator project
metadata:
  type: reference
---

# ROS1 to ROS2 Migration Patterns

## System Plugin Pattern (Gazebo Harmonic)

### Plugin Registration
**ROS1 (Gazebo 11):** `GZ_REGISTER_WORLD_PLUGIN(UnderwaterCurrentPlugin)`
**ROS2 (Gazebo Harmonic):** `GZ_ADD_PLUGIN(UnderwaterCurrentPlugin, ignition::gazebo::System, UnderwaterCurrentPlugin::ISystemConfigure, UnderwaterCurrentPlugin::ISystemUpdate)`

The system plugin must inherit from `ignition::gazebo::System` and implement the required interface methods:
- `ISystemConfigure` - Called during Gazebo initialization
- `ISystemUpdate` - Called each simulation step

### Plugin Class Structure
```cpp
class UnderwaterCurrentPlugin :
  public ignition::gazebo::SystemPlugin,
  public ignition::gazebo::SystemConfigure,
  public ignition::gazebo::SystemUpdate
{
public:
  UnderwaterCurrentPlugin();
  ~UnderwaterCurrentPlugin();

  void OnConfigure(const ignition::gazebo::ConfigureInfo &info) override;
  void OnUpdate(const ignition::gazebo::UpdateInfo &info) override;

private:
  // Member variables
};
```

### Namespace Handling
ROS2 plugins use `this->GetSystemName()` for unique naming within a world:
```cpp
std::string ns = this->GetSystemName();
```

## ROS2 API Mappings

### Headers
| ROS1 | ROS2 |
|------|------|
| `#include <ros/ros.h>` | `#include <rclcpp/rclcpp.hpp>` |
| `#include <std_msgs/String.h>` | `#include <std_msgs/msg/string.hpp>` |
| `#include <geometry_msgs/Twist.h>` | `#include <geometry_msgs/msg/twist.hpp>` |
| `#include <sensor_msgs/JointState.h>` | `#include <sensor_msgs/msg/joint_state.hpp>` |
| `#include <nav_msgs/Odometry.h>` | `#include <nav_msgs/msg/odometry.hpp>` |
| `#include <tf/transform_broadcaster.h>` | `#include <tf2_ros/transform_broadcaster.h>` |
| `#include <boost/shared_ptr.hpp>` | `#include <memory>` |

### Message Types
**ROS1:** `geometry_msgs::Twist`
**ROS2:** `geometry_msgs::msg::Twist`

**ROS1:** `geometry_msgs::TwistStamped`
**ROS2:** `geometry_msgs::msg::TwistStamped`

**ROS1:** `nav_msgs::Odometry`
**ROS2:** `nav_msgs::msg::Odometry`

### Publishers
**ROS1:**
```cpp
ros::Publisher pub = node->advertise<geometry_msgs::Twist>("topic", 10);
pub.publish(msg);
```

**ROS2:**
```cpp
auto pub = node->create_publisher<geometry_msgs::msg::Twist>("topic", 10);
pub->publish(std::make_shared<geometry_msgs::msg::Twist>());
```

### Subscribers
**ROS1:**
```cpp
ros::Subscriber sub = node->subscribe("topic", 10, &ClassName::callback, this);
```

**ROS2:**
```cpp
auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
  "topic", 10,
  std::bind(&ClassName::callback, this, std::placeholders::_1));
```

### Services
**ROS1:**
```cpp
ros::ServiceServer srv = node->advertiseService("service_name",
  &ClassName::ServiceCallback, this);
```

**ROS2:**
```cpp
auto srv = rclcpp::create_service<my_msgs::srv::MyService>(
  node, "service_name",
  std::bind(&ClassName::ServiceCallback, this,
    std::placeholders::_1, std::placeholders::_2));
```

### Service Messages
**ROS1:** `uuv_world_ros_plugins_msgs/SetCurrentModel.h`
**ROS2:** `uuv_world_ros_plugins_msgs/srv/SetCurrentModel`

Request/Response access:
- **ROS1:** `_req.velocity`
- **ROS2:** `_req->velocity` (use arrow operator for shared_ptr)

### Time Handling
**ROS1:** `ros::Time::now()`
**ROS2:** `rclcpp::Clock().now()`

**ROS1:** `msg.header.stamp = ros::Time::now()`
**ROS2:** `msg->header.stamp = rclcpp::Clock().now()`

### Node Initialization
**ROS1:** `ros::NodeHandle node(ns);`
**ROS2:** `rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("plugin_name", ns);`

### Shared Pointers
**ROS1:** `boost::shared_ptr<T>`
**ROS2:** `std::shared_ptr<T>`

## Custom Service Messages

Service messages must be defined in a separate package with `.srv` files:

**SetCurrentModel.srv:**
```
float64 mean
float64 min
float64 max
float64 mu
float64 noise
---
bool success
```

**CurrentModel.srv:**
```
float64 mean
float64 min
float64 max
float64 mu
float64 noise
---
bool success
```

**SetCurrentVelocity.srv:**
```
float64 velocity
float64 horizontal_angle
float64 vertical_angle
---
bool success
```

**SetCurrentDirection.srv:**
```
float64 angle
---
bool success
```

**GetCurrentModel.srv:**
```
---
float64 mean
float64 min
float64 max
float64 mu
float64 noise
---
bool success
```

## Build System Changes

### CMakeLists.txt
```cmake
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uuv_world_ros_plugins_msgs REQUIRED)

ament_target_dependencies(your_target
  rclcpp
  geometry_msgs
  uuv_world_ros_plugins_msgs
)
```

### package.xml
```xml
<build_depend>rclcpp</build_depend>
<exec_depend>rclcpp</exec_depend>
<build_depend>geometry_msgs</build_depend>
<exec_depend>geometry_msgs</exec_depend>
<build_depend>uuv_world_ros_plugins_msgs</build_depend>
<exec_depend>uuv_world_ros_plugins_msgs</exec_depend>
```

## Common Patterns

### Message Conversion
**ROS1:** `msgs::Set(&msg, value)`
**ROS2:** Direct assignment: `msg->linear.x = value;`

### Ignition Math vs Eigen
**ROS1:** `ignition::math::Vector3d`
**ROS2:** `ignition::math::Vector3d` (no change in Gazebo Harmonic)

### World Time Access
**ROS1:** `world->GetSimTime()`
**ROS2:** `world->SimTime()`

### Event Connections
**ROS1:** `gazebo::event::Events::ConnectWorldUpdateBegin(callback)`
**ROS2:** Same, but prefer system plugin callbacks (OnConfigure, OnUpdate)

## Known Issues and Workarounds

1. **Service Callback Signatures:** ROS2 services require `SharedPtr` for both request and response, and use arrow operator for member access
2. **Plugin Name:** Use `this->GetSystemName()` instead of hardcoding plugin names
3. **Namespace:** ROS2 plugins may have different namespace handling; verify with `this->GetSystemName()`
4. **Time:** Use `rclcpp::Clock().now()` instead of `ros::Time::now()`
5. **Shared Pointers:** Use `std::make_shared<T>()` and arrow operators for service callbacks
