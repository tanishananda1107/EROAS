# ROS1 to ROS2 Jazzy Migration Plan for EROAS Package

## Overview

This document outlines the comprehensive migration plan for converting the EROAS ROS1 codebase to ROS2 Jazzy on Ubuntu 24.04. The EROAS package contains navigation, control, and simulation components for autonomous underwater vehicles (AUVs).

## Current State Analysis

### ROS1 Components Identified:
1. **nps_uw_multibeam_sonar** - Sonar plugin with CUDA library
2. **navigator_auv** - Navigation package for AUV
3. **uuv_simulator** - Complete UUV simulation framework (metapackage)
4. **uuv_control** - Control packages (cascaded PIDs, trajectory control, thruster manager)
5. **uuv_sensor_plugins** - Sensor plugins for Gazebo
6. **uuv_world_plugins** - World/environment plugins
7. **uuv_gazebo_plugins** - Gazebo plugins
8. **uuv_descriptions** - Vehicle descriptions and URDF models

### Key Migration Challenges:

| Category | Issues Found | Priority |
|----------|-------------|----------|
| **Package Configuration** | ROS1 package.xml format 2, catkin buildtool | HIGH |
| **Build System** | catkin vs ament_cmake | HIGH |
| **Dependencies** | ROS1 packages (rospy, tf, sensor_msgs v1) | HIGH |
| **TF Migration** | tf2 vs legacy tf, coordinate frames | MEDIUM |
| **Message Types** | ROS1 messages vs ROS2 messages | HIGH |
| **Gazebo Integration** | gazebo_ros vs ros_gz_bridge | MEDIUM |

## Migration Phases

### Phase 1: Package Configuration Migration (Week 1)

#### 1.1 Update package.xml Files

**Target Format**: ROS2 package.xml format 3

**Required Changes**:
```xml
<!-- ROS1 -->
<package format="2">
  <buildtool_depend>catkin</buildtool_depend>
  <depend>rospy</depend>
  <depend>tf</depend>
</package>

<!-- ROS2 -->
<package format="3">
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclpy</depend>
  <depend>tf2_ros</depend>
</package>
```

**Packages to Update**:
- nps_uw_multibeam_sonar/package.xml
- navigator_auv/package.xml
- All uuv_simulator subpackages

#### 1.2 Update CMakeLists.txt Files

**Required Changes**:
```cmake
# ROS1
cmake_minimum_required(VERSION 3.0.2)
project(navigator_auv)
find_package(catkin REQUIRED COMPONENTS ...)
catkin_package(...)
catkin_install_python(...)

# ROS2
cmake_minimum_required(VERSION 3.8)
project(navigator_auv)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_export_dependencies(...)
ament_package()
```

**Action Items**:
- Replace `catkin_find_package` with `ament_find_package`
- Update `catkin_package()` to `ament_package()`
- Update install rules for ROS2 structure

### Phase 2: Message and Service Migration (Week 2)

#### 2.1 Message Type Conversions

| ROS1 Type | ROS2 Type | Notes |
|-----------|-----------|-------|
| `rospy.Time.now()` | `rclpy.clock.Clock().now()` | Time handling |
| `tf.TransformListener` | `tf2_ros.Buffer` + `TransformBroadcaster` | TF2 migration |
| `sensor_msgs/LaserScan` | `sensor_msgs/msg/LaserScan` | New syntax |
| `geometry_msgs/PoseStamped` | `geometry_msgs/msg/PoseStamped` | New syntax |

#### 2.2 Message Definition Files

**Action Items**:
- Convert `.msg` files (no changes needed for syntax)
- Update `.srv` files (no changes needed for syntax)
- Ensure `rosidl_default_generators` in package.xml
- Update CMakeLists.txt for message generation

### Phase 3: Node Migration (Week 3-4)

#### 3.1 Python Node Migration

**Pattern 1: Simple Node**
```python
# ROS1
import rospy
from std_msgs.msg import String

class MyNode:
    def __init__(self):
        rospy.init_node('my_node')
        self.pub = rospy.Publisher('topic', String, queue_size=10)
        self.sub = rospy.Subscriber('topic', String, callback)
        self.rate = rospy.Rate(10)

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.pub = self.create_publisher(String, 'topic', 10)
        self.sub = self.create_subscription(String, 'topic', callback, 10)
        self.timer = self.create_timer(0.1, timer_callback)
```

**Pattern 2: Service Client/Server**
```python
# ROS1
self.service = rospy.Service('service_name', MyService, handle_service)

# ROS2
self.service = self.create_service(MyService, 'service_name', handle_service)
```

#### 3.2 C++ Node Migration

**Pattern**:
```cpp
// ROS1
ros::NodeHandle nh;
ros::Publisher pub = nh.advertise<std_msgs::String>("topic", 10);
ros::Subscriber sub = nh.subscribe("topic", 10, callback);

// ROS2
auto node = rclcpp::Node::make_shared("my_node");
auto pub = node->create_publisher<std_msgs::msg::String>("topic", 10);
auto sub = node->create_subscription<std_msgs::msg::String>("topic", 10, callback);
```

### Phase 4: TF Migration (Week 5)

#### 4.1 TF1 to TF2 Migration

**Key Changes**:
```python
# ROS1 TF
import tf
listener = tf.TransformListener()
listener.waitForTransform('/base_link', '/sensor', rospy.Time(0))
trans, rot = listener.lookupTransform('/base_link', '/sensor', rospy.Time(0))

# ROS2 TF2
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import pose_from_tf2

buffer = Buffer()
listener = TransformListener(buffer, node)
try:
    trans = buffer.lookup_transform('/base_link', '/sensor', rclpy.time.Time())
except tf2_ros.LookupException as e:
    pass
```

#### 4.2 Coordinate Frame Updates

**Required Actions**:
- Update all frame_id references
- Ensure consistent naming conventions
- Verify transform tree connectivity
- Update URDF coordinate frames if needed

### Phase 5: Gazebo Integration (Week 6)

#### 5.1 gazebo_ros to ros_gz_bridge

**Migration Pattern**:
```xml
<!-- ROS1 Gazebo -->
<plugin filename="libgazebo_ros_underwater_object.so" name="underwater_object">
  <robotNamespace>/uuv</robotNamespace>
</plugin>

<!-- ROS2 Gazebo (Harmonic) -->
<plugin filename="libuuv_gazebo_ros_plugins.so" name="underwater_object">
  <ros>
    <namespace>/uuv</namespace>
  </ros>
</plugin>
```

#### 5.2 Plugin Updates

**Packages Affected**:
- uuv_sensor_plugins
- uuv_world_plugins
- uuv_gazebo_plugins

**Required Actions**:
- Update plugin registration for Gazebo Harmonic
- Migrate to gz-sim8, gz-transport12, gz-msgs9
- Update message bridges

### Phase 6: Testing and Validation (Week 7-8)

#### 6.1 Unit Testing

**Migration Strategy**:
```bash
# ROS1
catkin_make run_tests

# ROS2
colcon test
colcon test-result
```

#### 6.2 Integration Testing

**Test Scenarios**:
1. Navigation stack functionality
2. Control loop performance
3. Sensor data flow
4. Service/client communication
5. TF tree completeness

### Phase 7: Documentation and Deployment (Week 9)

#### 7.1 Documentation Updates

**Required Documentation**:
- Updated README.md with ROS2 setup instructions
- Migration guide for developers
- API documentation updates
- Troubleshooting guide

#### 7.2 Deployment

**Steps**:
1. Create ROS2 workspace on Ubuntu 24.04
2. Install ROS2 Jazzy dependencies
3. Clone and build migrated packages
4. Run integration tests
5. Deploy to target systems

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| TF coordinate frame issues | Medium | High | Extensive testing with known transforms |
| Gazebo plugin compatibility | Medium | Medium | Test in isolated simulation environment |
| Message type mismatches | Low | High | Automated message validation scripts |
| Performance degradation | Low | Medium | Benchmark before/after migration |

## Dependencies Checklist

### ROS2 Jazzy Core Dependencies:
- [ ] rclpy (Python client library)
- [ ] rclcpp (C++ client library)
- [ ] tf2_ros (TF2 runtime)
- [ ] tf2_geometry_msgs
- [ ] geometry_msgs
- [ ] sensor_msgs
- [ ] std_msgs
- [ ] nav_msgs
- [ ] visualization_msgs

### Simulation Dependencies:
- [ ] ros_gz_sim
- [ ] ros_gz_bridge
- [ ] ros_gz_interfaces
- [ ] gz-sim8
- [ ] gz-transport12
- [ ] gz-msgs9

### Control Dependencies:
- [ ] python3-numpy
- [ ] python3-scipy
- [ ] python3-yaml
- [ ] python3-matplotlib

## Timeline Summary

| Phase | Duration | Deliverables |
|-------|----------|--------------|
| Phase 1 | Week 1 | Updated package.xml and CMakeLists.txt |
| Phase 2 | Week 2 | Message/service definitions migrated |
| Phase 3 | Weeks 3-4 | All nodes migrated to ROS2 APIs |
| Phase 4 | Week 5 | TF2 migration complete |
| Phase 5 | Week 6 | Gazebo integration updated |
| Phase 6 | Weeks 7-8 | Testing complete |
| Phase 7 | Week 9 | Documentation and deployment |

**Total Estimated Time**: 9 weeks

## Next Steps

1. **Immediate**: Set up ROS2 Jazzy development environment on Ubuntu 24.04
2. **Week 1**: Begin package.xml and CMakeLists.txt migration
3. **Ongoing**: Document all API changes and migration patterns
4. **Parallel**: Update CI/CD pipelines for ROS2 testing

## Notes

- This migration assumes Ubuntu 24.04 as the target platform
- ROS2 Jazzy is the target ROS2 distribution
- Gazebo Harmonic will be used for simulation
- Backward compatibility with ROS1 is not required
- All changes should be tested in a controlled environment before deployment
