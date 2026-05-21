# ROS2 Migration Plan for uuv_tutorials

## Packages to Migrate

1. **uuv_tutorial_disturbances** - Launch files, config, CMakeLists
2. **uuv_tutorial_dp_controller** - Launch files, Python script, CMakeLists
3. **uuv_tutorial_rov_model** - Launch files, XACRO files, CMakeLists
4. **uuv_tutorial_seabed_world** - Launch files, world files, SDF models, CMakeLists
5. **uuv_tutorials** - Main package, CMakeLists
6. **uuv_tutorials_description** - Description package
7. **uuv_tutorials_worlds** - World files
8. **uuv_tutorials_navigation** - Navigation setup
9. **uuv_tutorials_terrains** - Terrain handling
10. **uuv_tutorials_buoyancy** - Buoyancy plugins
11. **uuv_tutorial_disturbances** - Disturbance simulation
12. **uuv_tutorial_rov_model** - ROV model files
13. **uuv_tutorials_python** - Python tutorial package (empty shell)
14. **uuv_tutorials_control** - Control tutorials package (empty shell)
15. **uuv_tutorials_auv_teleop** - Empty shell, needs setup.py
16. **uuv_tutorials_auv_gazebo** - Empty shell, needs setup.py

## File Types to Convert

### package.xml
- Format: 3
- Replace `ros` with `rclpy`, `rospy`
- Use `exec_depend` instead of `depend`
- Add `member_of_group>rosidl_interface_packages</member_of_group>` for custom messages
- Add `<export><build_type>ament_cmake</build_type></export>`

### CMakeLists.txt
- Replace `catkin_package` with `ament_python`
- Add `ament_package()` at end
- Remove all catkin references

### Launch Files (.launch -> .launch.py)
- Use `LaunchDescription`
- Use `launch_ros.actions.Node`
- Use `PythonLaunchDescriptionSource` for includes
- Use `get_package_share_directory` for `find pkg`
- Use `LaunchConfiguration` for `arg`

### Python Files
- Wrap in `rclpy.node.Node` class
- Replace `rospy.init_node` with `super().__init__()`
- Use `create_publisher`, `create_subscription`
- Replace `rospy.spin()` with `rclpy.spin(node)`
- Replace `rospy.loginfo()` with `self.get_logger().info()`

### World/SDF Files
- Update to Gazebo Harmonic standards
- SDF version 1.10
- Update plugin names for Gazebo Harmonic

## Key Patterns

- `rospy.Publisher` -> `self.create_publisher()`
- `rospy.Subscriber` -> `self.create_subscription()`
- `rospy.init_node()` -> `rclpy.init()`, `super().__init__()`
- `rospy.spin()` -> `rclpy.spin(node)`
- `rospy.loginfo()` -> `self.get_logger().info()`
- `rospy.logwarn()` -> `self.get_logger().warn()`
- `rospy.logerr()` -> `self.get_logger().error()`
- `rospy.Time.now().to_sec()` -> `rclpy.time.Time.now().nanoseconds / 1e9`

## Launch File Patterns

- `find_package` -> `get_package_share_directory()`
- `arg` -> `LaunchConfiguration`
- `include` -> `IncludeLaunchDescription`
- `ns` -> `namespace` argument in Node
- `param` -> `parameters=[{...}]`

## World File Patterns

- Update to SDF 1.10
- Update plugin names for Gazebo Harmonic
- Replace `plugin name="gazebo_ros_force"` with new Gazebo Harmonic equivalents

## Special Cases

### uuv_tutorials_python and uuv_tutorials_control
- Both packages are empty shells with only package.xml
- package.xml already in ROS2 format (format="3")
- Need to create setup.py for both
- Need to create resource/ directory with package index file
- No CMakeLists.txt needed for pure Python packages without executables

### uuv_tutorials_auv_teleop and uuv_tutorials_auv_gazebo
- Both packages are empty shells with only package.xml
- package.xml already in ROS2 format (format="3")
- Need to create setup.py for both
- Need to create resource/ directory with package index file
- No CMakeLists.txt needed for pure Python packages without executables

### setup.py Template
```python
from setuptools import setup

package_name = '<package_name>'

setup(
    name=package_name,
    version='0.0.0',
    packages=['<package_name>'],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='AIRLab IISc',
    maintainer_email='airlab@iisc.ac.in',
    description='Description here',
    license='Apache-2.0',
)
```

### resource Directory Template
```
uuv_tutorials_python
```
(Just a file with the package name, no content needed)

## Build Verification
After creating files, run:
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Notes
- Both packages are essentially empty shells in the ROS1 state
- The package.xml files are already in ROS2 format (format="3")
- These packages appear to be metapackages or placeholder packages
- No CMakeLists.txt needed for pure Python packages without executables
