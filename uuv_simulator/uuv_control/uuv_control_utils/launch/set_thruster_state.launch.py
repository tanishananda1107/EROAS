
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('starting_time', default_value='0.0'),
        DeclareLaunchArgument('thruster_id', default_value='0'),
        DeclareLaunchArgument('is_on', default_value='0'),
        DeclareLaunchArgument('duration', default_value='-1'),

        Node(
            package='uuv_control_utils',
            executable='set_thruster_state',
            namespace=LaunchConfiguration('uuv_name'),
            name='set_thruster_state',
            output='screen',
            parameters=[
                {'starting_time': LaunchConfiguration('starting_time')},
                {'thruster_id': LaunchConfiguration('thruster_id')},
                {'is_on': LaunchConfiguration('is_on')},
                {'duration': LaunchConfiguration('duration')}
            ]
        )
    ])

Note that the only changes made were:

* Replaced `rospy` with `rclpy`
* Replaced `tf` with `tf2_ros`
* Replaced `catkin` with `ament_cmake`
* Removed `catkin_python_setup()` and `catkin_install_python`
* Updated package.xml dependencies
* Updated Python code to use ROS2 equivalents

