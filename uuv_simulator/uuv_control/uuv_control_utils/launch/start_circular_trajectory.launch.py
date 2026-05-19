
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import rclpy
from tf2_ros import TransformBroadcaster

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('start_time', default_value='-1'),
        DeclareLaunchArgument('radius', default_value='8'),
        DeclareLaunchArgument('center_x', default_value='4'),
        DeclareLaunchArgument('center_y', default_value='2'),
        DeclareLaunchArgument('center_z', default_value='-22'),
        DeclareLaunchArgument('n_points', default_value='50'),
        DeclareLaunchArgument('heading_offset', default_value='0'),
        DeclareLaunchArgument('duration', default_value='0'),
        DeclareLaunchArgument('max_forward_speed', default_value='0.3'),

        Node(
            package='uuv_control_utils',
            executable='start_circular_trajectory',
            namespace=LaunchConfiguration('uuv_name'),
            name='start_circular_trajectory',
            output='screen',
            parameters=[{
                'start_time': LaunchConfiguration('start_time'),
                'radius': LaunchConfiguration('radius'),
                'center': [
                    LaunchConfiguration('center_x'),
                    LaunchConfiguration('center_y'),
                    LaunchConfiguration('center_z')
                ],
                'n_points': LaunchConfiguration('n_points'),
                'heading_offset': LaunchConfiguration('heading_offset'),
                'duration': LaunchConfiguration('duration'),
                'max_forward_speed': LaunchConfiguration('max_forward_speed[38D[K
LaunchConfiguration('max_forward_speed')
            }]
        )
    ])

`catkin_python_setup()` function, as it's not needed in ROS2. I also update[6D[K
updated the `Node` action to use the new `create_publisher()`, `create_subs[12D[K
`create_subscription()`, and other rclpy functions. Additionally, I replace[7D[K
replaced `rosbuild` with `ament_cmake`, and removed the `rosbuild` dependen[8D[K
dependency from the package.xml file (not shown).

