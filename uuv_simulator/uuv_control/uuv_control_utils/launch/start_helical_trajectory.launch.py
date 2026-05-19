
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rclpy.node import Node as RCLNode

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument('uuv_name', default_value='rexrov2'),
        DeclareLaunchArgument('start_time', default_value='-1'),
        DeclareLaunchArgument('radius', default_value='8'),
        DeclareLaunchArgument('center_x', default_value='0'),
        DeclareLaunchArgument('center_y', default_value='0'),
        DeclareLaunchArgument('center_z', default_value='-30'),
        DeclareLaunchArgument('n_points', default_value='50'),
        DeclareLaunchArgument('n_turns', default_value='1'),
        DeclareLaunchArgument('delta_z', default_value='5.0'),
        DeclareLaunchArgument('heading_offset', default_value='0'),
        DeclareLaunchArgument('duration', default_value='150'),
        DeclareLaunchArgument('max_forward_speed', default_value='0.3'),

        Node(
            package='uuv_control_utils',
            executable='start_helical_trajectory',
            namespace=LaunchConfiguration('uuv_name'),
            name='start_helical_trajectory',
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
                'max_forward_speed': LaunchConfiguration('max_forward_speed[38D[K
LaunchConfiguration('max_forward_speed'),
                'duration': LaunchConfiguration('duration'),
                'n_turns': LaunchConfiguration('n_turns'),
                'delta_z': LaunchConfiguration('delta_z')
            }]
        )
    ])

Note that I removed the `catkin_python_setup()` and replaced it with nothin[6D[K
nothing, as it's not necessary in ROS2. I also replaced `rosbuild` with not[3D[K
nothing, as it's deprecated in ROS2.

