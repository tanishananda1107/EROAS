
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, DeclareLaunchDependency
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('force_x', default_value='0.0'),
        DeclareLaunchArgument('force_y', default_value='0.0'),
        DeclareLaunchArgument('force_z', default_value='0.0'),
        DeclareLaunchArgument('torque_x', default_value='0.0'),
        DeclareLaunchArgument('torque_y', default_value='0.0'),
        DeclareLaunchArgument('torque_z', default_value='0.0'),
        DeclareLaunchArgument('starting_time', default_value='0.0'),
        DeclareLaunchArgument('duration', default_value='1.0'),

        DeclareLaunchDependency(package='uuv_control_utils'),

        Node(
            package='uuv_control_utils',
            executable='apply_body_wrench',
            namespace=LaunchConfiguration('uuv_name'),
            name='apply_body_wrench',
            output='screen',
            parameters=[{
                'starting_time': LaunchConfiguration('starting_time'),
                'force': [
                    LaunchConfiguration('force_x').value,
                    LaunchConfiguration('force_y').value,
                    LaunchConfiguration('force_z').value
                ],
                'torque': [
                    LaunchConfiguration('torque_x').value,
                    LaunchConfiguration('torque_y').value,
                    LaunchConfiguration('torque_z').value
                ],
                'duration': LaunchConfiguration('duration').value
            }]
        )
    ])

