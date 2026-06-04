import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('rexrov2_gazebo')

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='world_a'),
        DeclareLaunchArgument('teleop_on', default_value='true'),
        DeclareLaunchArgument('teleop_prefix', default_value=''),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x', default_value='auto'),
        DeclareLaunchArgument('y', default_value='auto'),
        DeclareLaunchArgument('z', default_value='auto'),
        DeclareLaunchArgument('yaw', default_value='auto'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'start_EROAS_demo.launch.py')),
            launch_arguments={
                'world_name': LaunchConfiguration('world_name'),
                'gui': LaunchConfiguration('gui'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'yaw': LaunchConfiguration('yaw'),
                'start_navigator': 'false',
                'start_cbf': 'true',
                'start_sonar_reconstruction': 'true',
            }.items(),
        ),

        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='rexrov2_teleop',
            prefix=LaunchConfiguration('teleop_prefix'),
            remappings=[('/cmd_vel', '/rexrov2/cmd_vel_1')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('teleop_on')),
        ),
    ])
