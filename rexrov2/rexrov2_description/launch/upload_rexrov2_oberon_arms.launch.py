import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_desc = get_package_share_directory('rexrov2_description')

    namespace = LaunchConfiguration('namespace')

    robot_desc = Command([
        'xacro ',
        os.path.join(pkg_desc, 'robots', 'rexrov2_oberon_arms.xacro'),
        ' namespace:=', namespace,
    ])

    return LaunchDescription([

        DeclareLaunchArgument('namespace', default_value='rexrov2'),

        GroupAction([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=namespace,
                parameters=[{'robot_description': robot_desc}]
            ),

            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', namespace, '-topic', 'robot_description'],
                output='screen'
            ),
        ]),
    ])
