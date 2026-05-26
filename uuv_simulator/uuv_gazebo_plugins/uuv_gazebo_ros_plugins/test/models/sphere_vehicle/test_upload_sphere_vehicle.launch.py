from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    pkg_share = FindPackageShare('uuv_gazebo_ros_plugins').find('uuv_gazebo_ros_plugins')

    xacro_file = os.path.join(
        pkg_share,
        'test/models/sphere_vehicle/model.xacro'
    )

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        # Equivalent of <param robot_description>
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='vehicle',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        # Gazebo Harmonic entity spawn (replacement for spawn_model)
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'vehicle',
                '-x', '0',
                '-y', '0',
                '-z', '0',
                '-R', '0',
                '-P', '0',
                '-Y', '0',
                '-topic', 'robot_description'
            ],
            output='screen'
        )
    ])
