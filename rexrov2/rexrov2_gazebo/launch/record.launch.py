from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag_filename', default_value='recording.db3'),

        Node(
            package='rosbag2_transport',
            executable='record',
            name='rosbag_record',
            output='screen',
            arguments=[
                '-o', LaunchConfiguration('bag_filename'),
                '/rexrov2/pose_gt',
                '/rexrov2/thruster_manager/input',
                '/hydrodynamics/current_velocity'
            ],
            condition=IfCondition(LaunchConfiguration('record'))
        )
    ])
