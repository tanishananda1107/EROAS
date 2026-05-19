from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    uuv_name = LaunchConfiguration('uuv_name')

    return LaunchDescription([

        DeclareLaunchArgument('uuv_name', default_value='rexrov2'),
        DeclareLaunchArgument('joy_id', default_value='0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rexrov2_control'),
                    'launch',
                    'start_thruster_manager.launch.py'
                ])
            ])
        ),

        GroupAction([
            Node(
                package='uuv_control_cascaded_pid',
                executable='AccelerationControl.py',
                namespace=uuv_name,
                output='screen'
            ),
            Node(
                package='uuv_control_cascaded_pid',
                executable='VelocityControl.py',
                namespace=uuv_name,
                output='screen'
            )
        ])
    ])
