from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument('uuv_name', default_value='rexrov2'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('uuv_trajectory_control'),
                    'launch',
                    'rov_pid_controller.launch.py'
                ])
            ]),
            launch_arguments={'uuv_name': LaunchConfiguration('uuv_name')}.items()
        )
    ])
