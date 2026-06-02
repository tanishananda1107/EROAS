from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    gazebo = FindPackageShare('rexrov2_gazebo').find('rexrov2_gazebo')
    worlds = FindPackageShare('uuv_gazebo_worlds').find('uuv_gazebo_worlds')
    desc = FindPackageShare('rexrov2_description').find('rexrov2_description')
    control = FindPackageShare('rexrov2_control').find('rexrov2_control')

    return LaunchDescription([

        DeclareLaunchArgument('x', default_value='25'),
        DeclareLaunchArgument('y', default_value='-35'),
        DeclareLaunchArgument('z', default_value='-47'),
        DeclareLaunchArgument('yaw', default_value='2.27'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(worlds, 'launch', 'ocean_waves.launch.py'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(desc, 'launch', 'upload_rexrov2.launch.py'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(control, 'launch', 'start_pid_controller.launch.py'))
        )
    ])
