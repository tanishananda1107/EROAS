from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    gazebo = FindPackageShare('rexrov2_gazebo').find('rexrov2_gazebo')
    desc = FindPackageShare('rexrov2_description').find('rexrov2_description')
    sim = FindPackageShare('uuv_simulation_wrapper').find('uuv_simulation_wrapper')

    return LaunchDescription([

        DeclareLaunchArgument('x', default_value='25'),
        DeclareLaunchArgument('y', default_value='-35'),
        DeclareLaunchArgument('z', default_value='-57'),
        DeclareLaunchArgument('yaw', default_value='2'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'ocean_waves.launch.py'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim, 'launch', 'unpause_simulation.launch.py'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(desc, 'launch', 'upload_rexrov2.launch.py'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'start_pid_controller.launch.py'))
        )
    ])
