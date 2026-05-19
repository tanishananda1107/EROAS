from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    gazebo = FindPackageShare('rexrov2_gazebo').find('rexrov2_gazebo')
    desc = FindPackageShare('rexrov2_description').find('rexrov2_description')
    control = FindPackageShare('rexrov2_control').find('rexrov2_control')

    return LaunchDescription([

        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='-25'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'empty_underwater_world.launch.py'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(desc, 'launch', 'upload_rexrov2.launch.py'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(control, 'launch', 'start_nmb_sm_controller.launch.py'))
        )
    ])
