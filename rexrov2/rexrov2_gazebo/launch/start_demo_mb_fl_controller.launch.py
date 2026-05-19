from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    gazebo = FindPackageShare('rexrov2_gazebo').find('rexrov2_gazebo')
    desc = FindPackageShare('rexrov2_description').find('rexrov2_description')
    control = FindPackageShare('rexrov2_control').find('rexrov2_control')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'ocean_waves.launch.py'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(desc, 'launch', 'upload_rexrov2.launch.py'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(control, 'launch', 'start_mb_fl_controller.launch.py'))
        )
    ])
