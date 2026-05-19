from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument('model_name', default_value='rexrov2'),
        DeclareLaunchArgument('uuv_name', default_value='rexrov2'),
        DeclareLaunchArgument('reset_tam', default_value='false'),
    ])
