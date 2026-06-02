from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rexrov2_config_dir = PathJoinSubstitution([
        FindPackageShare('rexrov2_control'),
        'config',
    ])

    return LaunchDescription([

        DeclareLaunchArgument('uuv_name', default_value='rexrov2'),
        DeclareLaunchArgument('model_name', default_value='rexrov2'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('uuv_trajectory_control'),
                    'launch',
                    'rov_pid_controller.launch.py'
                ])
            ]),
            launch_arguments={
                'uuv_name': LaunchConfiguration('uuv_name'),
                'model_name': LaunchConfiguration('model_name'),
                'output_dir': rexrov2_config_dir,
                'config_file': PathJoinSubstitution([
                    rexrov2_config_dir,
                    'thruster_manager.yaml',
                ]),
                'tam_file': PathJoinSubstitution([
                    rexrov2_config_dir,
                    'TAM.yaml',
                ]),
            }.items()
        )
    ])
