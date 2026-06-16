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

        DeclareLaunchArgument('model_name', default_value='rexrov2'),
        DeclareLaunchArgument('uuv_name', default_value='rexrov2'),
        DeclareLaunchArgument('reset_tam', default_value='false'),
        DeclareLaunchArgument('base_link', default_value='base_link'),
        DeclareLaunchArgument('timeout', default_value='-1'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('uuv_thruster_manager'),
                    'launch',
                    'thruster_manager.launch.py',
                ])
            ]),
            launch_arguments={
                'model_name': LaunchConfiguration('model_name'),
                'uuv_name': LaunchConfiguration('uuv_name'),
                'base_link': LaunchConfiguration('base_link'),
                'timeout': LaunchConfiguration('timeout'),
                'reset_tam': LaunchConfiguration('reset_tam'),
                'output_dir': rexrov2_config_dir,
                'config_file': PathJoinSubstitution([
                    rexrov2_config_dir,
                    'thruster_manager.yaml',
                ]),
                'tam_file': PathJoinSubstitution([
                    rexrov2_config_dir,
                    'TAM.yaml',
                ]),
            }.items(),
        ),
    ])
