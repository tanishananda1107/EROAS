from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --------------------------------------------------------------------------
    # Launch arguments
    # --------------------------------------------------------------------------

    gui = LaunchConfiguration('gui')
    paused = LaunchConfiguration('paused')

    # --------------------------------------------------------------------------
    # World file
    # --------------------------------------------------------------------------

    world_file = PathJoinSubstitution([
        FindPackageShare('uuv_tutorial_seabed_world'),
        'worlds',
        'example_underwater.sdf'
    ])

    # --------------------------------------------------------------------------
    # Gazebo Harmonic / GZ Sim 8 launch
    # --------------------------------------------------------------------------

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [
                '-r ',
                world_file
            ]
        }.items()
    )

    # --------------------------------------------------------------------------
    # Launch description
    # --------------------------------------------------------------------------

    return LaunchDescription([

        DeclareLaunchArgument(
            'gui',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'paused',
            default_value='false'
        ),

        gz_sim_launch
    ])
