from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_descriptions'),
                'launch',
                'empty_underwater_world.launch.py'
            ])
        ),
        launch_arguments={
            'gui': 'true',
            'paused': 'true'
        }.items()
    )

    simulation_timer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_simulation_wrapper'),
                'launch',
                'set_simulation_timer.launch.py'
            ])
        ),
        launch_arguments={
            'timeout': '10'
        }.items()
    )

    unpause_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_simulation_wrapper'),
                'launch',
                'unpause_simulation.launch.py'
            ])
        ),
        launch_arguments={
            'timeout': '5'
        }.items()
    )

    return LaunchDescription([
        empty_world,
        simulation_timer,
        unpause_sim
    ])
