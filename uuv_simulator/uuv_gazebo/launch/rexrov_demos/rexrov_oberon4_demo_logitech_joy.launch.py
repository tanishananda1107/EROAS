from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    joy_id = LaunchConfiguration('joy_id')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    demo = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'launch/rexrov_demos',
        'rexrov_oberon4_demo.launch.py'
    )

    return LaunchDescription([

        DeclareLaunchArgument('namespace', default_value='rexrov'),
        DeclareLaunchArgument('joy_id', default_value='0'),

        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='-20'),
        DeclareLaunchArgument('yaw', default_value='0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo),

            launch_arguments={

                'namespace': namespace,
                'joy_id': joy_id,

                'x': x,
                'y': y,
                'z': z,
                'yaw': yaw,

                'axis_yaw': '2',
                'axis_x': '1',
                'axis_y': '0',
                'axis_z': '5',

                'deadman_button': '-1',
                'exclusion_buttons': '1',

                'axis_oberon_azimuth': '2',
                'axis_oberon_shoulder': '1',
                'axis_oberon_wrist': '0',

                'oberon_exclusion_button': '0',
                'oberon_deadman_button': '1',
                'oberon_home_button': '6',

                'gripper_open_button': '11',
                'gripper_close_button': '10'

            }.items()
        )
    ])
