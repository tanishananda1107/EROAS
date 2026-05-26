from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    record = LaunchConfiguration('record')
    bag_filename = LaunchConfiguration('bag_filename')
    use_ned_frame = LaunchConfiguration('use_ned_frame')

    ned_topics = [
        '/rexrov/dp_controller/trajectory',
        '/rexrov/dp_controller/reference',
        '/rexrov/pose_gt_ned',
        '/hydrodynamics/current_velocity',
        '/rexrov/thruster_manager/input',
        '/rexrov/wrench_perturbation',
        '/rexrov/thrusters/0/thrust',
        '/rexrov/thrusters/1/thrust',
        '/rexrov/thrusters/2/thrust',
        '/rexrov/thrusters/3/thrust',
        '/rexrov/thrusters/4/thrust',
        '/rexrov/thrusters/5/thrust',
        '/rexrov/thrusters/6/thrust',
        '/rexrov/thrusters/7/thrust'
    ]

    enu_topics = ned_topics.copy()
    enu_topics[2] = '/rexrov/pose_gt'

    return LaunchDescription([

        DeclareLaunchArgument(
            'record',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'bag_filename',
            default_value='recording'
        ),

        DeclareLaunchArgument(
            'use_ned_frame',
            default_value='false'
        ),

        ExecuteProcess(
            condition=IfCondition(record),
            cmd=[
                'ros2', 'bag', 'record',
                '-o', bag_filename,
                *ned_topics
            ],
            output='screen'
        ),

        ExecuteProcess(
            condition=UnlessCondition(use_ned_frame),
            cmd=[
                'ros2', 'bag', 'record',
                '-o', bag_filename,
                *enu_topics
            ],
            output='screen'
        )
    ])
