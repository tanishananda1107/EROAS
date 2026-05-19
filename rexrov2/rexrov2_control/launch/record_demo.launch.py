from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

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
            'uuv_name',
            default_value='rexrov2'
        ),

        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('record')),
            cmd=[
                'ros2', 'bag', 'record',
                '-o', LaunchConfiguration('bag_filename'),

                '/rexrov2/dp_controller/trajectory',
                '/rexrov2/dp_controller/reference',
                '/rexrov2/pose_gt',
                '/hydrodynamics/current_velocity',
                '/rexrov2/thruster_manager/input_stamped',
                '/rexrov2/wrench_perturbation',

                '/rexrov2/thrusters/0/thrust',
                '/rexrov2/thrusters/1/thrust',
                '/rexrov2/thrusters/2/thrust',
                '/rexrov2/thrusters/3/thrust',
                '/rexrov2/thrusters/4/thrust',
                '/rexrov2/thrusters/5/thrust',
            ],
            output='screen'
        )
    ])
