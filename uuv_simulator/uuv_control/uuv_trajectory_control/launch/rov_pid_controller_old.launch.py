
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import os

def generate_launch_description():

    uuv_name = LaunchConfiguration('uuv_name')
    model_name = LaunchConfiguration('model_name')
    gui_on = LaunchConfiguration('gui_on')
    use_ned_frame = LaunchConfiguration('use_ned_frame')

    return LaunchDescription([

        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('model_name', default_value=uuv_name),
        DeclareLaunchArgument('gui_on', default_value='true'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),

        GroupAction([
            PushRosNamespace(uuv_name),

            Node(
                package='uuv_trajectory_control',
                executable='rov_pid_controller',
                name='rov_pid_controller',
                output='screen',
                condition=UnlessCondition(use_ned_frame),
                parameters=[{
                    'inertial_frame_id': 'world'
                }],
                remappings=[
                    ('odom', 'pose_gt'),
                    ('trajectory', 'dp_controller/trajectory'),
                    ('input_trajectory', 'dp_controller/input_trajectory'),[34D[K
'dp_controller/input_trajectory'),
                    ('waypoints', 'dp_con[7D[K
'dp_controller/waypoints'),
                    ('error', 'dp_controller/error'),
                    ('reference', 'dp_controller/reference'),
                    ('thruster_output', 'thruster_manager/input_stamped')
                ]
            ),

            Node(
                package='uuv_trajectory_control',
                executable='rov_pid_controller',
                name='rov_pid_controller',
                output='screen',
                condition=IfCondition(use_ned_frame),
                parameters=[{
                    'inertial_frame_id': 'world_ned'
                }],
                remappings=[
                    ('odom', 'pose_gt_ned'),
                    ('trajectory', 'dp_controller/trajectory'),
                    ('input_trajectory', 'dp_controller/input_trajectory'),[34D[K
'dp_controller/input_trajectory'),
                    ('waypoints', 'dp_con[7D[K
'dp_controller/waypoints'),
                    ('error', 'dp_controller/error'),
                    ('reference', 'dp_controller/reference'),
                    ('thruster_output', 'thruster_manager/input_stamped')
                ]
            )
        ])
    ])

Note: There were no changes required to the Python code as it only contains[8D[K
with ROS2.

