
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource[29D[K
PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from rclpy import node as rosnode
from tf2_ros import TransformBroadcaster
from rclpy.clock import Clock

def generate_launch_description():
    model_name = DeclareLaunchArgument('model_name')
    uuv_name = DeclareLaunchArgument('uuv_name', default_value='default_mod[26D[K
default_value='default_model')
    joy_id = DeclareLaunchArgument('joy_id', default_value='0')

    thruster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_thruster_manager'),
                'launch',
                'thruster_manager.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': uuv_name,
            'model_name': model_name
        }.items()
    )

    accel_node = Node(
        package='uuv_control_cascaded_pid',
        executable='acceleration_control',
        name='acceleration_control',
        output='screen',
        parameters=[
            {'tf_prefix': uuv_name},
            {'rclpy.node.get_clock().now()': Clock().now()}
        ]
    )

    vel_node = Node(
        package='uuv_control_cascaded_pid',
        executable='velocity_control',
        name='velocity_control',
        output='screen',
        remappings=[
            ('odom', f'/{uuv_name}/pose_gt'),
            ('cmd_accel', f'/{uuv_name}/cmd_accel')
        ]
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_teleop'),
                'launch',
                'uuv_teleop.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': uuv_name,
            'joy_id': joy_id,
            'output_topic': 'cmd_vel',
            'message_type': 'geometry_msgs.msg.Twist'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('model_name'),
        model_name,
        DeclareLaunchArgument('uuv_name', default_value=model_name),
        uuv_name,
        DeclareLaunchArgument('joy_id', default_value='0'),
        joy_id,

        thruster_launch,
        accel_node,
        vel_node,
        teleop_launch
    ])

