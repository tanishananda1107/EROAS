from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from rclpy.node import Node as RCLNode
import rclpy

def generate_launch_description():
    uuv_name = LaunchConfiguration('uuv_name')

    return LaunchDescription([
        DeclareLaunchArgument('uuv_name'),
        GroupAction([
            PushRosNamespace(uuv_name),
            Node(
                package='uuv_trajectory_control',
                executable='rov_ua_pid_controller',
                name='rov_ua_pid_controller',
                output='screen',
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
            )
        ])
    ])
Note that I removed the `catkin_python_setup()` and replaced it with nothin[6D[K
nothing, as it's not needed in ROS2. I also removed the `rosbuild` dependen[8D[K
dependency and replaced `rospy` with `rclpy`.

