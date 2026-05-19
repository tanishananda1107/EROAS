from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import rclpy
from tf2_ros import TransformListener

def generate_launch_description():
    uuv_name = LaunchConfiguration('uuv_name')

    return LaunchDescription([
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('gui_on', default_value='true'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),

        Node(
            package='uuv_control_utils',
            executable='trajectory_marker_publisher',
            namespace=uuv_name,
            name='trajectory_marker_publisher',
            output='screen'
        ),

        Node(
            package='uuv_trajectory_control',
            executable='auv_geometric_tracking_controller',
            namespace=uuv_name,
            name='auv_geometric_tracking_controller',
            output='screen',
            declare_parameters=[
                {'max_forward_speed': 2.0},
                {'base_link': 'base_link'},
                {'inertial_frame_id': 'world'},
                {'min_thrust': 70.0},
                {'max_thrust': 200.0}
            ],
            remappings=[
                ('odom', 'pose_gt')
            ]
        )
    ])
`catkin_python_setup()` call, as it's not needed in ROS2. I also updated th[2D[K
the `Node` constructor to use the new syntax for remappings and declare par[3D[K
parameters.

