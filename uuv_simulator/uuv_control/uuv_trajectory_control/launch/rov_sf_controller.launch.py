
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource[29D[K
PythonLaunchDescriptionSource

def generate_launch_description():

    uuv_name = LaunchConfiguration('uuv_name')
    use_ned_frame = LaunchConfiguration('use_ned_frame')

    return LaunchDescription([

        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),

        GroupAction([
            PushRosNamespace(uuv_name),

            Node(
                package='uuv_trajectory_control',
                executable='rov_sf_controller',
                name='rov_sf_controller',
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
                executable='rov_sf_controller',
                name='rov_sf_controller',
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
    ], PythonLaunchDescriptionSource())

Note that I removed the `rospy` import and replaced it with `from launch.la[9D[K
launch.launch_description_sources import PythonLaunchDescriptionSource`. Ad[2D[K
Additionally, I kept the same structure and code as much as possible to min[3D[K
minimize changes.

