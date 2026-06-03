from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():

    gazebo = FindPackageShare('rexrov2_gazebo').find('rexrov2_gazebo')
    worlds = FindPackageShare('uuv_gazebo_worlds').find('uuv_gazebo_worlds')
    desc = FindPackageShare('rexrov2_description').find('rexrov2_description')
    control = FindPackageShare('rexrov2_control').find('rexrov2_control')

    return LaunchDescription([

        DeclareLaunchArgument('x', default_value='25'),
        DeclareLaunchArgument('y', default_value='-35'),
        DeclareLaunchArgument('z', default_value='-47'),
        DeclareLaunchArgument('yaw', default_value='2.27'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag_filename', default_value='recording.db3'),
        DeclareLaunchArgument('teleop_on', default_value='false'),
        DeclareLaunchArgument('joy_id', default_value='0'),
        DeclareLaunchArgument('gui', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(worlds, 'launch', 'coral.launch.py')),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(desc, 'launch', 'upload_rexrov2.launch.py')),
            launch_arguments={
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'yaw': LaunchConfiguration('yaw'),
                'sonar_name': 'blueview_p900',
                'gpu_ray': 'true',
                'maxDistance': '15',
                'fidelity': '500',
                'raySkips': '10',
                'sonar_image_topic': 'sonar_image',
                'sonar_image_raw_topic': 'sonar_image_raw',
                'plotScaler': '1',
                'sensorGain': '0.04',
                'ray_visual': 'true',
                'writeLog': 'true',
                'writeFrameInterval': '5',
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(control, 'launch', 'start_pid_controller.launch.py')),
            launch_arguments={
                'uuv_name': 'rexrov2',
                'model_name': 'rexrov2',
            }.items()
        ),

        Node(
            package='navigator_auv',
            executable='just_gap.py',
            name='sonar_heading_node',
            output='screen',
        ),

        Node(
            package='navigator_auv',
            executable='velocity_cbf.py',
            name='obstacle_avoidance_node',
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'record.launch.py')),
            launch_arguments={
                'record': LaunchConfiguration('record'),
                'bag_filename': LaunchConfiguration('bag_filename'),
            }.items()
        ),
    ])
