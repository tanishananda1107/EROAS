
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from rclpy.node import Node as RCLNode
from tf2_ros import TransformListener
from launch.substitutions.text_substitution import TextSubstitution


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_name'),
        DeclareLaunchArgument('uuv_name'),

        Node(
            package='uuv_control_cascaded_pid',
            executable='acceleration_control',
            name='acceleration_control',
            output='log',
            parameters=[declare_parameter('uuv_name', LaunchConfiguration('[21D[K
LaunchConfiguration('uuv_name'))]
        ),

        Node(
            package='uuv_control_cascaded_pid',
            executable='velocity_control',
            name='velocity_control',
            output='log',
            remappings=[
                ('odom', '/pose_gt'),
                ('cmd_accel', '/cmd_accel')
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                TextSubstitution(text=os.path.join(get_package_share_direct[59D[K
TextSubstitution(text=os.path.join(get_package_share_directory('uuv_teleop'TextSubstitution(text=os.path.join(get_package_share_directry('uuv_teleop'), 'launch', 'uuv_keyboard_teleop.launch.py'))
            ),
            launch_arguments={
                'uuv_name': LaunchConfiguration('uuv_name'),
                'output_topic': 'cmd_vel',
                'message_type': 'twist'
            }.items()
        )
    ])

Please note that the changes were made according to the rules provided, whi[3D[K
which include:

* Replacing `rospy` with `rclpy`
* Replacing `tf` with `tf2_ros`
* Replacing `catkin` with `ament_cmake`
* Removing `catkin_python_setup()`
* Removing `catkin_install_python`
* Changing `CATKIN_PACKAGE_BIN_DESTINATION` and `CATKIN_PACKAGE_SHARE_DESTI[27D[K
`CATKIN_PACKAGE_SHARE_DESTINATION`
* Updating package.xml dependencies
* Replacing `rospy.get_param()` with `declare_parameter()`
* Replacing `rospy.Time.now()` with `node.get_clock().now()`
* Replacing `rospy.get_time()` with `clock.nanoseconds`

