from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Debug flag
    debug = LaunchConfiguration('debug')
    debug_arg = DeclareLaunchArgument('debug', default_value='0')

    # Vehicle's initial pose
    x = LaunchConfiguration('x')
    x_arg = DeclareLaunchArgument('x', default_value='0')
    y = LaunchConfiguration('y')
    y_arg = DeclareLaunchArgument('y', default_value='0')
    z = LaunchConfiguration('z')
    z_arg = DeclareLaunchArgument('z', default_value='-20')
    roll = LaunchConfiguration('roll')
    roll_arg = DeclareLaunchArgument('roll', default_value='0')
    pitch = LaunchConfiguration('pitch')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0')
    yaw = LaunchConfiguration('yaw')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0')

    # Mode to open different robot configurations
    mode = LaunchConfiguration('mode')
    mode_arg = DeclareLaunchArgument('mode', default_value='default')

    # Vehicle's namespace
    namespace = LaunchConfiguration('namespace')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='rov_example')

    # World frame
    world_frame = LaunchConfiguration('world_frame')
    world_frame_arg = DeclareLaunchArgument('world_frame', default_value='world')

    # Create the robot description
    robot_description_command = [
        PathJoinSubstitution([FindPackageShare('xacro'), 'xacro.py']),
        ' ',
        PathJoinSubstitution([FindPackageShare('uuv_tutorial_rov_model'),
                              'robots',
                              'rov_example_' + LaunchConfiguration('mode').perform() + '.xacro']),
        ' debug:=', debug,
        ' namespace:=', namespace
    ]

    # URDF spawner node
    urdf_spawner = Node(
        package='gazebo_ros',
        executable='spawn_model',
        name='urdf_spawner',
        output='screen',
        arguments=[
            '-urdf',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw,
            '-model', namespace,
            '-param', '/$(arg namespace)/robot_description'
        ]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '/$(arg namespace)/robot_description'
        }],
        respawn=True
    )

    # Include message_to_tf.launch
    message_to_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_assistants'),
                'launch',
                'message_to_tf.launch'
            ])
        ),
        launch_arguments={
            'namespace': namespace
        }.items()
    )

    # Group action with namespace
    ns_group = GroupAction([
        PushRosNamespace(namespace),
        urdf_spawner,
        robot_state_publisher,
        message_to_tf_launch
    ])

    return LaunchDescription([
        debug_arg,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        mode_arg,
        namespace_arg,
        world_frame_arg,
        ns_group
    ])
