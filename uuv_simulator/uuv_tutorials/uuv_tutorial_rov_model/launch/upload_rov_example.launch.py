from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # --------------------------------------------------------------------------
    # Launch arguments
    # --------------------------------------------------------------------------

    debug = LaunchConfiguration('debug')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')

    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    mode = LaunchConfiguration('mode')

    namespace = LaunchConfiguration('namespace')

    world_frame = LaunchConfiguration('world_frame')

    # --------------------------------------------------------------------------
    # Robot description
    # --------------------------------------------------------------------------

    xacro_file = PathJoinSubstitution([
        FindPackageShare('uuv_tutorial_rov_model'),
        'robots',
        ['rov_example_', mode, '.xacro']
    ])

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' debug:=', debug,
            ' namespace:=', namespace
        ]),
        value_type=str
    )

    # --------------------------------------------------------------------------
    # Robot State Publisher
    # --------------------------------------------------------------------------

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # --------------------------------------------------------------------------
    # Spawn robot into Gazebo Harmonic / GZ Sim 8
    # --------------------------------------------------------------------------

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', namespace,
            '-topic', 'robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ]
    )

    # --------------------------------------------------------------------------
    # TF helper node
    # --------------------------------------------------------------------------

    tf_helper = Node(
        package='uuv_assistants',
        executable='message_to_tf',
        name='message_to_tf',
        output='screen',
        parameters=[{
            'namespace': namespace,
            'world_frame': world_frame,
            'use_sim_time': True
        }]
    )

    # --------------------------------------------------------------------------
    # Group under namespace
    # --------------------------------------------------------------------------

    robot_group = GroupAction([
        PushRosNamespace(namespace),

        robot_state_publisher_node,
        spawn_robot
    ])

    # --------------------------------------------------------------------------
    # Launch description
    # --------------------------------------------------------------------------

    return LaunchDescription([

        DeclareLaunchArgument(
            'debug',
            default_value='0'
        ),

        DeclareLaunchArgument(
            'x',
            default_value='0'
        ),

        DeclareLaunchArgument(
            'y',
            default_value='0'
        ),

        DeclareLaunchArgument(
            'z',
            default_value='-20'
        ),

        DeclareLaunchArgument(
            'roll',
            default_value='0'
        ),

        DeclareLaunchArgument(
            'pitch',
            default_value='0'
        ),

        DeclareLaunchArgument(
            'yaw',
            default_value='0'
        ),

        DeclareLaunchArgument(
            'mode',
            default_value='default'
        ),

        DeclareLaunchArgument(
            'namespace',
            default_value='rov_example'
        ),

        DeclareLaunchArgument(
            'world_frame',
            default_value='world'
        ),

        robot_group,

        tf_helper
    ])
