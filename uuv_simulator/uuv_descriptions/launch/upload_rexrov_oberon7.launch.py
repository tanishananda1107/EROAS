from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    debug = LaunchConfiguration('debug')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')

    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    namespace = LaunchConfiguration('namespace')
    use_ned_frame = LaunchConfiguration('use_ned_frame')

    world_robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('uuv_descriptions'),
                'robots',
                'rexrov_oberon7.xacro'
            ]),
            ' debug:=', debug,
            ' namespace:=', namespace,
            ' inertial_reference_frame:=world'
        ]),
        value_type=str
    )

    ned_robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('uuv_descriptions'),
                'robots',
                'rexrov_oberon7.xacro'
            ]),
            ' debug:=', debug,
            ' namespace:=', namespace,
            ' inertial_reference_frame:=world_ned'
        ]),
        value_type=str
    )

    world_group = GroupAction(
        condition=UnlessCondition(use_ned_frame),
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=namespace,
                output='screen',
                parameters=[
                    {
                        'robot_description':
                            world_robot_description
                    }
                ]
            ),

            Node(
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
                ],
                namespace=namespace
            )
        ]
    )

    ned_group = GroupAction(
        condition=IfCondition(use_ned_frame),
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=namespace,
                output='screen',
                parameters=[
                    {
                        'robot_description':
                            ned_robot_description
                    }
                ]
            ),

            Node(
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
                ],
                namespace=namespace
            )
        ]
    )

    oberon7_params = SetParameter(
        name='arms.oberon7.robot_config',
        value=PathJoinSubstitution([
            FindPackageShare(
                'oberon7_description'
            ),
            'params',
            'robot_config.yaml'
        ])
    )

    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(
                    'uuv_assistants'
                ),
                'launch',
                'message_to_tf.launch.py'
            ])
        ),
        launch_arguments={
            'namespace': namespace
        }.items()
    )

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
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'pitch',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'yaw',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'namespace',
            default_value='rexrov'
        ),

        DeclareLaunchArgument(
            'use_ned_frame',
            default_value='false'
        ),

        world_group,
        ned_group,

        SetParameter(
            name='arms.n_arms',
            value=1
        ),

        SetParameter(
            name='arms.names',
            value=['oberon7']
        ),

        oberon7_params,

        tf_launch
    ])
