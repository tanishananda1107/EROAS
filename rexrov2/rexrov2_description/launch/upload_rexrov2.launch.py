import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_desc = get_package_share_directory('rexrov2_description')
    pkg_control = get_package_share_directory('rexrov2_control')

    namespace = LaunchConfiguration('namespace')
    mode = LaunchConfiguration('mode')
    use_ned = LaunchConfiguration('use_ned_frame')
    use_geo = LaunchConfiguration('use_geodetic')

    xacro_file = os.path.join(pkg_desc, 'robots', 'rexrov2_' + LaunchConfiguration('mode').perform({}) if False else 'default.xacro')

    robot_desc = Command([
        'xacro ',
        os.path.join(pkg_desc, 'robots', 'rexrov2_default.xacro'),
        ' namespace:=', namespace,
        ' inertial_reference_frame:=world'
    ])
    robot_desc_param = ParameterValue(robot_desc, value_type=str)

    return LaunchDescription([

        DeclareLaunchArgument('namespace', default_value='rexrov2'),
        DeclareLaunchArgument('mode', default_value='default'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),
        DeclareLaunchArgument('use_geodetic', default_value='false'),

        GroupAction([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=namespace,
                parameters=[{'robot_description': robot_desc_param}]
            ),

            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-name', namespace, '-param', 'robot_description'],
                parameters=[{'robot_description': robot_desc_param}],
                output='screen',
                condition=UnlessCondition(use_geo)
            ),
        ]),

    ])
