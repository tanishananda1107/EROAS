import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_desc = get_package_share_directory('rexrov2_description')

    namespace = LaunchConfiguration('namespace')

    robot_desc = Command([
        'xacro ',
        os.path.join(pkg_desc, 'robots', 'rexrov2_oberon_arms.xacro'),
        ' namespace:=', namespace,
    ])
    robot_desc_param = ParameterValue(robot_desc, value_type=str)

    return LaunchDescription([

        DeclareLaunchArgument('namespace', default_value='rexrov2'),

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
                output='screen'
            ),
        ]),
    ])
