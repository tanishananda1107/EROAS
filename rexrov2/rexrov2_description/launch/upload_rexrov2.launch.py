import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_desc = get_package_share_directory('rexrov2_description')
    namespace = LaunchConfiguration('namespace')
    mode = LaunchConfiguration('mode')
    use_geo = LaunchConfiguration('use_geodetic')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    xacro_file = os.path.join(pkg_desc, 'robots', 'rexrov2_' + LaunchConfiguration('mode').perform({}) if False else 'default.xacro')

    robot_desc = Command([
        'xacro ',
        os.path.join(pkg_desc, 'robots', 'rexrov2_default.xacro'),
        ' namespace:=', namespace,
        ' inertial_reference_frame:=world',
        ' debug:=', LaunchConfiguration('debug'),
        ' use_simplified_mesh:=', LaunchConfiguration('use_simplified_mesh'),
        ' hover_mode:=', LaunchConfiguration('hover_mode'),
        ' green_wake:=', LaunchConfiguration('green_wake'),
        ' sonar_name:=', LaunchConfiguration('sonar_name'),
        ' gpu_ray:=', LaunchConfiguration('gpu_ray'),
        ' maxDistance:=', LaunchConfiguration('maxDistance'),
        ' fidelity:=', LaunchConfiguration('fidelity'),
        ' raySkips:=', LaunchConfiguration('raySkips'),
        ' sonar_image_topic:=', LaunchConfiguration('sonar_image_topic'),
        ' sonar_image_raw_topic:=', LaunchConfiguration('sonar_image_raw_topic'),
        ' plotScaler:=', LaunchConfiguration('plotScaler'),
        ' sensorGain:=', LaunchConfiguration('sensorGain'),
        ' ray_visual:=', LaunchConfiguration('ray_visual'),
        ' writeLog:=', LaunchConfiguration('writeLog'),
        ' writeFrameInterval:=', LaunchConfiguration('writeFrameInterval'),
    ])
    robot_desc_param = ParameterValue(robot_desc, value_type=str)

    return LaunchDescription([

        DeclareLaunchArgument('debug', default_value='0'),
        DeclareLaunchArgument('x', default_value='-40'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='-20'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('namespace', default_value='rexrov2'),
        DeclareLaunchArgument('mode', default_value='default'),
        DeclareLaunchArgument('use_simplified_mesh', default_value='false'),
        DeclareLaunchArgument('hover_mode', default_value='false'),
        DeclareLaunchArgument('green_wake', default_value='false'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),
        DeclareLaunchArgument('use_geodetic', default_value='false'),
        DeclareLaunchArgument('sonar_name', default_value='blueview_p900'),
        DeclareLaunchArgument('gpu_ray', default_value='true'),
        DeclareLaunchArgument('maxDistance', default_value='10'),
        DeclareLaunchArgument('fidelity', default_value='500'),
        DeclareLaunchArgument('raySkips', default_value='1'),
        DeclareLaunchArgument('sonar_image_topic', default_value='sonar_image'),
        DeclareLaunchArgument('sonar_image_raw_topic', default_value='sonar_image_raw'),
        DeclareLaunchArgument('plotScaler', default_value='0'),
        DeclareLaunchArgument('sensorGain', default_value='0.04'),
        DeclareLaunchArgument('ray_visual', default_value='false'),
        DeclareLaunchArgument('writeLog', default_value='false'),
        DeclareLaunchArgument('writeFrameInterval', default_value='5'),

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
                arguments=[
                    '-name', namespace,
                    '-param', 'robot_description',
                    '-x', x,
                    '-y', y,
                    '-z', z,
                    '-R', roll,
                    '-P', pitch,
                    '-Y', yaw,
                ],
                parameters=[{'robot_description': robot_desc_param}],
                output='screen',
                condition=UnlessCondition(use_geo)
            ),
        ]),

    ])
