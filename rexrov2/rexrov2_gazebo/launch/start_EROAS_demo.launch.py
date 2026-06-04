import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


GZ_WORLD_NAME = 'oceans_waves'

SNAP_ENV_VARS = [
    'SNAP',
    'SNAP_NAME',
    'SNAP_REVISION',
    'SNAP_VERSION',
    'SNAP_ARCH',
    'SNAP_INSTANCE_NAME',
    'SNAP_CONTEXT',
    'SNAP_COOKIE',
    'SNAP_DATA',
    'SNAP_COMMON',
    'SNAP_USER_DATA',
    'SNAP_USER_COMMON',
    'SNAP_REAL_HOME',
    'SNAP_LIBRARY_PATH',
    'SNAP_LAUNCHER_ARCH_TRIPLET',
    'SNAP_UID',
    'SNAP_EUID',
    'GTK_PATH',
    'LOCPATH',
    'FONTCONFIG_PATH',
    'LD_PRELOAD',
]

BLUE_BLOCK_WORLD = {
    'package': 'uuv_gazebo_worlds',
    'file': 'obstacle_avoidance.world',
    'spawn': ('29', '33', '-60', '2.82'),
    'waypoints': '29,45,-60;42,66,-58;55,87,-54;65,32,-60',
    'target_depth': '-60.0',
}

WORLD_A = {
    'package': 'rexrov2_gazebo',
    'file': 'eroas_world_a.sdf',
    'spawn': ('30', '52', '-56', '1.5708'),
    'waypoints': (
        '29,50,-56;42,66,-56;55,87,-54;35,70,-56;29,45,-56'
    ),
    'target_depth': '-56.0',
}

WORLD_CONFIGS = {
    'blue_blocks': BLUE_BLOCK_WORLD,
    'world_a': WORLD_A,
}


def _join_paths(paths):
    return os.pathsep.join(path for path in paths if path)


def _without_snap_paths(value):
    return os.pathsep.join(
        path for path in value.split(os.pathsep)
        if path and '/snap/' not in path
    )


def _camera_follow_actions():
    track_msg = (
        'track_mode: FOLLOW_LOOK_AT '
        'follow_target: {name: "rexrov2" type: MODEL} '
        'track_target: {name: "rexrov2" type: MODEL} '
        'follow_offset: {x: -8 y: 0 z: 4.5} '
        'track_offset: {x: 0 y: 0 z: 0.5} '
        'follow_pgain: 1.25 '
        'track_pgain: 1.25'
    )
    track_cmd = [
        'gz', 'topic',
        '-t', '/gui/track',
        '-m', 'gz.msgs.CameraTrack',
        '-p', track_msg,
    ]
    follow_cmd = [
        'gz', 'service',
        '-s', '/gui/follow',
        '--reqtype', 'gz.msgs.StringMsg',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', 'data: "rexrov2"',
    ]
    offset_cmd = [
        'gz', 'service',
        '-s', '/gui/follow/offset',
        '--reqtype', 'gz.msgs.Vector3d',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', 'x: -8 y: 0 z: 4.5',
    ]

    actions = []
    for period in (4.0, 7.0, 10.0, 14.0, 20.0, 28.0):
        actions.append(
            TimerAction(
                period=period,
                actions=[
                    ExecuteProcess(cmd=track_cmd, output='screen'),
                    ExecuteProcess(cmd=follow_cmd, output='screen'),
                    ExecuteProcess(cmd=offset_cmd, output='screen'),
                ],
                condition=IfCondition(LaunchConfiguration('auto_follow')),
            )
        )
    return actions


def _setup(context, *args, **kwargs):
    pkg_gazebo = get_package_share_directory('rexrov2_gazebo')
    pkg_worlds = get_package_share_directory('uuv_gazebo_worlds')
    pkg_desc = get_package_share_directory('rexrov2_description')
    pkg_control = get_package_share_directory('rexrov2_control')
    pkg_nps = get_package_share_directory('nps_uw_multibeam_sonar')

    world_name = LaunchConfiguration('world_name').perform(context)
    if world_name not in WORLD_CONFIGS:
        valid = ', '.join(sorted(WORLD_CONFIGS))
        raise RuntimeError(f'Unknown world_name "{world_name}". Valid values: {valid}')

    cfg = WORLD_CONFIGS[world_name]
    world_packages = {
        'rexrov2_gazebo': pkg_gazebo,
        'uuv_gazebo_worlds': pkg_worlds,
    }
    world_path = os.path.join(world_packages[cfg['package']], 'worlds', cfg['file'])
    spawn_x, spawn_y, spawn_z, spawn_yaw = cfg['spawn']

    def auto_value(arg_name, default):
        value = LaunchConfiguration(arg_name).perform(context)
        return default if value == 'auto' else value

    x = auto_value('x', spawn_x)
    y = auto_value('y', spawn_y)
    z = auto_value('z', spawn_z)
    yaw = auto_value('yaw', spawn_yaw)
    gui = LaunchConfiguration('gui').perform(context).lower() in ('1', 'true', 'yes', 'on')
    physics_args = '--physics-engine gz-physics-bullet-featherstone-plugin'
    gz_args = f'{physics_args} -r {world_path}' if gui else f'-s {physics_args} -r {world_path}'

    resource_paths = [
        pkg_gazebo,
        os.path.join(pkg_gazebo, 'models'),
        pkg_worlds,
        os.path.join(pkg_worlds, 'models'),
        os.path.join(pkg_worlds, 'models', 'nerf'),
        pkg_desc,
        os.path.join(pkg_desc, 'meshes'),
        pkg_nps,
        os.path.join(pkg_nps, 'models'),
        os.path.join(os.path.dirname(pkg_gazebo), 'blueview_p900_nps_multibeam'),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]

    plugin_paths = [
        os.path.join(get_package_prefix('uuv_gazebo_ros_plugins'), 'lib'),
        os.path.join(get_package_prefix('uuv_gazebo_plugins'), 'lib'),
        os.path.join(get_package_prefix('uuv_world_ros_plugins'), 'lib'),
        os.path.join(get_package_prefix('nps_uw_multibeam_sonar'), 'lib'),
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
    ]

    waypoints = auto_value('waypoints', cfg['waypoints'])
    ld_library_path = _without_snap_paths(os.environ.get('LD_LIBRARY_PATH', ''))

    return [
        *[SetEnvironmentVariable(name, '') for name in SNAP_ENV_VARS],
        SetEnvironmentVariable('LD_LIBRARY_PATH', ld_library_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', _join_paths(resource_paths)),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', _join_paths(resource_paths)),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', _join_paths(plugin_paths)),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', _join_paths(plugin_paths)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_worlds, 'launch', 'ocean_waves.launch.py')
            ),
            launch_arguments={'gz_args': gz_args}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_desc, 'launch', 'upload_rexrov2.launch.py')),
            launch_arguments={
                'namespace': 'rexrov2',
                'hover_mode': 'true',
                'green_wake': LaunchConfiguration('green_wake'),
                'x': x,
                'y': y,
                'z': z,
                'yaw': yaw,
                'sonar_name': 'blueview_p900',
                'gpu_ray': 'true',
                'maxDistance': '15',
                'fidelity': '500',
                'raySkips': '10',
                'sonar_image_topic': 'rexrov2/blueview_p900/sonar_image',
                'sonar_image_raw_topic': 'rexrov2/blueview_p900/sonar_image_raw',
                'plotScaler': '1',
                'sensorGain': '0.04',
                'ray_visual': 'false',
                'writeLog': 'true',
                'writeFrameInterval': '5',
            }.items(),
        ),

        *_camera_follow_actions(),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='eroas_gz_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/rexrov2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/rexrov2/pose_gt@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/rexrov2/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/rexrov2/blueview_p900_point_cloud@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/world/oceans_waves/create@ros_gz_interfaces/srv/SpawnEntity',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_control, 'launch', 'start_pid_controller.launch.py')),
            launch_arguments={'uuv_name': 'rexrov2', 'model_name': 'rexrov2'}.items(),
            condition=IfCondition(LaunchConfiguration('start_pid_controller')),
        ),

        Node(
            package='navigator_auv',
            executable='only_gap.py',
            name='sonar_heading_node',
            parameters=[{
                'use_sim_time': True,
                'waypoints': waypoints,
                'cmd_vel_topic': '/rexrov2/cmd_vel_1',
                'pose_topic': '/rexrov2/pose_gt',
                'sonar_topic': '/rexrov2/blueview_p900/sonar_image_raw',
                'loop_waypoints': world_name == 'world_a',
                'fallback_speed': 1.2 if world_name == 'world_a' else 0.35,
                'fallback_yaw_kp': 1.45 if world_name == 'world_a' else 0.8,
                'fallback_max_yaw_rate': 1.25 if world_name == 'world_a' else 0.5,
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_navigator')),
        ),

        Node(
            package='navigator_auv',
            executable='hover_hold.py',
            name='rexrov2_hover_hold',
            parameters=[{
                'use_sim_time': True,
                'pose_topic': '/rexrov2/pose_gt',
                'cmd_vel_topic': '/rexrov2/cmd_vel',
                'target_depth': float(cfg['target_depth']),
                'depth_kp': 0.8,
                'max_vertical_speed': 1.2,
                'publish_rate': 20.0,
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_hover_hold')),
        ),

        Node(
            package='navigator_auv',
            executable='velocity_cbf.py',
            name='obstacle_avoidance_node',
            parameters=[{
                'use_sim_time': True,
                'target_depth': float(cfg['target_depth']),
                'depth_hold_kp': 0.18,
                'max_vertical_speed': 0.45,
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_cbf')),
        ),

        Node(
            package='navigator_auv',
            executable='sonar_reconstruction.py',
            name='sonar_reconstruction',
            parameters=[{'use_sim_time': True}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_sonar_reconstruction')),
        ),

        Node(
            package='navigator_auv',
            executable='spawner.py',
            name='eroas_trail_spawner',
            parameters=[{
                'use_sim_time': True,
                'world_name': GZ_WORLD_NAME,
                'pose_topic': '/rexrov2/pose_gt',
                'distance_threshold': 0.35,
                'marker_radius': 0.7,
                'initial_delay': 0.3,
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_trail')),
        ),

        Node(
            package='navigator_auv',
            executable='pose_gt_to_odom.py',
            name='rexrov2_odom_alias',
            parameters=[{
                'use_sim_time': True,
                'input_topic': '/rexrov2/pose_gt',
                'output_topic': '/rexrov2/odom',
            }],
            output='screen',
        ),

        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '--once', '/rexrov2/blueview_p900/sonar_image_raw'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('show_sonar_probe')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='world_a',
                              description='World configuration: world_a or blue_blocks'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x', default_value='auto'),
        DeclareLaunchArgument('y', default_value='auto'),
        DeclareLaunchArgument('z', default_value='auto'),
        DeclareLaunchArgument('yaw', default_value='auto'),
        DeclareLaunchArgument('waypoints', default_value='auto',
                              description='Semicolon-separated x,y,z waypoint list'),
        DeclareLaunchArgument('start_navigator', default_value='false'),
        DeclareLaunchArgument('start_hover_hold', default_value='true'),
        DeclareLaunchArgument('start_cbf', default_value='false'),
        DeclareLaunchArgument('start_pid_controller', default_value='false'),
        DeclareLaunchArgument('start_sonar_reconstruction', default_value='false'),
        DeclareLaunchArgument('start_trail', default_value='false'),
        DeclareLaunchArgument('green_wake', default_value='false'),
        DeclareLaunchArgument('auto_follow', default_value='true'),
        DeclareLaunchArgument('show_sonar_probe', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
