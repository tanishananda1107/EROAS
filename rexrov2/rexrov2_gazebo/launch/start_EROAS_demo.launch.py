import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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
    'gz_world_name': 'eroas_world_a',
    'spawn': ('29', '33', '-54', '1.5708'),
    'waypoints': '29,97,-50;31,110,-55;30,90,-90;30,120,-40',
    'target_depth': '-54.0',
    'cruise_speed': 0.42,
    'fallback_speed': 0.30,
    'waypoint_tolerance': 5.0,
    'final_waypoint_tolerance': 5.0,
    'planar_waypoint_tolerance': True,
    'loop_waypoints': False,
    'green_wake': 'true',
    'min_forward_speed': 0.16,
    'max_vertical_speed': 0.22,
    'collision_distance': 2.2,
    'hard_stop_distance': 0.9,
    'scan_yaw_rate': 0.35,
    'use_point_cloud_obstacles': True,
    'use_analytical_obstacles': False,
    # Harmonic uses the GPU-lidar PointCloud2 stream as the FLS source.  The
    # sonar_frame_converter node rebuilds the ProjectedSonarImage profile for
    # the paper-faithful SPD2C gap logic (use_raw_sonar consumes that
    # synthetic image), while ST-CBF consumes the raw points directly.
    'use_raw_sonar': True,
    'use_point_cloud_sonar': True,
    'prefer_point_cloud_sonar': True,
    'min_detection_range': 0.45,
    'point_cloud_min_range': 0.45,
    'point_cloud_body_clearance_x': 0.60,
    'point_cloud_body_clearance_y': 1.20,
    'point_cloud_body_clearance_z': 1.00,
    'point_cloud_max_abs_z': 0.0,
    'depth_deadband': 0.15,
    'depth_hold_kp': 0.22,
    'safety_radius_xy': 1.35,
    'safety_radius_xz': 1.25,
    'safety_radius_sonar_pivot': 1.35,
    'max_active_constraints': 3,
    'pivot_min_angle': -0.8,
    'pivot_max_angle': 0.8,
    'pivot_sample_count': 81,
    'vertical_detection_threshold': 120.0,
    'spatial_memory_timeout': 8.0,
    'spatial_memory_radius': 15.0,
    'spatial_memory_voxel_size': 0.35,
    'spatial_memory_max_points': 3500,
    'barrier_slide_enabled': False,
    'barrier_slide_trigger_h': -1.0,
    'hover_lock_enabled': False,
    'attitude_hold_enabled': True,
    'attitude_hold_kp': 1.2,
    'attitude_hold_max_rate': 0.45,
    'attitude_recovery_tilt': 0.45,
    'paper_controller': True,
    'paper_k_t': 0.12,
    'paper_k_v': 0.35,
    'paper_psi_max': 1.5707963267948966,
    'paper_vx_max': 1.0,
    'paper_max_yaw_rate': 0.26,
    'paper_convexity_threshold': 0.02,
}

WORLD_A = {
    'package': 'uuv_gazebo_worlds',
    'file': 'obstacle_avoidance.world',
    'gz_world_name': 'eroas_world_a',
    # Paper Figure 8 / original EROAS World A: use the existing DAVE/UUV
    # obstacle_avoidance.world course and goal-behind-obstacle route.
    # Spawn matches the ROS1 reference's commented-out "World A" block in
    # start_EROAS_demo.launch (x=29 y=33 z=-54 yaw=2.82) -- the launch file's
    # active (uncommented) defaults there are actually World B's spawn.
    'spawn': ('29', '33', '-54', '2.82'),
    'waypoints': '29,97,-50;31,110,-55;30,90,-90;30,120,-40',
    'target_depth': '-54.0',
    'cruise_speed': 0.5,
    'fallback_speed': 0.35,
    'waypoint_tolerance': 5.0,
    'final_waypoint_tolerance': 5.0,
    # XY progression while CBF/planner handle depth (Figure 8 dive segment).
    'planar_waypoint_tolerance': True,
    'loop_waypoints': False,
    'green_wake': 'false',
    'paper_controller': True,
    'paper_k_t': 0.12,
    'paper_k_v': 0.35,
    'paper_psi_max': 1.5707963267948966,
    'paper_vx_max': 1.0,
    'paper_max_yaw_rate': 0.26,
    'min_forward_speed': 0.18,
    'max_vertical_speed': 0.5,
    'collision_distance': 2.2,
    'hard_stop_distance': 0.9,
    'scan_yaw_rate': 0.35,
    'use_point_cloud_obstacles': True,
    # The paper's ST-CBF is driven by the closest obstacle retained by SCG,
    # not by a preloaded map of analytical boxes.  Those boxes caused the
    # Harmonic port to wall-slide around cube_7 while SPD2C saw a clear gap.
    'use_analytical_obstacles': False,
    # Use the synthesized BlueView sonar profile for SPD2C, matching the
    # original implementation.  PointCloud2 remains enabled for ST-CBF in the
    # safety-filter node, but the navigation decision itself must come from
    # the FLS profile rather than a point-cloud shortcut.
    'use_raw_sonar': True,
    'use_point_cloud_sonar': True,
    'prefer_point_cloud_sonar': True,
    'min_detection_range': 0.45,
    'raw_sonar_self_echo_range': 0.90,
    'point_cloud_min_range': 0.45,
    'point_cloud_body_clearance_x': 0.60,
    'point_cloud_body_clearance_y': 1.20,
    'point_cloud_body_clearance_z': 1.00,
    'point_cloud_max_abs_z': 0.0,
    'xz_context_lateral_window': 2.0,
    'hover_lock_enabled': False,
    'depth_deadband': 0.15,
    'depth_hold_kp': 0.35,
    'recovery_trigger_seconds': 3.0,
    'safety_radius_xy': 2.0,
    'safety_radius_xz': 2.0,
    'safety_radius_sonar_pivot': 2.0,
    # _scg_constraints (velocity_cbf.py) now emits one real constraint per
    # detected obstacle sector instead of a single guessed point (see that
    # function's docstring). A pinch with obstacles flanking both sides of
    # the gap -- e.g. cube_6_1/cube_7 -- needs both visible at once, or the
    # QP only ever sees one wall and has no way to represent the corridor.
    'max_active_constraints': 3,
    # Algorithm 1 uses a complete elevation sweep and then accepts a vertical
    # corridor only from a length-30 run. Headless validation against the
    # cube_5/cube_6/cube_6_1/cube_7 cluster (best_run vs required=88-of-300
    # beams, logged per angle) showed genuinely wide-open corridors exist at
    # only a few, scattered elevations within the old +/-0.8 rad range --
    # not enough to sustain a 30-consecutive-angle run. Widened toward the
    # sonar_vertical_joint's actual limit (+/-1.5708 rad, rexrov2_sensors.xacro)
    # with a small margin, giving the sweep more elevations to find a
    # sustained clear run at this specific pinch.
    'pivot_min_angle': -1.4,
    'pivot_max_angle': 1.4,
    'pivot_sample_count': 81,
    # Sonar pivot joint has an 8 rad/s velocity limit (rexrov2_sensors.xacro),
    # so even the largest step (+/-0.8 rad) settles in well under 0.1s -- 5.0s
    # per sample meant the full 81-sample sweep took 405s, far longer than
    # any test window and effectively making vertical avoidance never finish.
    'pivot_sample_timeout': 0.3,
    'pivot_sample_retries': 4,
    # vertical_gap_run_length is a *sample count*, not an angle. With the
    # original +/-0.8 rad range over 81 samples, 30 samples was a ~0.59 rad
    # (~34deg) corridor requirement -- the number the paper's Algorithm 1
    # was effectively calibrated against. Widening the range to +/-1.4 rad
    # without changing pivot_sample_count coarsened resolution, so leaving
    # this at 30 would have silently demanded a ~1.04 rad (~59deg) corridor
    # instead: a stricter requirement even though the search got broader.
    # Scaled down to keep the same ~34deg corridor requirement over the
    # wider search range: 30 * (1.6/2.8) ~= 17.
    'vertical_gap_run_length': 17,
    # The 30-angle corridor is point-clear. Move to another valid midpoint
    # 0.12 rad farther inside it so the full REXROV hull retains d_min=2 m.
    'vertical_gap_safety_margin_rad': 0.12,
    # Match the paper's free/occupied classification after PointCloud2 is
    # converted into the synthetic ProjectedSonarImage intensity scale.
    'vertical_detection_threshold': 120.0,
    # Hold position while Gazebo sensors and the FLS stream initialize.
    'startup_hover_duration': 0.0,
    'startup_sensor_wait_timeout': 0.0,
    'gz_update_rate': 60,
    # ST-CBF should minimally project the SPD2C reference.  The additional
    # fixed-speed wall-slide is reserved for an actual near-collision.
    # (barrier_slide is dead weight here regardless of its value: WORLD_A
    # sets paper_controller=True, which routes velocity_cbf.py through
    # _process_data_paper_cbf / _opt_xy_paper -- a pure min-norm CBF-QP
    # projection with no barrier-slide/hover-lock/stall-recovery hooks at
    # all, per that method's own docstring. Headless testing on 2026-08-07
    # confirmed enabling this had zero effect on a reproducible SPD2C/CBF
    # deadlock at cube_6_1: only_gap.py's gap search has no safety-radius
    # inflation and commits to headings the CBF's inflated SCG model treats
    # as fully blocked (obstacle_boundaries spanning the whole FOV,
    # context_h<0); the min-norm projection then has ~zero tangential
    # component to work with since the requested velocity is nearly all
    # forward, and there is no fallback heuristic on this code path to
    # nudge it free. Net displacement was <2m over 100s in both cases.)
    'barrier_slide_trigger_h': -1.0,
    'barrier_slide_speed': 0.20,
    'barrier_slide_enabled': False,
    'vertical_escape_duration': 180.0,
    # Finish the climb tightly around the first waypoint's safe depth.
    'vertical_depth_tolerance': 0.2,
    # The climb ends on the first waypoint's safe depth; the post-vertical
    # route state then carries the vehicle beyond the obstacle footprint.
    'vertical_escape_min_duration': 4.0,
    'vertical_escape_min_planar_distance': 0.0,
    # Resume only long enough to clear cube_7. A 180 s override suppressed the
    # paper waterfall at the next horizontal body and drove into its collision
    # geometry instead of allowing a new gap/convergence/pivot decision.
    'post_vertical_resume_duration': 30.0,
    # Figure 6 retains sonar obstacles spatially until they leave the local
    # radius. The complete Harmonic pivot takes over two minutes, so a 7 s
    # timeout erased cube_7 before the vertical CBF maneuver began.
    'spatial_memory_timeout': 8.0,
    'spatial_memory_radius': 15.0,
    'spatial_memory_voxel_size': 0.35,
    'spatial_memory_max_points': 3500,
    # Gazebo Classic used a low-level PID to keep the hovering AUV level.
    # Harmonic's velocity system needs the equivalent roll/pitch rate loop.
    'attitude_hold_enabled': True,
    'attitude_hold_kp': 1.2,
    'attitude_hold_max_rate': 0.45,
    'attitude_recovery_tilt': 0.45,
}

# World A with old-repo Figure 5 spawn/waypoints for SPD2C+CBF navigation
WORLD_A_FIGURE5 = {
    **WORLD_A,
    'spawn': ('24', '55', '-56', '1.5708'),
    'waypoints': '28,59,-56;34,62,-56;44,65,-56;55,66,-56;62,75,-56;61,88,-56;55,92,-56',
    'target_depth': '-56.0',
    'cruise_speed': 0.38,
    'use_raw_sonar': True,
    'use_point_cloud_sonar': True,
    'prefer_point_cloud_sonar': True,
    'green_wake': 'true',
    'gz_update_rate': 60,
    'depth_hold_kp': 0.14,
    'max_vertical_speed': 0.16,
    'spatial_memory_timeout': 8.0,
    'spatial_memory_radius': 15.0,
}

WORLD_CONFIGS = {
    'blue_blocks': BLUE_BLOCK_WORLD,
    'world_a': WORLD_A,
    'world_a_figure5': WORLD_A_FIGURE5,
    'world_b': {
        'package': 'rexrov2_gazebo',
        'file': 'eroas_world_b.sdf',
        'gz_world_name': 'oceans_waves',
        # Spawn at origin facing +X, obstacle directly ahead at x=12
        'spawn': ('0', '0', '-20', '0.0'),
        'waypoints': '0,0,-20;30,0,-20',
        'target_depth': '-20.0',
        'cruise_speed': 0.45,
        'loop_waypoints': False,
        'green_wake': 'false',
        'recovery_trigger_seconds': 3.0,
        'safety_radius_xy': 1.05,
        'safety_radius_xz': 1.00,
        'safety_radius_sonar_pivot': 1.05,
    },
}

TRUTHY_LAUNCH_VALUES = "('1', 'true', 'yes', 'on')"


def _join_paths(paths):
    return os.pathsep.join(path for path in paths if path)


def _without_snap_paths(value):
    return os.pathsep.join(
        path for path in value.split(os.pathsep)
        if path and '/snap/' not in path
    )


def _enabled_and_not_hover(enabled_name):
    return PythonExpression([
        "'", LaunchConfiguration(enabled_name),
        "'.lower() in ", TRUTHY_LAUNCH_VALUES,
        " and '", LaunchConfiguration('start_hover_hold'),
        "'.lower() not in ", TRUTHY_LAUNCH_VALUES,
    ])


def _enabled_and_not_waypoint_hover(enabled_name):
    return PythonExpression([
        "'", LaunchConfiguration(enabled_name),
        "'.lower() in ", TRUTHY_LAUNCH_VALUES,
        " and '", LaunchConfiguration('start_hover_hold'),
        "'.lower() not in ", TRUTHY_LAUNCH_VALUES,
        " and '", LaunchConfiguration('start_waypoint_hover'),
        "'.lower() not in ", TRUTHY_LAUNCH_VALUES,
    ])


def _camera_follow_actions():
    track_msg = (
        'track_mode: FOLLOW '
        'follow_target: {name: "rexrov2" type: MODEL} '
        'track_target: {name: "rexrov2" type: MODEL} '
        'follow_offset: {x: -50 y: 0 z: 30} '
        'track_offset: {x: 0 y: 0 z: 0} '
        'follow_pgain: 0.5 '
        'track_pgain: 0.5'
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
        '--req', 'x: -16 y: 0 z: 8.5',
    ]

    return [
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(cmd=track_cmd, output='screen'),
                ExecuteProcess(cmd=follow_cmd, output='screen'),
                ExecuteProcess(cmd=offset_cmd, output='screen'),
            ],
            condition=IfCondition(LaunchConfiguration('auto_follow')),
        )
    ]


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

    start_waypoint_hover_val = LaunchConfiguration('start_waypoint_hover').perform(context).lower()
    if start_waypoint_hover_val in ('1', 'true', 'yes', 'on'):
        start_nav_val = LaunchConfiguration('start_navigator').perform(context).lower()
        start_cbf_val = LaunchConfiguration('start_cbf').perform(context).lower()
        if start_nav_val in ('1', 'true', 'yes', 'on') or start_cbf_val in ('1', 'true', 'yes', 'on'):
            print(
                '[WARNING] start_waypoint_hover:=true overrides start_navigator and start_cbf -- '
                'those nodes will NOT be started to avoid cmd_vel conflict.',
                flush=True,
            )

    cfg = WORLD_CONFIGS[world_name]
    gz_world_name = cfg.get('gz_world_name', 'oceans_waves')
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
    green_wake = auto_value('green_wake', cfg.get('green_wake', 'false'))
    gui = LaunchConfiguration('gui').perform(context).lower() in ('1', 'true', 'yes', 'on')
    start_pid_thrusters = (
        LaunchConfiguration('start_pid_thrusters').perform(context).lower()
        in ('1', 'true', 'yes', 'on')
    )
    # In Gazebo Harmonic with hover_mode=true, the gz-sim VelocityControl
    # plugin must always be active — it is the only actuator that works.
    # The classic UUV thruster plugins (libuuv_thruster_ros_plugin.so) do not
    # load in gz-sim, so the external ROS2 PID→thruster chain has no effect.
    use_velocity_control_plugin = 'true'
    physics_args = '--physics-engine gz-physics-bullet-featherstone-plugin'
    gz_args = f'{physics_args} -r {world_path}' if gui else f'-s {physics_args} -r {world_path}'
    # The <sensor> tag is authored on blueview_p900_link (a fixed child of
    # the pivoting rexrov2/sonar_link -- see multibeam_blueview_p900_macro in
    # multibeam_sonar_blueview_p900.xacro), but multibeam_sonar_joint is a
    # `type="fixed"` joint, so sdformat's URDF->SDF conversion lumps
    # blueview_p900_link into its fixed-joint parent during flattening.  The
    # sensor keeps its own name but ends up scoped under the *parent* link in
    # gz-sim's auto-generated topic (verified against the live `gz topic -l`
    # output), so the path must name rexrov2/sonar_link here, not
    # blueview_p900_link -- otherwise this bridge subscribes to a gz topic
    # that is never published and the point cloud never reaches ROS2.
    sonar_points_gz_topic = (
        f'/world/{gz_world_name}/model/rexrov2/link/rexrov2/sonar_link/'
        'sensor/blueview_p900_sensor/scan/points'
    )

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
        SetEnvironmentVariable('PYTHONDONTWRITEBYTECODE', '1'),
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

        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_desc, 'launch', 'upload_rexrov2.launch.py')),
                    launch_arguments={
                        'namespace': 'rexrov2',
                        'hover_mode': 'true',
                        'green_wake': green_wake,
                        'use_velocity_control_plugin': use_velocity_control_plugin,
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
                        'sonar_image_raw_topic': 'rexrov2/blueview_p900/sonar_image_raw_nps_disabled',
                        'plotScaler': '1',
                        'sensorGain': '0.04',
                        'ray_visual': 'true',
                        'writeLog': 'true',
                        'writeFrameInterval': '5',
                    }.items(),
                ),
            ],
        ),

        *_camera_follow_actions(),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='eroas_gz_bridge',
            arguments=[
                f'/world/{gz_world_name}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/rexrov2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/rexrov2/sonar_joint_position_controller/command'
                '@std_msgs/msg/Float64]gz.msgs.Double',
                '/model/rexrov2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/rexrov2/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                f'{sonar_points_gz_topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                f'/world/{gz_world_name}/create@ros_gz_interfaces/srv/SpawnEntity',
                # gz-sim's own VisualizeLidar GUI plugin has no SDF-configurable
                # default topic (see obstacle_avoidance.world's <gui> comment) --
                # it only shows rays after a manual per-launch click. Bridging the
                # ray-visual sensor's LaserScan to ROS 2 lets RViz2 display it
                # instead, pre-configured and already enabled (navigator_auv/rviz/
                # sonar_rays.rviz), so no GUI interaction is required.
                '/rexrov2/sonar_visual_rays@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            ],
            remappings=[
                (f'/world/{gz_world_name}/clock', '/clock'),
                ('/model/rexrov2/odometry', '/rexrov2/pose_gt'),
                (sonar_points_gz_topic, '/rexrov2/blueview_p900_point_cloud'),
            ],
            parameters=[{'use_sim_time': False}],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_control, 'launch', 'start_velocity_control.launch.py')),
            launch_arguments={'uuv_name': 'rexrov2'}.items(),
            condition=IfCondition(LaunchConfiguration('start_pid_thrusters')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_control, 'launch', 'start_pid_controller.launch.py')),
            launch_arguments={'uuv_name': 'rexrov2', 'model_name': 'rexrov2'}.items(),
            condition=IfCondition(LaunchConfiguration('start_pid_controller')),
        ),

        Node(
            package='navigator_auv',
            executable='sonar_frame_converter.py',
            name='sonar_frame_converter',
            output='screen',
        ),

        Node(
            package='navigator_auv',
            executable='only_gap.py',
            name='sonar_heading_node',
            parameters=[{
                'use_sim_time': False,
                'waypoints': waypoints,
                'cmd_vel_topic': '/rexrov2/cmd_vel_1',
                'pose_topic': '/rexrov2/pose_gt',
                'sonar_topic': '/rexrov2/blueview_p900/sonar_image_raw',
                'point_cloud_topic': '/rexrov2/blueview_p900_point_cloud',
                'use_raw_sonar': bool(cfg.get('use_raw_sonar', False)),
                'use_point_cloud_sonar': bool(cfg.get('use_point_cloud_sonar', True)),
                'prefer_point_cloud_sonar': bool(cfg.get('prefer_point_cloud_sonar', True)),
                'loop_waypoints': bool(cfg.get('loop_waypoints', False)),
                'cruise_speed': cfg.get('cruise_speed', 0.38),
                'fallback_speed': cfg.get('fallback_speed', cfg.get('cruise_speed', 0.38)),
                'hard_stop_distance': cfg.get('hard_stop_distance', 0.9),
                'collision_distance': cfg.get('collision_distance', 2.2),
                'waypoint_tolerance': cfg.get('waypoint_tolerance', 5.0),
                'final_waypoint_tolerance': cfg.get('final_waypoint_tolerance', 5.0),
                'planar_waypoint_tolerance': bool(cfg.get('planar_waypoint_tolerance', False)),
                'min_forward_speed': cfg.get('min_forward_speed', 0.18),
                'max_vertical_speed': cfg.get('max_vertical_speed', 0.16),
                'scan_lateral_speed': cfg.get('scan_lateral_speed', 0.18),
                'scan_yaw_rate': cfg.get('scan_yaw_rate', 0.35),
                'yaw_kp': cfg.get('yaw_kp', 0.70),
                'max_yaw_rate': cfg.get('max_yaw_rate', 0.50),
                'max_yaw_delta': cfg.get('max_yaw_delta', 0.06),
                'max_speed_delta': cfg.get('max_speed_delta', 0.10),
                'recovery_lateral_speed': cfg.get('recovery_lateral_speed', 0.30),
                'recovery_yaw_bias': cfg.get('recovery_yaw_bias', 0.12),
                'recovery_speed_threshold': 0.06,
                'recovery_timeout': 3.0,
                'progress_recovery_distance': 0.25,
                'progress_recovery_timeout': 3.0,
                'progress_recovery_release_distance': 0.55,
                'fallback_yaw_kp': cfg.get('fallback_yaw_kp', 0.55),
                'fallback_max_yaw_rate': cfg.get('fallback_max_yaw_rate', 0.35),
                'fallback_lateral_gain': cfg.get('fallback_lateral_gain', 0.75),
                'fallback_min_forward_fraction': cfg.get('fallback_min_forward_fraction', 0.25),
                'startup_hover_duration': cfg.get('startup_hover_duration', 0.0),
                'startup_sensor_wait_timeout': cfg.get('startup_sensor_wait_timeout', 0.0),
                'pivot_min_angle': cfg.get('pivot_min_angle', -0.8),
                'pivot_max_angle': cfg.get('pivot_max_angle', 0.8),
                'pivot_sample_count': cfg.get('pivot_sample_count', 81),
                'pivot_sample_timeout': cfg.get('pivot_sample_timeout', 0.3),
                'pivot_sample_retries': cfg.get('pivot_sample_retries', 3),
                'vertical_gap_run_length': cfg.get('vertical_gap_run_length', 30),
                'vertical_gap_safety_margin_rad': cfg.get('vertical_gap_safety_margin_rad', 0.0),
                'vertical_escape_duration': cfg.get('vertical_escape_duration', 4.0),
                'vertical_depth_tolerance': cfg.get('vertical_depth_tolerance', 0.5),
                'vertical_escape_min_duration': cfg.get('vertical_escape_min_duration', 0.0),
                'vertical_escape_min_planar_distance': cfg.get('vertical_escape_min_planar_distance', 0.0),
                'post_vertical_resume_duration': cfg.get('post_vertical_resume_duration', 35.0),
                'vertical_detection_threshold': cfg.get('vertical_detection_threshold', 15.0),
                'paper_controller': bool(cfg.get('paper_controller', False)),
                'paper_k_t': cfg.get('paper_k_t', 0.12),
                'paper_k_v': cfg.get('paper_k_v', 0.35),
                'paper_psi_max': cfg.get('paper_psi_max', 1.5707963267948966),
                'paper_vx_max': cfg.get('paper_vx_max', 1.0),
                'paper_max_yaw_rate': cfg.get('paper_max_yaw_rate', 0.26),
                'paper_convexity_threshold': cfg.get('paper_convexity_threshold', 0.02),
            }],
            output='screen',
            condition=IfCondition(_enabled_and_not_waypoint_hover('start_navigator')),
        ),

        # Hover hold owns /rexrov2/cmd_vel directly, so it must be mutually
        # exclusive with the navigator/CBF stack.
        Node(
            package='navigator_auv',
            executable='hover_hold.py',
            name='rexrov2_hover_hold',
            parameters=[{
                'use_sim_time': False,
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
                'use_sim_time': False,
                'paper_controller': bool(cfg.get('paper_controller', False)),
                'target_depth': float(cfg['target_depth']),
                'depth_hold_kp': cfg.get('depth_hold_kp', 0.14),
                'depth_deadband': cfg.get('depth_deadband', 0.12),
                'max_vertical_speed': cfg.get('max_vertical_speed', 0.16),
                'max_vertical_delta': cfg.get('max_vertical_delta', 0.025),
                'use_point_cloud_obstacles': bool(cfg.get('use_point_cloud_obstacles', True)),
                'use_analytical_obstacles': bool(cfg.get('use_analytical_obstacles', True)),
                'point_cloud_points_are_local': True,
                'cbf_influence_distance': 3.0,
                'cbf_blend_distance': 1.4,
                # CBF constraint is hdot >= -kappa*(h - min_clearance). At a
                # typical ~2.5m standoff from a 2m safety radius (h~2,
                # gradient magnitude ~2*distance~5), kappa=0.10 only permits
                # ~0.03 m/s of closing velocity before throttling -- so the
                # vehicle crawled to a near-stop whenever any part of its
                # path pointed toward a nearby obstacle, even far from the
                # true safety boundary. Raised so normal cruise speeds are
                # permitted until much closer to the boundary.
                'cbf_gain_xy': cfg.get('cbf_gain_xy', 0.5),
                'cbf_gain_xz': cfg.get('cbf_gain_xz', 0.6),
                'max_active_constraints': cfg.get('max_active_constraints', 6),
                'max_xy_speed': cfg.get('cruise_speed', 0.38),
                'max_xy_delta': cfg.get('max_xy_delta', 0.06),
                'safety_radius_xy': cfg.get('safety_radius_xy', 1.05),
                'safety_radius_xz': cfg.get('safety_radius_xz', 1.00),
                'safety_radius_sonar_pivot': cfg.get('safety_radius_sonar_pivot', 1.05),
                'minimum_clearance': 0.35,
                'imminent_collision_distance': 1.05,
                'collision_stop_distance': 0.55,
                'hold_depth_without_planner': True,
                'point_cloud_timeout': 0.8,
                'point_cloud_min_range': cfg.get('point_cloud_min_range', 0.75),
                'point_cloud_body_clearance_x': cfg.get('point_cloud_body_clearance_x', 0.60),
                'point_cloud_body_clearance_y': cfg.get('point_cloud_body_clearance_y', 1.20),
                'point_cloud_body_clearance_z': cfg.get('point_cloud_body_clearance_z', 1.00),
                'point_cloud_max_abs_z': cfg.get('point_cloud_max_abs_z', 0.0),
                'spatial_memory_timeout': cfg.get('spatial_memory_timeout', 8.0),
                'spatial_memory_radius': cfg.get('spatial_memory_radius', 15.0),
                'spatial_memory_voxel_size': cfg.get('spatial_memory_voxel_size', 0.35),
                'spatial_memory_max_points': cfg.get('spatial_memory_max_points', 3500),
                'min_avoid_forward_speed': cfg.get('min_forward_speed', 0.12),
                'max_reverse_speed': 0.18,
                'reverse_allowed_distance': 1.35,
                'planner_cmd_timeout': 1.0,
                'cbf_stall_speed_threshold': 0.06,
                'recovery_trigger_seconds': cfg.get('recovery_trigger_seconds', 3.0),
                'recovery_forward_speed': cfg.get('recovery_forward_speed', 0.18),
                'recovery_side_step_speed': cfg.get('recovery_side_step_speed', 0.12),
                'recovery_yaw_rate': cfg.get('recovery_yaw_rate', 0.28),
                'recovery_gap_fov_deg': 180.0,
                'recovery_heading_kp': 1.0,
                'recovery_heading_hold_seconds': 2.5,
                'recovery_heading_deadband': 0.08,
                'safety_escape_speed': cfg.get('safety_escape_speed', 0.24),
                'barrier_slide_speed': cfg.get('barrier_slide_speed', 0.22),
                'barrier_slide_away_weight': 0.70,
                'barrier_slide_trigger_h': cfg.get('barrier_slide_trigger_h', 1.20),
                'free_space_unstick_enabled': True,
                'free_space_unstick_timeout': 2.5,
                'free_space_unstick_duration': 3.0,
                'free_space_unstick_speed': cfg.get('free_space_unstick_speed', 0.30),
                'hover_lock_enabled': bool(cfg.get('hover_lock_enabled', False)),
                'hover_lock_distance': 5.0,
                'hover_lock_release_distance': 6.0,
                'hover_lock_lateral_min': 0.35,
                'hover_lock_squeeze_seconds': 1.0,
                'hover_lock_min_seconds': 4.0,
                'hover_lock_cooldown_seconds': 1.0,
                'hover_lock_oscillation_threshold': 2.0,
                'hover_lock_oscillation_decay': 0.82,
                'hover_lock_min_lateral_cmd': 0.05,
                'hover_lock_min_yaw_cmd': 0.08,
            }],
            output='screen',
            condition=IfCondition(_enabled_and_not_waypoint_hover('start_cbf')),
        ),

        Node(
            package='navigator_auv',
            executable='waypoint_hover.py',
            name='waypoint_hover',
            parameters=[{
                'use_sim_time': False,
                'waypoints': waypoints,
            }],
            output='screen',
            condition=IfCondition(_enabled_and_not_hover('start_waypoint_hover')),
        ),

        Node(
            package='navigator_auv',
            executable='sonar_ray_markers.py',
            name='sonar_ray_markers',
            output='screen',
        ),

        Node(
            package='navigator_auv',
            executable='sonar_reconstruction.py',
            name='sonar_reconstruction',
            parameters=[{'use_sim_time': False}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_sonar_reconstruction')),
        ),

        *([
            Node(
                package='navigator_auv',
                executable='trail_marker.py',
                name='rexrov2_green_trail',
                parameters=[{
                    'use_sim_time': False,
                    'pose_topic': '/rexrov2/pose_gt',
                    'marker_topic': '/rexrov2/trail_marker',
                    'frame_id': 'world',
                    'max_points': 45,
                    'max_length': 10.0,
                    'min_distance': 0.18,
                    'line_width': 0.18,
                    'publish_rate': 12.0,
                }],
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_trail')),
            ),
        ] if world_name == 'world_a' else []),

        *([
            Node(
                package='navigator_auv',
                executable='spawner.py',
                name='eroas_trail_spawner',
                parameters=[{
                    'use_sim_time': False,
                    'world_name': gz_world_name,
                    'pose_topic': '/rexrov2/pose_gt',
                    'distance_threshold': 0.35,
                    'marker_radius': 0.25,
                    'initial_delay': 0.3,
                }],
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_gazebo_trail')),
            ),
        ] if world_name == 'world_a' else []),

        Node(
            package='navigator_auv',
            executable='pose_gt_to_odom.py',
            name='rexrov2_odom_alias',
            parameters=[{
                'use_sim_time': False,
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
        DeclareLaunchArgument('world_name', default_value='blue_blocks',
                              description='World configuration: world_a (paper Figure 8), world_a_figure5 (paper Figure 5 SPD2C path), blue_blocks, world_b'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x', default_value='auto'),
        DeclareLaunchArgument('y', default_value='auto'),
        DeclareLaunchArgument('z', default_value='auto'),
        DeclareLaunchArgument('yaw', default_value='auto'),
        DeclareLaunchArgument('waypoints', default_value='auto',
                              description='Semicolon-separated x,y,z waypoint list'),
        DeclareLaunchArgument('start_navigator', default_value='true'),
        DeclareLaunchArgument('start_hover_hold', default_value='false'),
        DeclareLaunchArgument('start_cbf', default_value='true'),
        DeclareLaunchArgument('start_pid_thrusters', default_value='false'),
        DeclareLaunchArgument('start_pid_controller', default_value='false'),
        DeclareLaunchArgument('start_sonar_reconstruction', default_value='false'),
        DeclareLaunchArgument('start_trail', default_value='true'),
        DeclareLaunchArgument('start_gazebo_trail', default_value='false'),
        DeclareLaunchArgument('green_wake', default_value='auto'),
        # Keep the GUI camera fixed by default so vehicle motion is visible.
        # Pass auto_follow:=true when a chase camera is desired.
        DeclareLaunchArgument('auto_follow', default_value='false'),
        DeclareLaunchArgument('show_sonar_probe', default_value='false'),
        DeclareLaunchArgument('start_waypoint_hover', default_value='false',
                              description='Launch waypoint_hover standalone controller (disables navigator and CBF)'),
        OpaqueFunction(function=_setup),
    ])
