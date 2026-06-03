import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


SNAP_ENV_VARS = [
    "SNAP",
    "SNAP_NAME",
    "SNAP_REVISION",
    "SNAP_VERSION",
    "SNAP_ARCH",
    "SNAP_INSTANCE_NAME",
    "SNAP_CONTEXT",
    "SNAP_COOKIE",
    "SNAP_DATA",
    "SNAP_COMMON",
    "SNAP_USER_DATA",
    "SNAP_USER_COMMON",
    "SNAP_REAL_HOME",
    "SNAP_LIBRARY_PATH",
    "SNAP_LAUNCHER_ARCH_TRIPLET",
    "SNAP_UID",
    "SNAP_EUID",
    "GTK_PATH",
    "LOCPATH",
    "FONTCONFIG_PATH",
]


def _join_paths(paths):
    return os.pathsep.join(path for path in paths if path)


def generate_launch_description():
    gz_pkg = get_package_share_directory("ros_gz_sim")
    sim_pkg = get_package_share_directory("rexrov2_gazebo")
    desc_pkg = get_package_share_directory("rexrov2_description")
    control_pkg = get_package_share_directory("rexrov2_control")
    world_pkg = get_package_share_directory("uuv_gazebo_worlds")
    sonar_pkg = get_package_share_directory("nps_uw_multibeam_sonar")

    world_file = os.path.join(sim_pkg, "worlds", "coral_reef_ocean.sdf")
    rexrov2_fallback_sdf = os.path.join(sim_pkg, "models", "rexrov2_visible", "model.sdf")

    resource_path = _join_paths([
        sim_pkg,
        os.path.join(sim_pkg, "models"),
        desc_pkg,
        os.path.dirname(desc_pkg),
        os.path.join(world_pkg, "models"),
        os.path.join(world_pkg, "models", "nerf"),
        os.path.join(sonar_pkg, "models"),
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    ])
    plugin_path = _join_paths([
        os.path.join(get_package_prefix("uuv_gazebo_ros_plugins"), "lib"),
        os.path.join(get_package_prefix("uuv_world_ros_plugins"), "lib"),
        os.path.join(get_package_prefix("uuv_sensor_ros_plugins"), "lib"),
        os.path.join(get_package_prefix("nps_uw_multibeam_sonar"), "lib"),
        os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
    ])

    return LaunchDescription([
        *[SetEnvironmentVariable(name, "") for name in SNAP_ENV_VARS],

        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("x", default_value="0"),
        DeclareLaunchArgument("y", default_value="0"),
        DeclareLaunchArgument("z", default_value="-3"),
        DeclareLaunchArgument("yaw", default_value="0"),
        DeclareLaunchArgument("use_full_rexrov2", default_value="false"),
        DeclareLaunchArgument("spawn_fallback_rexrov2", default_value="true"),
        DeclareLaunchArgument("start_control", default_value="false"),
        DeclareLaunchArgument("start_navigator", default_value="false"),
        DeclareLaunchArgument("demo_motion", default_value="false"),

        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_pkg, "launch", "gz_sim.launch.py")),
            launch_arguments={
                "gz_args": PythonExpression([
                    "'--physics-engine gz-physics-bullet-featherstone-plugin -r " + world_file + "' if '",
                    LaunchConfiguration("gui"),
                    "' == 'true' else '-s --physics-engine gz-physics-bullet-featherstone-plugin -r " + world_file + "'",
                ]),
            }.items(),
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="coral_reef_gz_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(desc_pkg, "launch", "upload_rexrov2.launch.py")),
            launch_arguments={
                "namespace": "rexrov2",
                "x": LaunchConfiguration("x"),
                "y": LaunchConfiguration("y"),
                "z": LaunchConfiguration("z"),
                "yaw": LaunchConfiguration("yaw"),
                "sonar_name": "blueview_p900",
                "gpu_ray": "true",
                "maxDistance": "30",
                "fidelity": "500",
                "raySkips": "1",
                "sonar_image_topic": "sonar_image",
                "sonar_image_raw_topic": "sonar_image_raw",
                "plotScaler": "1",
                "sensorGain": "0.04",
                "ray_visual": "true",
                "writeLog": "false",
                "writeFrameInterval": "5",
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_full_rexrov2")),
        ),

        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_rexrov2_fallback",
            arguments=[
                "-name",
                "rexrov2",
                "-file",
                rexrov2_fallback_sdf,
                "-x",
                LaunchConfiguration("x"),
                "-y",
                LaunchConfiguration("y"),
                "-z",
                LaunchConfiguration("z"),
                "-Y",
                LaunchConfiguration("yaw"),
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("spawn_fallback_rexrov2")),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(control_pkg, "launch", "start_pid_controller.launch.py")),
            launch_arguments={
                "uuv_name": "rexrov2",
                "model_name": "rexrov2",
            }.items(),
            condition=IfCondition(LaunchConfiguration("start_control")),
        ),

        Node(
            package="navigator_auv",
            executable="sonar_reconstruction.py",
            name="sonar_reconstruction",
            parameters=[{"use_sim_time": True}],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_navigator")),
        ),
        Node(
            package="navigator_auv",
            executable="just_gap.py",
            name="sonar_heading_node",
            parameters=[{"use_sim_time": True}],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_navigator")),
        ),
        Node(
            package="navigator_auv",
            executable="velocity_cbf.py",
            name="obstacle_avoidance_node",
            parameters=[{"use_sim_time": True}],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_navigator")),
        ),

        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "topic",
                        "pub",
                        "--rate",
                        "5",
                        "/rexrov2/cmd_vel_1",
                        "geometry_msgs/msg/Twist",
                        "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}",
                    ],
                    name="coral_reef_demo_cmd_vel",
                    output="screen",
                    condition=IfCondition(LaunchConfiguration("demo_motion")),
                )
            ],
        ),
    ])
