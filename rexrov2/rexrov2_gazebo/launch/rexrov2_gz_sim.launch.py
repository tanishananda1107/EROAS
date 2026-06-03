import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
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
    world_pkg = get_package_share_directory("uuv_gazebo_worlds")
    sonar_pkg = get_package_share_directory("nps_uw_multibeam_sonar")

    world_file = os.path.join(sim_pkg, "worlds", "coral_reef_world.sdf")
    rexrov2_sdf = os.path.join(sim_pkg, "models", "rexrov2_visible", "model.sdf")
    model_paths = [
        sim_pkg,
        desc_pkg,
        os.path.dirname(desc_pkg),
        os.path.join(world_pkg, "models"),
        os.path.join(world_pkg, "models", "nerf"),
        os.path.join(sonar_pkg, "models"),
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    ]
    plugin_paths = [
        os.path.join(get_package_prefix("uuv_gazebo_ros_plugins"), "lib"),
        os.path.join(get_package_prefix("uuv_world_ros_plugins"), "lib"),
        os.path.join(get_package_prefix("nps_uw_multibeam_sonar"), "lib"),
        os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
    ]
    resource_path = _join_paths(model_paths)

    return LaunchDescription([
        *[SetEnvironmentVariable(name, "") for name in SNAP_ENV_VARS],

        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="-5.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),

        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", _join_paths(plugin_paths)),

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
            name="gz_clock_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),

        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_rexrov2",
            arguments=[
                "-name",
                "rexrov2",
                "-file",
                rexrov2_sdf,
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
        ),
    ])
