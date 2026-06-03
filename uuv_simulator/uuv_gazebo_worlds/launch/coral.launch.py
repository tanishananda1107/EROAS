#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix


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


def generate_launch_description():
    gz_pkg = FindPackageShare("ros_gz_sim").find("ros_gz_sim")
    world_pkg = FindPackageShare("uuv_gazebo_worlds").find("uuv_gazebo_worlds")
    sonar_pkg = FindPackageShare("nps_uw_multibeam_sonar").find("nps_uw_multibeam_sonar")
    world_file = os.path.join(world_pkg, "worlds", "coral.world")
    models_dir = os.path.join(world_pkg, "models")
    nested_models_dir = os.path.join(models_dir, "nerf")
    sonar_models_dir = os.path.join(sonar_pkg, "models")
    plugin_paths = [
        os.path.join(get_package_prefix("uuv_gazebo_ros_plugins"), "lib"),
        os.path.join(get_package_prefix("uuv_world_ros_plugins"), "lib"),
        os.path.join(get_package_prefix("nps_uw_multibeam_sonar"), "lib"),
    ]

    return LaunchDescription([
        *[SetEnvironmentVariable(name, "") for name in SNAP_ENV_VARS],

        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Gazebo GUI together with the server.",
        ),

        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            os.pathsep.join([models_dir, nested_models_dir, sonar_models_dir, world_pkg]),
        ),

        SetEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            os.pathsep.join(plugin_paths),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_pkg, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": PythonExpression([
                    "'--physics-engine gz-physics-bullet-featherstone-plugin -r " + world_file + "' if '",
                    LaunchConfiguration("gui"),
                    "' == 'true' else '-s --physics-engine gz-physics-bullet-featherstone-plugin -r " + world_file + "'"
                ])
            }.items(),
        ),

        Node(
            package="uuv_assistants",
            executable="publish_world_ned_frame",
            output="screen",
        ),

        Node(
            package="uuv_assistants",
            executable="publish_world_models",
            name="publish_world_models",
            output="screen",
            parameters=[{
                "meshes": {
                    "heightmap": {
                        "mesh": "package://uuv_gazebo_worlds/models/sand_heightmap/meshes/heightmap.dae",
                        "model": "sand_heightmap",
                    },
                    "seafloor": {
                        "plane": [2000, 2000, 0.1],
                        "pose": {"position": [0, 0, -100]},
                    },
                    "north": {"plane": [0.1, 2000, 100], "pose": {"position": [1000, 0, -50]}},
                    "south": {"plane": [0.1, 2000, 100], "pose": {"position": [-1000, 0, -50]}},
                    "west": {"plane": [2000, 0.1, 100], "pose": {"position": [0, -1000, -50]}},
                    "east": {"plane": [2000, 0.1, 100], "pose": {"position": [0, 1000, -50]}},
                }
            }],
        ),
    ])
