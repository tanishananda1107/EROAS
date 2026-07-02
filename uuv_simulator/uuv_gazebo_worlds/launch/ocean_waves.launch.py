from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


BLUE_BLOCK_WORLD = "obstacle_avoidance.world"


def generate_launch_description():

    gz_pkg = FindPackageShare("ros_gz_sim").find("ros_gz_sim")
    world_pkg = FindPackageShare("uuv_gazebo_worlds").find("uuv_gazebo_worlds")

    world_file = os.path.join(world_pkg, "worlds", BLUE_BLOCK_WORLD)

    return LaunchDescription([
        DeclareLaunchArgument(
            "gz_args",
            default_value=f"--physics-engine gz-physics-bullet-featherstone-plugin -r {world_file}",
            description="Gazebo arguments; defaults to the Blue Block World",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_pkg, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items()
        ),

        Node(
            package="uuv_assistants",
            executable="publish_world_ned_frame"
        ),

        Node(
            package="uuv_assistants",
            executable="publish_world_models",
            parameters=[{
                "meshes": {
                    "heightmap": {
                        "mesh": "package://uuv_gazebo_worlds/models/sand_heightmap/meshes/heightmap.dae"
                    },
                    "seafloor": {
                        "plane": [2000, 2000, 0.1],
                        "pose": {"position": [0, 0, -100]}
                    },
                    "north": {"plane": [0.1, 2000, 100], "pose": {"position": [1000, 0, -50]}},
                    "south": {"plane": [0.1, 2000, 100], "pose": {"position": [-1000, 0, -50]}},
                    "west":  {"plane": [2000, 0.1, 100], "pose": {"position": [0, -1000, -50]}},
                    "east":  {"plane": [2000, 0.1, 100], "pose": {"position": [0, 1000, -50]}}
                }
            }]
        )
    ])
