from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    gz_pkg = FindPackageShare("ros_gz_sim").find("ros_gz_sim")
    world_pkg = FindPackageShare("uuv_gazebo_worlds").find("uuv_gazebo_worlds")

    world_file = os.path.join(world_pkg, "worlds", "empty_underwater.sdf")

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_pkg, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": f"-r {world_file}"
            }.items()
        ),

        Node(
            package="uuv_assistants",
            executable="publish_world_ned_frame",
            output="screen"
        ),

        Node(
            package="uuv_assistants",
            executable="publish_world_models",
            output="screen",
            parameters=[{
                "meshes": {
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
