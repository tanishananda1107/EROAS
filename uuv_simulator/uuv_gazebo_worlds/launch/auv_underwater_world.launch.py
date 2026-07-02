from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    gui = LaunchConfiguration("gui")
    paused = LaunchConfiguration("paused")

    gz_pkg = FindPackageShare("ros_gz_sim").find("ros_gz_sim")
    world_pkg = FindPackageShare("uuv_gazebo_worlds").find("uuv_gazebo_worlds")

    world_file = os.path.join(world_pkg, "worlds", "auv_underwater_world.world")

    return LaunchDescription([

        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("paused", default_value="false"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_pkg, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": f"-r {world_file} --gui={gui}"
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
                    "sea_surface": {
                        "mesh": "package://uuv_gazebo_worlds/models/sea_surface_1000m_x_1000m.dae",
                        "scale": [2, 2, 1]
                    },
                    "sea_bottom": {
                        "plane": [2000, 2000, 0.1],
                        "pose": {"position": [0, 0, -80]}
                    }
                }
            }]
        )
    ])
