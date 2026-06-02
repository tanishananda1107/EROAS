from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    gz_pkg = FindPackageShare("ros_gz_sim").find("ros_gz_sim")
    world_pkg = FindPackageShare("uuv_gazebo_worlds").find("uuv_gazebo_worlds")

    world_file = os.path.join(world_pkg, "worlds", "empty_underwater.world")

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_pkg, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": f"-r {world_file}"}.items()
        ),

        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-file", "rov_bop_panel.sdf",
                "-name", "rov_bop_panel",
                "-x", "-1",
                "-y", "-1",
                "-z", "-4",
                "-R", "0",
                "-P", "1.57",
                "-Y", "0"
            ],
            output="screen"
        )
    ])
