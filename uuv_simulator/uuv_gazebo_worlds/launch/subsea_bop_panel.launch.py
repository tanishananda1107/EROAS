from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    gz_pkg = FindPackageShare("ros_gz_sim").find("ros_gz_sim")
    world_pkg = FindPackageShare("uuv_gazebo_worlds").find("uuv_gazebo_worlds")

    world_file = os.path.join(
        world_pkg,
        "worlds",
        "subsea_bop_panel.sdf"   # recommended (or keep .world if not migrated yet)
    )

    return LaunchDescription([

        # Launch Gazebo Harmonic
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_pkg, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": f"-r {world_file}"
            }.items()
        ),

        # NED frame publisher (ROS2 version of your uuv_assistants node)
        Node(
            package="uuv_assistants",
            executable="publish_world_ned_frame",
            output="screen"
        ),

        # OPTIONAL: world models (currently commented in ROS1, kept optional here)
        # Node(
        #     package="uuv_assistants",
        #     executable="publish_world_models",
        #     output="screen",
        #     parameters=[{
        #         "meshes": {
        #             "herkules_seabed": {
        #                 "mesh": "package://uuv_gazebo_worlds/models/herkules_seabed/meshes/herkules_seabed.dae",
        #                 "pose": {
        #                     "position": [0, 0, -60]
        #                 },
        #                 "scale": [4, 4, 1]
        #             },
        #             "herkules_ship_wreck": {
        #                 "mesh": "package://uuv_gazebo_worlds/models/herkules_ship_wreck/meshes/herkules.dae",
        #                 "pose": {
        #                     "position": [0, 0, -60],
        #                     "orientation": [0, 0, 1.57]
        #                 }
        #             }
        #         }
        #     }]
        # )
    ])
