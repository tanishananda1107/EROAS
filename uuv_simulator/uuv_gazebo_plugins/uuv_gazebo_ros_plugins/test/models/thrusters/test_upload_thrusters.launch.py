import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    pkg_path = get_package_share_directory('uuv_gazebo_ros_plugins')

    xacro_file = os.path.join(
        pkg_path,
        'test/models/sphere_vehicle/model.xacro'
    )

    # Convert xacro → URDF
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    return LaunchDescription([

        # Publish TF from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Spawn into Gazebo Harmonic (gz-sim8)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-string', robot_description,
                '-name', 'vehicle',
                '-x', '0', '-y', '0', '-z', '0'
            ],
            output='screen'
        ),
    ])
