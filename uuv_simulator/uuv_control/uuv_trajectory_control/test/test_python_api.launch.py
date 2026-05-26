#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', 'test/test_waypoint.py'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['python3', 'test/test_trajectory_point.py'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['python3', 'test/test_waypoint_set.py'],
            output='screen'
        )
    ])
