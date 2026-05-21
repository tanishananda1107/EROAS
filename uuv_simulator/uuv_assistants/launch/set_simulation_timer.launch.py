#!/usr/bin/env python3
"""
ROS2 conversion of set_simulation_timer.launch

ROS1 original:
  <arg name="timeout" />                   ← required, no default
  <node name="simulation_timeout"
        pkg="uuv_assistants"
        type="set_simulation_timer.py"
        required="true"                    ← shuts entire launch on exit
        output="screen">
    <rosparam subst_value="true">
      timeout: $(arg timeout)
    </rosparam>
  </node>

ROS2 note:
  'required="true"' has no direct attribute in ROS2 Node().
  It is replicated with RegisterEventHandler + OnProcessExit -> Shutdown().
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    simulation_timeout_node = Node(
        package='uuv_assistants',
        executable='set_simulation_timer.py',   # = type= in ROS1
        name='simulation_timeout',
        output='screen',
        parameters=[{                            # = <rosparam>
            'timeout': LaunchConfiguration('timeout'),
        }],
    )

    return LaunchDescription([

        # ── mirrors <arg name="timeout" /> (required, no default) ────────────
        DeclareLaunchArgument('timeout',
                              description='Simulation timeout in seconds (required)'),

        simulation_timeout_node,

        # ── mirrors required="true": tear down launch when node exits ─────────
        RegisterEventHandler(
            OnProcessExit(
                target_action=simulation_timeout_node,
                on_exit=[Shutdown()],
            )
        ),
    ])
