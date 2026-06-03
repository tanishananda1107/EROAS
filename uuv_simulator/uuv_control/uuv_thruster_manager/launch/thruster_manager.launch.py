#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    model_name = LaunchConfiguration('model_name')
    uuv_name = LaunchConfiguration('uuv_name')
    base_link = LaunchConfiguration('base_link')
    timeout = LaunchConfiguration('timeout')
    reset_tam = LaunchConfiguration('reset_tam')

    output_dir = LaunchConfiguration('output_dir')
    config_file = LaunchConfiguration('config_file')
    tam_file = LaunchConfiguration('tam_file')

    return LaunchDescription([

        DeclareLaunchArgument(
            'model_name'
        ),

        DeclareLaunchArgument(
            'uuv_name',
            default_value=model_name
        ),

        DeclareLaunchArgument(
            'base_link',
            default_value='base_link'
        ),

        DeclareLaunchArgument(
            'timeout',
            default_value='-1.0'
        ),

        DeclareLaunchArgument(
            'reset_tam',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'output_dir',
            default_value=PathJoinSubstitution([
                FindPackageShare(
                    'uuv_thruster_manager'
                ),
                'config',
                model_name
            ])
        ),

        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare(
                    'uuv_thruster_manager'
                ),
                'config',
                model_name,
                'thruster_manager.yaml'
            ])
        ),

        DeclareLaunchArgument(
            'tam_file',
            default_value=PathJoinSubstitution([
                FindPackageShare(
                    'uuv_thruster_manager'
                ),
                'config',
                model_name,
                'TAM.yaml'
            ])
        ),

        Node(
            package='uuv_thruster_manager',

            executable='thruster_allocator.py',

            name='thruster_allocator',

            namespace=uuv_name,

            output='screen',

            parameters=[
                config_file,

                {
                    'thruster_manager.tf_prefix':
                    uuv_name,

                    'thruster_manager.timeout':
                    timeout,

                    'output_dir':
                    output_dir
                }
            ],

            condition=IfCondition(
                reset_tam
            )
        ),

        Node(
            package='uuv_thruster_manager',

            executable='thruster_allocator.py',

            name='thruster_allocator',

            namespace=uuv_name,

            output='screen',

            parameters=[
                config_file,

                tam_file,

                {
                    'thruster_manager.tf_prefix':
                    uuv_name,

                    'thruster_manager.timeout':
                    timeout
                }
            ],

            condition=UnlessCondition(
                reset_tam
            )
        )
    ])
