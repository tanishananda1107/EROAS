# Copyright (c) 2016 The UUV Simulator Authors.
# Licensed under the Apache License, Version 2.0.

import os
import unittest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_launch_description():
    return LaunchDescription([
        launch_testing.actions.ReadyToTest(),
    ])


class TestURDFFiles(unittest.TestCase):
    """Launched as part of the launch test — no simulator needed."""

    def test_xacro_files(self):
        pkg_share = get_package_share_directory('uuv_sensor_ros_plugins')
        urdf_dir  = os.path.join(pkg_share, 'urdf')

        self.assertTrue(
            os.path.isdir(urdf_dir),
            f'urdf directory not found: {urdf_dir}')

        xacro_files = [
            f for f in os.listdir(urdf_dir)
            if os.path.isfile(os.path.join(urdf_dir, f))
            and f.endswith(('.xacro', '.urdf.xacro', '.urdf'))
        ]

        self.assertGreater(
            len(xacro_files), 0,
            f'No xacro/urdf files found in {urdf_dir}')

        for item in xacro_files:
            full_path = os.path.join(urdf_dir, item)
            result = __import__('subprocess').run(
                ['xacro', full_path],
                capture_output=True, text=True)

            self.assertEqual(
                result.returncode, 0,
                f'xacro failed for {item}:\n{result.stderr}')
            self.assertNotIn(
                'XML parsing error', result.stdout + result.stderr,
                f'XML parsing error in {item}')
            self.assertNotIn(
                'No such file or directory', result.stderr,
                f'Missing file referenced in {item}')
