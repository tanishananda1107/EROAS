#!/usr/bin/env python3
# Copyright (c) 2016 The UUV Simulator Authors.
# Licensed under the Apache License, Version 2.0.

import unittest
import subprocess
import os


def call_xacro(xml_file):
    assert os.path.isfile(xml_file), f'Invalid xacro file: {xml_file}'
    result = subprocess.run(
        ['xacro', xml_file],
        capture_output=True,
        text=True
    )
    return result.stdout, result.stderr, result.returncode


class TestURDFFiles(unittest.TestCase):

    def test_xacro(self):
        test_dir  = os.path.abspath(os.path.dirname(__file__))
        urdf_dir  = os.path.join(test_dir, '..', 'urdf')

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
            stdout, stderr, returncode = call_xacro(full_path)

            self.assertEqual(
                returncode, 0,
                f'xacro failed for {item}:\n{stderr}')

            self.assertNotIn(
                'XML parsing error', stdout,
                f'XML parsing error in {item}')

            self.assertNotIn(
                'XML parsing error', stderr,
                f'XML parsing error in {item}')

            self.assertNotIn(
                'No such file or directory', stderr,
                f'Missing file referenced in {item}')


if __name__ == '__main__':
    unittest.main()
