#!/usr/bin/env python3

import os
import subprocess
import unittest
from ament_index_python.packages import get_package_share_directory


def call_xacro(file_path):
    assert os.path.isfile(file_path), f"Invalid xacro file: {file_path}"

    return subprocess.check_output(
        ['xacro', file_path],
        text=True
    )


class TestRexROVURDFFiles(unittest.TestCase):

    def test_xacro_files(self):
        pkg_path = get_package_share_directory('rexrov2_description')
        robots_dir = os.path.join(pkg_path, 'robots')

        self.assertTrue(os.path.isdir(robots_dir), "robots directory missing")

        for item in os.listdir(robots_dir):
            if 'oberon' in item:
                continue

            file_path = os.path.join(robots_dir, item)

            if not os.path.isfile(file_path):
                continue

            try:
                output = call_xacro(file_path)

                self.assertNotIn(
                    "XML parsing error",
                    output,
                    f"Parsing error found in {item}"
                )

                self.assertNotIn(
                    "No such file or directory",
                    output,
                    f"Missing file reference in {item}"
                )

            except subprocess.CalledProcessError as e:
                self.fail(f"xacro failed for {item}: {e}")


if __name__ == '__main__':
    import pytest
    pytest.main()
