#!/usr/bin/env python3

# Copyright (c) 2016 The UUV Simulator Authors.
# ROS2 Jazzy conversion

import unittest
import subprocess
import os
from pathlib import Path


PKG = "uuv_descriptions"
NAME = "test_urdf_files"


def call_xacro(xml_file):
    xml_file = Path(xml_file)

    assert xml_file.exists(), \
        f"Invalid XML xacro file: {xml_file}"

    result = subprocess.run(
        [
            "xacro",
            str(xml_file)
        ],
        capture_output=True,
        text=True
    )

    return result


class TestRexROVURDFFiles(unittest.TestCase):

    def test_xacro(self):

        test_dir = Path(
            os.path.dirname(
                os.path.abspath(__file__)
            )
        )

        robots_dir = (
            test_dir.parent /
            "robots"
        )

        self.assertTrue(
            robots_dir.exists(),
            f"Robots directory missing: {robots_dir}"
        )

        for item in robots_dir.iterdir():

            if "oberon" in item.name:
                continue

            if not item.is_file():
                continue

            result = call_xacro(item)

            self.assertEqual(
                result.returncode,
                0,
                f"""
Failed parsing:

File:
{item.name}

STDERR:
{result.stderr}

STDOUT:
{result.stdout}
"""
            )

            self.assertNotIn(
                "XML parsing error",
                result.stderr,
                f"XML parsing error in {item.name}"
            )

            self.assertNotIn(
                "No such file or directory",
                result.stderr,
                f"Missing dependency in {item.name}"
            )


if __name__ == "__main__":
    unittest.main()

