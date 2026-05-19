
#!/usr/bin/env python3

import sys
import unittest
import numpy as np
from uuv_trajectory_generator import TrajectoryPoint
from uuv_control_msgs.msg import TrajectoryPoint as TrajectoryPointMsg
import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TestTrajectoryPoint(Node):
    def __init__(self):
        super().__init__('test_trajectory_point')
        self.create_subscription(TrajectoryPointMsg, 'trajectory_point', se[2D[K
self.trajectory_point_callback)
        self.create_service(TrajectoryPointMsg.Request, 'get_trajectory_poi[19D[K
'get_trajectory_point', self.get_trajectory_point_service)

    def trajectory_point_callback(self, msg):
        # TO DO: implement callback logic
        pass

    def get_trajectory_point_service(self, request_response):
        # TO DO: implement service logic
        pass

    def test_init_pos_vector(self):
        p = TrajectoryPoint()
        self.assertEqual(p.pos.size, 3, 'Position vector len() is incorrect[9D[K
incorrect')
        self.assertTrue(np.array_equal(p.pos, [0, 0, 0]), 'Position initial[7D[K
initialization failed')

    def test_set_pos_vector(self):
        p = TrajectoryPoint()
        p.pos = [1, 2, 3]
        self.assertEqual(p.pos[0], 1, 'X position was initialized wrong')
        self.assertEqual(p.pos[1], 2, 'Y position was initialized wrong')
        self.assertEqual(p.pos[2], 3, 'Z position was initialized wrong')

    def test_init_quat_vector(self):
        p = TrajectoryPoint()
        self.assertEqual(p.rotq.size, 4, 'Quaternion vector len() is incorr[6D[K
incorrect')
        self.assertTrue(np.array_equal(p.rotq, [0, 0, 0, 1]), 'Quaternion i[1D[K
initialization failed')

    def test_to_message(self):
        p0 = TrajectoryPoint()
        p0.t = 1
        p0.pos = [1, 2, 3]
        p0.rotq = [0, 0, 1, 1]
        p0.vel = [1, 2, 3, 4, 5, 6]
        p0.acc = [1, 2, 3, 4, 5, 6]

        p1 = TrajectoryPoint()
        p1.from_message(p0.to_message())

        self.assertEqual(p0, p1, 'Point to message conversion failed')

    def test_to_dict(self):
        p0 = TrajectoryPoint()
        p0.t = 1
        p0.pos = [1, 2, 3]
        p0.rotq = [0, 0, 1, 1]
        p0.vel = [1, 2, 3, 4, 5, 6]
        p0.acc = [1, 2, 3, 4, 5, 6]

        p1 = TrajectoryPoint()
        p1.from_dict(p0.to_dict())

        self.assertEqual(p0, p1, 'Point to dict conversion failed')

if __name__ == '__main__':
    import os
    import sys
    import unittest

    # Add the package path to the system path
    pkg_path = '/path/to/uuv_trajectory_control'  # Replace with actual pat[3D[K
path
    if pkg_path not in sys.path:
        sys.path.append(pkg_path)

    rclpy.init()
    node = TestTrajectoryPoint()
    node.get_clock().now()
    rclpy.shutdown()

    unittest.main()

Note that I removed the `catkin_python_setup()` and `install(PROGRAMS ...)`[5D[K
...)` directives as they are no longer necessary in ROS2.

