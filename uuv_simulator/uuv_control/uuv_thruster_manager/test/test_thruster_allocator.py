
#!/usr/bin/env python3
import unittest
import numpy as np
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from uuv_thruster_manager.srv import (
    GetThrusterManagerConfig,
    ThrusterManagerInfo
)

NS = 'test_vehicle'

AXIS_X_TAM = np.array([
    [1, 0, 0, 0, 0, 0],
    [0.87758256, 0, -0.47942554, 0.47942554, 0.47942554, 0.87758256],
    [0.87758256, 0.47942554, 0, -0.47942554, 0.87758256, -0.87758256]
]).T

AXIS_Y_TAM = np.array([
    [0, 0.87758256, 0.47942554, 0, 0.47942554, -0.87758256],
    [0, 1, 0, 0, 0, 1],
    [-0.47942554, 0.87758256, 0, -0.87758256, -0.47942554, 0.47942554]
]).T

AXIS_Z_TAM = np.array([
    [0, -0.47942554, 0.87758256, 0, 0.87758256, 0.47942554],
    [0.47942554, 0, 0.87758256, -0.87758256, -0.87758256, 0.47942554],
    [0., 0., 1., 1., 0., 0.]
]).T


class TestThrusterAllocator(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_thruster_allocator')
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_services_exist(self):

        services = [
            '/test_vehicle/thruster_manager/get_thrusters_info',
            '/test_vehicle/thruster_manager/get_thruster_curve',
            '/test_vehicle/thruster_manager/set_config',
            '/test_vehicle/thruster_manager/get_config'
        ]

        for srv_name in services:
            client = self.node.create_client(
                GetThrusterManagerConfig,
                srv_name
            )

            available = client.wait_for_service(timeout_sec=10.0)

            self.assertTrue(
                available,
                msg=f'Service not available: {srv_name}'
            )

    def test_config(self):

        client = self.node.create_client(
            GetThrusterManagerConfig,
            '/test_vehicle/thruster_manager/get_config'
        )

        self.assertTrue(client.wait_for_service(timeout_sec=10.0))

        request = GetThrusterManagerConfig.Request()

        future = client.call_async(request)

        rclpy.spin_until_future_complete(self.node, future)

        tm_config = future.result()

        self.assertIsNotNone(tm_config)

        self.assertEqual(tm_config.tf_prefix, '/test_vehicle/')
        self.assertEqual(tm_config.base_link, 'base_link')
        self.assertEqual(tm_config.thruster_frame_base, 'thruster_')
        self.assertEqual(tm_config.thruster_topic_suffix, '/input')
        self.assertEqual(tm_config.timeout, -1.0)
        self.assertEqual(tm_config.max_thrust, 1000.0)
        self.assertEqual(tm_config.n_thrusters, 3)

        tam_flat = AXIS_X_TAM.flatten()

        self.assertEqual(
            len(tm_config.allocation_matrix),
            tam_flat.size
        )

        for x, y in zip(tam_flat, tm_config.allocation_matrix):
            self.assertAlmostEqual(x, y)


if __name__ == '__main__':
    unittest.main()

