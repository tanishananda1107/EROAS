#!/usr/bin/env python3
# Copyright (c) 2016 The UUV Simulator Authors.
# Licensed under the Apache License, Version 2.0.

import unittest
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3, Inertia
from uuv_gazebo_ros_plugins_msgs.msg import UnderwaterObjectModel
from uuv_gazebo_ros_plugins_msgs.srv import GetModelProperties

# Sphere model radius
RADIUS = 0.1
# Drag coefficient
CD = 0.5


class TestSphereVehicle(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_sphere_vehicle')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, srv_type, srv_name, request=None):
        client = self.node.create_client(srv_type, srv_name)
        self.assertTrue(
            client.wait_for_service(timeout_sec=10.0),
            f'Service {srv_name} not available')
        req = request if request is not None else srv_type.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertIsNotNone(future.result(), f'No response from {srv_name}')
        return future.result()

    def _get_models(self):
        return self._call(GetModelProperties, '/vehicle/get_model_properties')

    def test_get_model_parameters(self):
        res = self._get_models()

        self.assertEqual(len(res.link_names), 1)
        self.assertEqual(len(res.models), 1)

        self.assertEqual(
            res.link_names[0], 'vehicle/base_link',
            'Link name is invalid, name=' + str(res.link_names[0]))

        m = res.models[0]
        self.assertIsInstance(list(m.added_mass), list)
        self.assertIsInstance(list(m.linear_damping), list)
        self.assertIsInstance(list(m.linear_damping_forward_speed), list)
        self.assertIsInstance(list(m.quadratic_damping), list)
        self.assertIsInstance(m.volume, float)
        self.assertIsInstance(m.bbox_length, float)
        self.assertIsInstance(m.bbox_width, float)
        self.assertIsInstance(m.bbox_height, float)
        self.assertIsInstance(m.fluid_density, float)
        self.assertIsInstance(m.neutrally_buoyant, bool)
        self.assertIsInstance(m.cob, Vector3)
        self.assertIsInstance(m.inertia, Inertia)

        self.assertEqual(len(m.added_mass), 36)
        self.assertEqual(len(m.linear_damping), 36)
        self.assertEqual(len(m.linear_damping_forward_speed), 36)
        self.assertEqual(len(m.quadratic_damping), 36)

        self.assertEqual(m.fluid_density, 1028.0)
        self.assertAlmostEqual(m.volume, 0.009727626, places=6)
        self.assertEqual(m.bbox_height, 1.0)
        self.assertEqual(m.bbox_length, 1.0)
        self.assertEqual(m.bbox_width, 1.0)

    def test_added_mass_coefs(self):
        m = self._get_models().models[0]

        d_idxs = [i * 6 + j for i, j in zip(range(3), range(3))]
        sphere_ma = 2.0 / 3.0 * m.fluid_density * np.pi * RADIUS ** 3.0

        for i, v in enumerate(m.added_mass):
            if i in d_idxs:
                self.assertLess(abs(v - sphere_ma), 0.001)
            else:
                self.assertEqual(v, 0.0)

    def test_nonlinear_damping_coefs(self):
        m = self._get_models().models[0]

        area_section = np.pi * RADIUS ** 2
        dq = -0.5 * m.fluid_density * CD * area_section

        d_idxs = [i * 6 + j for i, j in zip(range(3), range(3))]
        for i, v in enumerate(m.quadratic_damping):
            if i in d_idxs:
                self.assertLess(abs(v - dq), 0.001)
            else:
                self.assertEqual(v, 0.0)


if __name__ == '__main__':
    unittest.main()
