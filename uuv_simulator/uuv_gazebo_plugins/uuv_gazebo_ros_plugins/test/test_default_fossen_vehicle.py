#!/usr/bin/env python3
# Copyright (c) 2016 The UUV Simulator Authors.
# Licensed under the Apache License, Version 2.0.

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Inertia
from uuv_gazebo_ros_plugins_msgs.msg import UnderwaterObjectModel
from uuv_gazebo_ros_plugins_msgs.srv import (
    GetModelProperties, SetFloat, GetFloat
)


class TestDefaultFossenVehicle(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_default_fossen_vehicle')

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

    def test_get_model_parameters(self):
        res = self._call(GetModelProperties, '/vehicle/get_model_properties')

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

        d_idxs = [i * 6 + j for i, j in zip(range(6), range(6))]

        self.assertEqual(len(m.added_mass), 36)
        for i, v in enumerate(m.added_mass):
            self.assertEqual(v, 1.0 if i in d_idxs else 0.0)

        self.assertEqual(len(m.linear_damping), 36)
        for i, v in enumerate(m.linear_damping):
            self.assertEqual(v, 1.0 if i in d_idxs else 0.0)

        self.assertEqual(len(m.linear_damping_forward_speed), 36)
        for i, v in enumerate(m.linear_damping_forward_speed):
            self.assertEqual(v, 1.0 if i in d_idxs else 0.0)

        self.assertEqual(len(m.quadratic_damping), 36)
        for i, v in enumerate(m.quadratic_damping):
            self.assertEqual(v, 1.0 if i in d_idxs else 0.0)

        self.assertEqual(m.fluid_density, 1028.0)
        self.assertEqual(m.volume, 1.0)
        self.assertEqual(m.bbox_height, 1.0)
        self.assertEqual(m.bbox_length, 1.0)
        self.assertEqual(m.bbox_width, 1.0)

    def test_set_fluid_density(self):
        get_req = GetFloat.Request()
        set_req_1025 = SetFloat.Request(); set_req_1025.data = 1025.0
        set_req_1028 = SetFloat.Request(); set_req_1028.data = 1028.0

        self.assertEqual(
            self._call(GetFloat, '/vehicle/get_fluid_density', get_req).data,
            1028.0)
        self.assertTrue(
            self._call(SetFloat, '/vehicle/set_fluid_density', set_req_1025).success)
        self.assertEqual(
            self._call(GetFloat, '/vehicle/get_fluid_density', get_req).data,
            1025.0)
        self.assertTrue(
            self._call(SetFloat, '/vehicle/set_fluid_density', set_req_1028).success)

    def test_volume_offset(self):
        get_req  = GetFloat.Request()
        set_1    = SetFloat.Request(); set_1.data = 1.0
        set_0    = SetFloat.Request(); set_0.data = 0.0

        self.assertEqual(
            self._call(GetFloat, '/vehicle/get_volume_offset', get_req).data, 0.0)
        self.assertTrue(
            self._call(SetFloat, '/vehicle/set_volume_offset', set_1).success)
        self.assertEqual(
            self._call(GetFloat, '/vehicle/get_volume_offset', get_req).data, 1.0)

        # Actual volume must not change — offset only affects force computation
        res = self._call(GetModelProperties, '/vehicle/get_model_properties')
        self.assertEqual(res.models[0].volume, 1.0)

        self.assertTrue(
            self._call(SetFloat, '/vehicle/set_volume_offset', set_0).success)

    def _test_scaling(self, set_srv, get_srv, default=1.0, test_val=0.8):
        get_req = GetFloat.Request()
        set_test = SetFloat.Request(); set_test.data = test_val
        set_back = SetFloat.Request(); set_back.data = default

        self.assertEqual(
            self._call(GetFloat, get_srv, get_req).data, default)
        self.assertTrue(
            self._call(SetFloat, set_srv, set_test).success)
        self.assertEqual(
            self._call(GetFloat, get_srv, get_req).data, test_val)
        self.assertTrue(
            self._call(SetFloat, set_srv, set_back).success)

    def _test_offset(self, set_srv, get_srv, test_val=1.0):
        get_req = GetFloat.Request()
        set_test = SetFloat.Request(); set_test.data = test_val
        set_back = SetFloat.Request(); set_back.data = 0.0

        self.assertEqual(
            self._call(GetFloat, get_srv, get_req).data, 0.0)
        self.assertTrue(
            self._call(SetFloat, set_srv, set_test).success)
        self.assertEqual(
            self._call(GetFloat, get_srv, get_req).data, test_val)
        self.assertTrue(
            self._call(SetFloat, set_srv, set_back).success)

    def test_added_mass_scaling(self):
        self._test_scaling(
            '/vehicle/set_added_mass_scaling',
            '/vehicle/get_added_mass_scaling')

    def test_damping_scaling(self):
        self._test_scaling(
            '/vehicle/set_damping_scaling',
            '/vehicle/get_damping_scaling')

    def test_volume_scaling(self):
        self._test_scaling(
            '/vehicle/set_volume_scaling',
            '/vehicle/get_volume_scaling')

    def test_added_mass_offset(self):
        self._test_offset(
            '/vehicle/set_added_mass_offset',
            '/vehicle/get_added_mass_offset')

    def test_linear_damping_offset(self):
        self._test_offset(
            '/vehicle/set_linear_damping_offset',
            '/vehicle/get_linear_damping_offset')

    def test_linear_forward_speed_damping_offset(self):
        self._test_offset(
            '/vehicle/set_linear_forward_speed_damping_offset',
            '/vehicle/get_linear_forward_speed_damping_offset')

    def test_nonlinear_damping_offset(self):
        self._test_offset(
            '/vehicle/set_nonlinear_damping_offset',
            '/vehicle/get_nonlinear_damping_offset')


if __name__ == '__main__':
    unittest.main()
