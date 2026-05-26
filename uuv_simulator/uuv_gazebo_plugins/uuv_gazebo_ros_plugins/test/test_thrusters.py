#!/usr/bin/env python3
# Copyright (c) 2016 The UUV Simulator Authors.
# Licensed under the Apache License, Version 2.0.

import unittest
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from uuv_gazebo_ros_plugins_msgs.srv import (
    GetThrusterConversionFcn,
    SetThrusterState, GetThrusterState,
    SetThrusterEfficiency, GetThrusterEfficiency,
)


class TestThrusters(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_thrusters')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        cls.thruster_input_pub = {}
        for i in range(3):
            cls.thruster_input_pub[i] = cls.node.create_publisher(
                FloatStamped,
                f'/vehicle/thrusters/{i}/input',
                qos)

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

    def _wait_for_message(self, topic, msg_type, timeout=30.0):
        """Spin until one message arrives on the topic or timeout expires."""
        received = []
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        sub = self.node.create_subscription(
            msg_type, topic,
            lambda msg: received.append(msg),
            qos)
        deadline = time.time() + timeout
        while not received and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        self.assertTrue(received, f'No message received on {topic}')
        return received[0]

    def test_input_output_topics_exist(self):
        for k in self.thruster_input_pub:
            msg = FloatStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.data = 0.2
            self.thruster_input_pub[k].publish(msg)
            time.sleep(1.0)

            output = self._wait_for_message(
                f'/vehicle/thrusters/{k}/thrust', FloatStamped, timeout=30.0)
            self.assertIsNot(output.data, 0.0)

            # Turn off
            msg.data = 0.0
            self.thruster_input_pub[k].publish(msg)

    def test_convertion_fcn_parameters(self):
        # Thruster 0 — Basic/proportional
        req = GetThrusterConversionFcn.Request()
        fcn = self._call(
            GetThrusterConversionFcn,
            '/vehicle/thrusters/0/get_thruster_conversion_fcn',
            req).fcn

        self.assertEqual(fcn.function_name, 'Basic')
        self.assertEqual(len(fcn.tags), len(fcn.data))
        self.assertEqual(len(fcn.tags), 1)
        self.assertIn('rotor_constant', fcn.tags)
        self.assertEqual(fcn.data[0], 0.001)

        # Thruster 1 — Bessa/nonlinear
        fcn = self._call(
            GetThrusterConversionFcn,
            '/vehicle/thrusters/1/get_thruster_conversion_fcn',
            req).fcn

        bessa_tags   = ['rotor_constant_l', 'rotor_constant_r', 'delta_l', 'delta_r']
        bessa_params = [0.001, 0.001, -0.01, 0.01]
        self.assertEqual(fcn.function_name, 'Bessa')
        self.assertEqual(len(fcn.tags), 4)
        for t, p in zip(fcn.tags, fcn.data):
            self.assertIn(t, bessa_tags)
            self.assertEqual(p, bessa_params[bessa_tags.index(t)])

        # Thruster 2 — LinearInterp
        fcn = self._call(
            GetThrusterConversionFcn,
            '/vehicle/thrusters/2/get_thruster_conversion_fcn',
            req).fcn

        self.assertEqual(fcn.function_name, 'LinearInterp')
        self.assertEqual(len(fcn.tags), 0)
        self.assertEqual(len(fcn.lookup_table_input),
                         len(fcn.lookup_table_output))
        self.assertListEqual([-0.1, 0.0, 0.1],
                             list(fcn.lookup_table_input))
        self.assertListEqual([-0.01, 0.0, 0.01],
                             list(fcn.lookup_table_output))

    def test_change_thruster_state(self):
        for i in range(3):
            set_off = SetThrusterState.Request(); set_off.on = False
            set_on  = SetThrusterState.Request(); set_on.on  = True

            self.assertTrue(
                self._call(SetThrusterState,
                           f'/vehicle/thrusters/{i}/set_thruster_state',
                           set_off).success)

            res = self._call(GetThrusterState,
                             f'/vehicle/thrusters/{i}/get_thruster_state')
            self.assertFalse(res.is_on)

            self.assertTrue(
                self._call(SetThrusterState,
                           f'/vehicle/thrusters/{i}/set_thruster_state',
                           set_on).success)
            self.assertTrue(
                self._call(GetThrusterState,
                           f'/vehicle/thrusters/{i}/get_thruster_state').is_on)

    def test_change_thrust_efficiency(self):
        for i in range(3):
            set_half = SetThrusterEfficiency.Request(); set_half.efficiency = 0.5
            set_full = SetThrusterEfficiency.Request(); set_full.efficiency = 1.0

            self.assertTrue(
                self._call(SetThrusterEfficiency,
                           f'/vehicle/thrusters/{i}/set_thrust_force_efficiency',
                           set_half).success)

            res = self._call(GetThrusterEfficiency,
                             f'/vehicle/thrusters/{i}/get_thrust_force_efficiency')
            self.assertEqual(res.efficiency, 0.5)

            self.assertTrue(
                self._call(SetThrusterEfficiency,
                           f'/vehicle/thrusters/{i}/set_thrust_force_efficiency',
                           set_full).success)
            self.assertEqual(
                self._call(GetThrusterEfficiency,
                           f'/vehicle/thrusters/{i}/get_thrust_force_efficiency'
                           ).efficiency, 1.0)

    def test_change_dyn_state_efficiency(self):
        for i in range(3):
            set_half = SetThrusterEfficiency.Request(); set_half.efficiency = 0.5
            set_full = SetThrusterEfficiency.Request(); set_full.efficiency = 1.0

            self.assertTrue(
                self._call(SetThrusterEfficiency,
                           f'/vehicle/thrusters/{i}/set_dynamic_state_efficiency',
                           set_half).success)

            res = self._call(GetThrusterEfficiency,
                             f'/vehicle/thrusters/{i}/get_dynamic_state_efficiency')
            self.assertEqual(res.efficiency, 0.5)

            self.assertTrue(
                self._call(SetThrusterEfficiency,
                           f'/vehicle/thrusters/{i}/set_dynamic_state_efficiency',
                           set_full).success)
            self.assertEqual(
                self._call(GetThrusterEfficiency,
                           f'/vehicle/thrusters/{i}/get_dynamic_state_efficiency'
                           ).efficiency, 1.0)


if __name__ == '__main__':
    unittest.main()
