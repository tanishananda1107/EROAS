#!/usr/bin/env python3

import rclpy
import pytest
import numpy as np

from rclpy.node import Node

from uuv_thruster_manager_msgs.srv import (
    GetThrusterManagerConfig,
    ThrusterManagerInfo
)

NS = "test_vehicle"

AXIS_X_TAM = np.array([
    [1,0,0,0,0,0],
    [0.87758256,0,-0.47942554,0.47942554,0.47942554,0.87758256],
    [0.87758256,0.47942554,0,-0.47942554,0.87758256,-0.87758256]
]).T


class TestNode(Node):

    def __init__(self):
        super().__init__("test_thruster_allocator")


@pytest.fixture(scope="module")
def ros_node():

    rclpy.init()

    node = TestNode()

    yield node

    node.destroy_node()

    rclpy.shutdown()


def test_config(ros_node):

    client = ros_node.create_client(
        GetThrusterManagerConfig,
        "/test_vehicle/thruster_manager/get_config"
    )

    assert client.wait_for_service(timeout_sec=20)

    req = GetThrusterManagerConfig.Request()

    future = client.call_async(req)

    rclpy.spin_until_future_complete(
        ros_node,
        future
    )

    result = future.result()

    assert result.tf_prefix == "/test_vehicle/"
    assert result.base_link == "base_link"

    tam_flat = AXIS_X_TAM.flatten()

    assert len(result.allocation_matrix) == tam_flat.size
