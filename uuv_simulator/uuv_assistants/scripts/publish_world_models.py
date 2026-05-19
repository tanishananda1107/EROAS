#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from gazebo_msgs.srv import GetModelState
from tf_transformations import quaternion_from_euler
import time


class WorldPublisher(Node):

    def __init__(self):
        super().__init__('publish_world_models')

        self.declare_parameter('meshes', {})
        self.meshes = self.get_parameter('meshes').value

        self.pub = self.create_publisher(MarkerArray, '/world_models', 10)

        self.cli = self.create_client(GetModelState, '/gazebo/get_model_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Gazebo service...")

        self.models = {}

        self.timer = self.create_timer(1.0, self.update_and_publish)

    def update_and_publish(self):
        if not self.meshes:
            return

        ma = MarkerArray()
        i = 0

        for model_name, cfg in self.meshes.items():

            req = GetModelState.Request()
            req.model_name = model_name

            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)

            if not future.result():
                continue

            state = future.result()

            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.action = Marker.ADD

            marker.pose = state.pose

            marker.scale.x = cfg.get('scale', [1, 1, 1])[0]
            marker.scale.y = cfg.get('scale', [1, 1, 1])[1]
            marker.scale.z = cfg.get('scale', [1, 1, 1])[2]

            marker.color.a = 0.5
            marker.color.b = 1.0

            ma.markers.append(marker)
            i += 1

        self.pub.publish(ma)


def main():
    rclpy.init()
    node = WorldPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
