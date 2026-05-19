#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from copy import deepcopy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32
from gazebo_msgs.srv import GetWorldProperties, GetModelProperties


class FootprintPublisher(Node):

    def __init__(self):
        super().__init__('publish_footprints')

        self.vehicle_pub = {}
        self.odom_sub = {}

        self.marker = np.array([[0, 0.75], [-0.5, -0.25], [0.5, -0.25]])

        self.get_world = self.create_client(GetWorldProperties, '/gazebo/get_world_properties')
        self.get_model = self.create_client(GetModelProperties, '/gazebo/get_model_properties')

        self.timer = self.create_timer(10.0, self.update_vehicle_list)

    def rot(self, alpha):
        return np.array([[np.cos(alpha), -np.sin(alpha)],
                         [np.sin(alpha), np.cos(alpha)]])

    def odom_callback(self, msg, name):
        if name not in self.vehicle_pub:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)

        new_marker = deepcopy(self.marker)
        points = []

        for i in range(new_marker.shape[0]):
            new_marker[i, :] = np.dot(self.rot(yaw - np.pi/2), new_marker[i, :])
            new_marker[i, 0] += x
            new_marker[i, 1] += y

            p = Point32()
            p.x = float(new_marker[i, 0])
            p.y = float(new_marker[i, 1])
            points.append(p)

        poly = PolygonStamped()
        poly.header.frame_id = 'world'
        poly.header.stamp = self.get_clock().now().to_msg()
        poly.polygon.points = points

        self.vehicle_pub[name].publish(poly)

    def quat_to_yaw(self, x, y, z, w):
        import math
        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        return math.atan2(siny_cosp, cosy_cosp)

    def update_vehicle_list(self):
        if not self.get_world.service_is_ready():
            return

        world_req = GetWorldProperties.Request()
        world_future = self.get_world.call_async(world_req)
        rclpy.spin_until_future_complete(self, world_future, timeout_sec=1.0)

        if not world_future.result():
            return

        models = world_future.result().model_names

        for model in models:
            if model in self.vehicle_pub:
                continue

            # subscriber per model (topic guess)
            topic = f'/{model}/pose_gt'

            self.vehicle_pub[model] = self.create_publisher(
                PolygonStamped,
                f'/{model}/footprint',
                10
            )

            self.create_subscription(
                Odometry,
                topic,
                lambda msg, m=model: self.odom_callback(msg, m),
                10
            )


def main():
    rclpy.init()
    node = FootprintPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
