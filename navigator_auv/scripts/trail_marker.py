#!/usr/bin/env python3
from collections import deque
import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker


class TrailMarker(Node):
    def __init__(self):
        super().__init__('trail_marker')
        self.declare_parameter('pose_topic', '/rexrov2/pose_gt')
        self.declare_parameter('marker_topic', '/rexrov2/trail_marker')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('max_points', 45)
        self.declare_parameter('max_length', 10.0)
        self.declare_parameter('min_distance', 0.18)
        self.declare_parameter('line_width', 0.18)
        self.declare_parameter('publish_rate', 12.0)

        pose_topic = self.get_parameter('pose_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.frame_id = self.get_parameter('frame_id').value
        self.max_points = int(self.get_parameter('max_points').value)
        self.max_length = float(self.get_parameter('max_length').value)
        self.min_distance = float(self.get_parameter('min_distance').value)
        self.line_width = float(self.get_parameter('line_width').value)
        self.points = deque()

        self.marker_pub = self.create_publisher(Marker, marker_topic, 10)
        self.create_subscription(Odometry, pose_topic, self.pose_cb, 10)
        self.create_timer(1.0 / publish_rate, self.publish_marker)

    @staticmethod
    def _dist(a, b):
        return math.sqrt(
            (a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    def pose_cb(self, msg):
        p = Point()
        p.x = msg.pose.pose.position.x
        p.y = msg.pose.pose.position.y
        p.z = msg.pose.pose.position.z

        if self.points and self._dist(p, self.points[-1]) < self.min_distance:
            return

        self.points.append(p)
        while len(self.points) > self.max_points:
            self.points.popleft()

        while len(self.points) > 2 and self._trail_length() > self.max_length:
            self.points.popleft()

    def _trail_length(self):
        return sum(
            self._dist(self.points[i], self.points[i - 1])
            for i in range(1, len(self.points)))

    def publish_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = 'rexrov2_trail'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width
        marker.scale.y = self.line_width
        marker.scale.z = self.line_width
        marker.color.r = 0.0
        marker.color.g = 0.95
        marker.color.b = 0.45
        marker.color.a = 0.95
        marker.points = list(self.points)
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = TrailMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
