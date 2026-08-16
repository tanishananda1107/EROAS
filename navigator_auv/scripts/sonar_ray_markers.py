#!/usr/bin/env python3
"""
Republish the sonar's LaserScan as radiating line-segment markers.

RViz2's stock LaserScan display only plots a dot at each beam's hit
location -- against a flat surface that collapses into a single straight
line of points, not the fan of rays the paper/original repo shows. This
draws the actual ray geometry: one line segment per beam from the sensor
origin out to its range (hit range, or max range for a clear beam).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class SonarRayMarkers(Node):

    def __init__(self):
        super().__init__('sonar_ray_markers')

        self.declare_parameter('scan_topic', '/rexrov2/sonar_visual_rays')
        self.declare_parameter('marker_topic', '/rexrov2/sonar_ray_markers')
        self.declare_parameter('line_width', 0.03)
        self.declare_parameter('color_r', 0.12)
        self.declare_parameter('color_g', 0.35)
        self.declare_parameter('color_b', 1.0)
        self.declare_parameter('color_a', 0.55)

        scan_topic = self.get_parameter('scan_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self.line_width = float(self.get_parameter('line_width').value)
        self.color = ColorRGBA(
            r=float(self.get_parameter('color_r').value),
            g=float(self.get_parameter('color_g').value),
            b=float(self.get_parameter('color_b').value),
            a=float(self.get_parameter('color_a').value),
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.marker_pub = self.create_publisher(Marker, marker_topic, 1)
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, sensor_qos)
        self.get_logger().info(f'[SONAR_RAYS] {scan_topic} -> {marker_topic}')

    def _scan_cb(self, msg: LaserScan) -> None:
        marker = Marker()
        marker.header = msg.header
        marker.ns = 'sonar_rays'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.line_width
        marker.color = self.color
        marker.pose.orientation.w = 1.0

        origin = Point(x=0.0, y=0.0, z=0.0)
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and r > 0.0:
                beam_range = min(r, msg.range_max)
            else:
                beam_range = msg.range_max
            marker.points.append(origin)
            marker.points.append(Point(
                x=beam_range * math.cos(angle),
                y=beam_range * math.sin(angle),
                z=0.0,
            ))
            angle += msg.angle_increment

        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = SonarRayMarkers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
