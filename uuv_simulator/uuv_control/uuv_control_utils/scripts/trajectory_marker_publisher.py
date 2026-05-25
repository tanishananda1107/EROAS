#!/usr/bin/env python3

import os

import rclpy

from rclpy.node import Node

from std_msgs.msg import Bool

from nav_msgs.msg import Path

from visualization_msgs.msg import (
    Marker,
    MarkerArray
)

from geometry_msgs.msg import (
    PoseStamped
)

from uuv_control_msgs.msg import (
    Trajectory,
    TrajectoryPoint,
    WaypointSet
)


class TrajectoryMarkerPublisher(Node):

    def __init__(self):

        super().__init__(
            'trajectory_marker_publisher'
        )

        self.trajectory = None

        self.waypoints = None

        self.create_subscription(
            Trajectory,
            'trajectory',
            self.update_trajectory,
            10
        )

        self.create_subscription(
            WaypointSet,
            'waypoints',
            self.update_waypoints,
            10
        )

        self.create_subscription(
            Bool,
            'automatic_on',
            lambda x: None,
            10
        )

        self.create_subscription(
            Bool,
            'trajectory_tracking_on',
            lambda x: None,
            10
        )

        self.create_subscription(
            Bool,
            'station_keeping_on',
            lambda x: None,
            10
        )

        self.create_subscription(
            TrajectoryPoint,
            'reference',
            self.reference_callback,
            10
        )

        self.traj_pub = self.create_publisher(
            Path,
            'trajectory_marker',
            10
        )

        self.wp_pub = self.create_publisher(
            MarkerArray,
            'waypoint_markers',
            10
        )

        self.ref_pub = self.create_publisher(
            Marker,
            'reference_marker',
            10
        )

        self.timer = self.create_timer(
            0.5,
            self.publish_markers
        )

    def update_trajectory(self, msg):

        self.trajectory = msg

    def update_waypoints(self, msg):

        self.waypoints = msg

    def publish_markers(self):

        path = Path()

        if self.trajectory:

            path.header.frame_id = \
                self.trajectory.header.frame_id

            for p in self.trajectory.points:

                pose = PoseStamped()

                pose.header = p.header

                pose.pose = p.pose

                path.poses.append(
                    pose
                )

        self.traj_pub.publish(
            path
        )

    def reference_callback(self, msg):

        marker = Marker()

        marker.header.frame_id = \
            msg.header.frame_id

        marker.type = Marker.SPHERE

        marker.action = Marker.MODIFY

        marker.pose.position = \
            msg.pose.position

        marker.scale.x = 0.3

        marker.scale.y = 0.3

        marker.scale.z = 0.3

        marker.color.a = 1.0

        marker.color.g = 1.0

        self.ref_pub.publish(
            marker
        )


def main(args=None):

    rclpy.init(args=args)

    node = TrajectoryMarkerPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
