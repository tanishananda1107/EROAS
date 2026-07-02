#!/usr/bin/env python3

import math
import numpy as np

import rclpy

from rclpy.node import Node

from copy import deepcopy

from nav_msgs.msg import (
    Odometry
)

from geometry_msgs.msg import (
    Vector3,
    Quaternion
)

from uuv_control_msgs.msg import (
    TrajectoryPoint
)

from uuv_gazebo_ros_plugins_msgs.msg import (
    FloatStamped
)

from uuv_control_interfaces import (
    DPControllerLocalPlanner
)

from uuv_thrusters.models import (
    Thruster
)

import tf2_ros

import tf_transformations as trans


class AUVGeometricTrackingController(
        Node):

    def __init__(self):

        super().__init__(
            "auv_geometric_tracking_controller"
        )

        self.namespace = (
            self.get_namespace()
            .replace(
                '/',
                ''
            )
        )

        self.local_planner = (
            DPControllerLocalPlanner(
                full_dof=True,
                thrusters_only=False,
                stamped_pose_only=False
            )
        )

        self.declare_parameter(
            "base_link",
            "base_link"
        )

        self.base_link = (
            self.get_parameter(
                "base_link"
            ).value
        )

        self.declare_parameter(
            "min_thrust",
            0.0
        )

        self.declare_parameter(
            "max_thrust",
            100.0
        )

        self.min_thrust = (
            self.get_parameter(
                "min_thrust"
            ).value
        )

        self.max_thrust = (
            self.get_parameter(
                "max_thrust"
            ).value
        )

        self.declare_parameter(
            "n_fins",
            4
        )

        self.n_fins = (
            self.get_parameter(
                "n_fins"
            ).value
        )

        self.tf_buffer = (
            tf2_ros.Buffer()
        )

        self.listener = (
            tf2_ros.TransformListener(
                self.tf_buffer,
                self
            )
        )

        self.pub_cmd = []

        for i in range(
                self.n_fins):

            self.pub_cmd.append(

                self.create_publisher(

                    FloatStamped,

                    f"fins/{i}/input",

                    10

                )

            )

        self.reference_pub = (
            self.create_publisher(
                TrajectoryPoint,
                "reference",
                10
            )
        )

        self.error_pub = (
            self.create_publisher(
                TrajectoryPoint,
                "error",
                10
            )
        )

        self.sub = (
            self.create_subscription(
                Odometry,
                "odom",
                self.odometry_callback,
                10
            )
        )

    def unwrap_angle(
            self,
            t):

        return math.atan2(

            math.sin(t),

            math.cos(t)

        )

    def odometry_callback(
            self,
            msg):

        pos = [

            msg.pose.pose.position.x,

            msg.pose.pose.position.y,

            msg.pose.pose.position.z

        ]

        quat = [

            msg.pose.pose.orientation.x,

            msg.pose.pose.orientation.y,

            msg.pose.pose.orientation.z,

            msg.pose.pose.orientation.w

        ]

        self.local_planner.update_vehicle_pose(
            pos,
            quat
        )

        t = (
            self.get_clock()
            .now()
            .nanoseconds
            * 1e-9
        )

        des = (
            self.local_planner
            .interpolate(t)
        )

        p = np.array(
            pos
        )

        ep = (
            des.p
            -
            p
        )

        thrust = min(

            self.max_thrust,

            np.linalg.norm(ep)

        )

        thrust = max(

            thrust,

            self.min_thrust

        )

        cmd = FloatStamped()

        cmd.data = thrust

        for pub in self.pub_cmd:

            pub.publish(
                cmd
            )


def main(args=None):

    rclpy.init(args=args)

    node = (
        AUVGeometricTrackingController()
    )

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()
