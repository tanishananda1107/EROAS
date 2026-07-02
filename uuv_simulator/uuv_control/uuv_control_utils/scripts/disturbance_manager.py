#!/usr/bin/env python3

import sys
import logging
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import (
    WrenchStamped,
    Vector3
)

from ros_gz_interfaces.srv import ApplyLinkWrench

from uuv_world_ros_plugins_msgs.srv import (
    SetCurrentVelocity
)

from uuv_gazebo_ros_plugins_msgs.srv import (
    SetThrusterState,
    SetThrusterEfficiency
)


class DisturbanceManager(Node):

    def __init__(self):

        super().__init__(
            'disturbance_manager'
        )

        self._logger = logging.getLogger(
            'disturbance_manager'
        )

        out_hdlr = logging.StreamHandler(
            sys.stdout
        )

        out_hdlr.setFormatter(
            logging.Formatter(
                '%(asctime)s | %(levelname)s | %(message)s'
            )
        )

        self._logger.addHandler(
            out_hdlr
        )

        self._logger.setLevel(
            logging.INFO
        )

        self.declare_parameter(
            'disturbances',
            []
        )

        self._disturbances = (
            self.get_parameter(
                'disturbances'
            ).value
        )

        if len(
            self._disturbances
        ) == 0:

            raise RuntimeError(
                'No disturbances supplied'
            )

        for d in self._disturbances:

            d['is_applied'] = False
            d['ended'] = False

        self._body_force = np.zeros(
            3
        )

        self._body_torque = np.zeros(
            3
        )

        self._wrench_pub = (
            self.create_publisher(
                WrenchStamped,
                'wrench_perturbation',
                10
            )
        )

        self.current_client = (
            self.create_client(
                SetCurrentVelocity,
                '/hydrodynamics/set_current_velocity'
            )
        )

        self.wrench_client = (
            self.create_client(
                ApplyLinkWrench,
                '/world/default/apply_link_wrench'
            )
        )

        while not (
            self.current_client.wait_for_service(
                timeout_sec=2.0
            )
        ):

            self.get_logger().info(
                'Waiting current service'
            )

        while not (
            self.wrench_client.wait_for_service(
                timeout_sec=2.0
            )
        ):

            self.get_logger().info(
                'Waiting wrench service'
            )

        self.create_timer(
            0.1,
            self.publish_wrench
        )

        self.create_timer(
            0.01,
            self.update_disturbances
        )

    def publish_wrench(self):

        msg = WrenchStamped()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            'world'
        )

        msg.wrench.force = (
            Vector3(
                x=float(
                    self._body_force[0]
                ),
                y=float(
                    self._body_force[1]
                ),
                z=float(
                    self._body_force[2]
                )
            )
        )

        msg.wrench.torque = (
            Vector3(
                x=float(
                    self._body_torque[0]
                ),
                y=float(
                    self._body_torque[1]
                ),
                z=float(
                    self._body_torque[2]
                )
            )
        )

        self._wrench_pub.publish(
            msg
        )

    def update_disturbances(
            self):

        t = (
            self.get_clock()
            .now()
            .nanoseconds
            / 1e9
        )

        for d in self._disturbances:

            if (
                t >
                d['starting_time']
                and
                not d[
                    'is_applied'
                ]
            ):

                if (
                    d['type']
                    ==
                    'current'
                ):

                    self.set_current(
                        d[
                            'velocity'
                        ],
                        d[
                            'horizontal_angle'
                        ],
                        d[
                            'vertical_angle'
                        ]
                    )

                elif (
                    d['type']
                    ==
                    'wrench'
                ):

                    self.set_body_wrench(
                        d['force'],
                        d['torque']
                    )

                d[
                    'is_applied'
                ] = True

    def set_current(
            self,
            velocity,
            h_angle,
            v_angle):

        req = (
            SetCurrentVelocity
            .Request()
        )

        req.velocity = (
            velocity
        )

        req.horizontal_angle = (
            h_angle
        )

        req.vertical_angle = (
            v_angle
        )

        self.current_client.call_async(
            req
        )

    def set_body_wrench(
            self,
            force,
            torque):

        ns = (
            self.get_namespace()
            .replace(
                '/',
                ''
            )
        )

        body_name = (
            f'{ns}/base_link'
        )

        req = (
            ApplyLinkWrench
            .Request()
        )

        req.link_name = (
            body_name
        )

        req.force = Vector3(
            x=float(
                force[0]
            ),
            y=float(
                force[1]
            ),
            z=float(
                force[2]
            )
        )

        req.torque = Vector3(
            x=float(
                torque[0]
            ),
            y=float(
                torque[1]
            ),
            z=float(
                torque[2]
            )
        )

        future = (
            self.wrench_client
            .call_async(
                req
            )
        )

        self._body_force += (
            np.array(
                force
            )
        )

        self._body_torque += (
            np.array(
                torque
            )
        )

        return future


def main(args=None):

    rclpy.init(
        args=args
    )

    node = (
        DisturbanceManager()
    )

    rclpy.spin(
        node
    )

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()
