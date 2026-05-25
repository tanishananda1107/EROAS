#!/usr/bin/env python3

import numpy as np
import yaml

from copy import deepcopy
from os.path import isdir, join

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import (
    Wrench,
    WrenchStamped
)

from uuv_thrusters import ThrusterManager

from uuv_thruster_manager.srv import (
    ThrusterManagerInfo,
    GetThrusterCurve,
    SetThrusterManagerConfig,
    GetThrusterManagerConfig
)


class ThrusterAllocatorNode(
    ThrusterManager,
    Node
):

    def __init__(self):

        Node.__init__(
            self,
            'thruster_allocator'
        )

        ThrusterManager.__init__(
            self
        )

        self.last_update = (
            self.get_clock().now()
        )

        self.input_sub = (
            self.create_subscription(
                Wrench,

                'thruster_manager/input',

                self.input_callback,

                10
            )
        )

        self.input_stamped_sub = (
            self.create_subscription(
                WrenchStamped,

                'thruster_manager/input_stamped',

                self.input_stamped_callback,

                10
            )
        )

        self.thruster_info_service = (
            self.create_service(
                ThrusterManagerInfo,

                'thruster_manager/get_thrusters_info',

                self.get_thruster_info
            )
        )

        self.curve_calc_service = (
            self.create_service(
                GetThrusterCurve,

                'thruster_manager/get_thruster_curve',

                self.get_thruster_curve
            )
        )

        self.set_thruster_manager_config_service = (
            self.create_service(
                SetThrusterManagerConfig,

                'thruster_manager/set_config',

                self.set_config
            )
        )

        self.get_thruster_manager_config_service = (
            self.create_service(
                GetThrusterManagerConfig,

                'thruster_manager/get_config',

                self.get_config
            )
        )

        update_rate = (
            self.config[
                'update_rate'
            ]
        )

        self.timer = (
            self.create_timer(
                1.0 / update_rate,

                self.timeout_callback
            )
        )

    def timeout_callback(
        self
    ):

        if (
            self.config[
                'timeout'
            ] <= 0
        ):
            return

        current_time = (
            self.get_clock()
            .now()
        )

        elapsed = (
            current_time
            -
            self.last_update
        )

        timeout_ns = int(
            self.config[
                'timeout'
            ] * 1e9
        )

        if (
            elapsed.nanoseconds
            >
            timeout_ns
        ):

            self.get_logger().info(
                'Turning thrusters off'
            )

            if (
                self.thrust
                is not None
            ):

                self.thrust.fill(
                    0
                )

                self.command_thrusters()

    def get_thruster_info(
        self,

        request,

        response
    ):

        response.n_thrusters = (
            self.n_thrusters
        )

        response.configuration = (
            self.configuration_matrix
            .flatten()
            .tolist()
        )

        response.base_link = (
            self.namespace
            +
            self.config[
                'base_link'
            ]
        )

        return response

    def get_thruster_curve(
        self,

        request,

        response
    ):

        if (
            self.n_thrusters
            == 0
        ):

            response.input = []

            response.thrust = []

            return response

        (
            input_values,

            thrust_values

        ) = (
            self.thrusters[
                0
            ].get_curve(

                request.min,

                request.max,

                request.n_points
            )
        )

        response.input = (
            input_values
        )

        response.thrust = (
            thrust_values
        )

        return response

    def set_config(
        self,

        request,

        response
    ):

        old_config = deepcopy(
            self.config
        )

        self.ready = False

        self.config[
            'base_link'
        ] = (
            request.base_link
        )

        self.config[
            'thruster_frame_base'
        ] = (
            request.thruster_frame_base
        )

        self.config[
            'thruster_topic_prefix'
        ] = (
            request
            .thruster_topic_prefix
        )

        self.config[
            'thruster_topic_suffix'
        ] = (
            request
            .thruster_topic_suffix
        )

        self.config[
            'timeout'
        ] = (
            request.timeout
        )

        self.get_logger().info(
            'Updating config'
        )

        if not self.update_tam(
            recalculate=True
        ):

            self.get_logger().warn(
                'Invalid config reverting'
            )

            self.config = (
                old_config
            )

            self.update_tam(
                recalculate=True
            )

        response.success = True

        return response

    def get_config(
        self,

        request,

        response
    ):

        response.namespace = (
            self.namespace
        )

        response.base_link = (
            self.config[
                'base_link'
            ]
        )

        response.thruster_frame_base = (
            self.config[
                'thruster_frame_base'
            ]
        )

        response.thruster_topic_prefix = (
            self.config[
                'thruster_topic_prefix'
            ]
        )

        response.thruster_topic_suffix = (
            self.config[
                'thruster_topic_suffix'
            ]
        )

        response.timeout = (
            self.config[
                'timeout'
            ]
        )

        response.max_thrust = (
            self.config[
                'max_thrust'
            ]
        )

        response.n_thrusters = (
            self.n_thrusters
        )

        response.configuration = (
            self.configuration_matrix
            .flatten()
            .tolist()
        )

        return response

    def input_callback(
        self,

        msg
    ):

        if not self.ready:
            return

        force = np.array(

            [

                msg.force.x,

                msg.force.y,

                msg.force.z

            ]
        )

        torque = np.array(

            [

                msg.torque.x,

                msg.torque.y,

                msg.torque.z

            ]
        )

        self.publish_thrust_forces(

            force,

            torque
        )

        self.last_update = (
            self.get_clock()
            .now()
        )

    def input_stamped_callback(
        self,

        msg
    ):

        if not self.ready:
            return

        force = np.array(

            [

                msg.wrench.force.x,

                msg.wrench.force.y,

                msg.wrench.force.z

            ]
        )

        torque = np.array(

            [

                msg.wrench.torque.x,

                msg.wrench.torque.y,

                msg.wrench.torque.z

            ]
        )

        frame = (
            msg.header.frame_id
            .split(
                '/'
            )[-1]
        )

        self.publish_thrust_forces(

            force,

            torque,

            frame
        )

        self.last_update = (
            self.get_clock()
            .now()
        )


def main(
    args=None
):

    rclpy.init(
        args=args
    )

    node = (
        ThrusterAllocatorNode()
    )

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
