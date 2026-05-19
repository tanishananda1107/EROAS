
#!/usr/bin/env python3

import numpy
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import Wrench, WrenchStamped

from uuv_thrusters import ThrusterManager

from uuv_thruster_manager.srv import (
    ThrusterManagerInfo,
    GetThrusterCurve,
    SetThrusterManagerConfig,
    GetThrusterManagerConfig
)


class ThrusterAllocatorNode(ThrusterManager, Node):
    """
    ROS2 Thruster Allocator Node
    """

    def __init__(self):
        Node.__init__(self, 'thruster_allocator')
        ThrusterManager.__init__(self)

        self.last_update = node.get_clock().now()

        # ---------------------------------------------------
        # Subscribers
        # ---------------------------------------------------

        self.input_sub = self.create_subscription(
            Wrench,
            'thruster_manager/input',
            self.input_callback,
            10
        )

        self.input_stamped_sub = self.create_subscription(
            WrenchStamped,
            'thruster_manager/input_stamped',
            self.input_stamped_callback,
            10
        )

        # ---------------------------------------------------
        # Services
        # ---------------------------------------------------

        self.create_service(
            ThrusterManagerInfo,
            'thruster_manager/get_thrusters_info',
            self.get_thruster_info
        )

        self.create_service(
            GetThrusterCurve,
            'thruster_manager/get_thruster_curve',
            self.get_thruster_curve
        )

        self.create_service(
            SetThrusterManagerConfig,
            'thruster_manager/set_config',
            self.set_config
        )

        self.create_service(
            GetThrusterManagerConfig,
            'thruster_manager/get_config',
            self.get_config
        )

        # ---------------------------------------------------
        # Timeout Timer
        # ---------------------------------------------------

        update_rate = self.config.get('update_rate', 50.0)

        self.timer = self.create_timer(
            1.0 / update_rate,
            self.timer_callback
        )

        self.get_logger().info('Thruster Allocator Node Started')

    # =====================================================
    # Timer Callback
    # =====================================================

    def timer_callback(self):

        timeout = self.config.get('timeout', -1)

        if timeout > 0:

            elapsed = (
                node.get_clock().now() - self.last_update
            ).nanoseconds / 1e9

            if elapsed > timeout:

                self.get_logger().warn(
                    'Turning thrusters off - inactive for too long'
                )

                if self.thrust is not None:
                    self.thrust.fill(0)
                    self.command_thrusters()

    # =====================================================
    # Services
    # =====================================================

    def get_thruster_info(self, request, response):

        response.n_thrusters = self.n_thrusters

        response.configuration_matrix = (
            self.configuration_matrix.flatten().tolist()
        )

        response.base_link = (
            self.namespace + self.config['base_link']
        )

        return response

    def get_thruster_curve(self, request, response):

        if self.n_thrusters == 0:
            response.input = []
            response.thrust = []
            return response

        input_values, thrust_values = self.thrusters[0].get_curve(
            request.min,
            request.max,
            request.n_points
        )

        response.input = input_values
        response.thrust = thrust_values

        return response

    def set_config(self, request, response):

        old_config = deepcopy(self.config)

        self.ready = False

        self.config['base_link'] = request.base_link
        self.config['thruster_frame_base'] = request.thruster_frame_base
        self.config['thruster_topic_prefix'] = request.thruster_topic_prefi[28D[K
request.thruster_topic_prefix
        self.config['thruster_topic_suffix'] = request.thruster_topic_suffi[28D[K
request.thruster_topic_suffix
        self.config['timeout'] = request.timeout

        self.get_logger().info('Updating Thruster Manager Configuration')

        for key in self.config:
            self.get_logger().info(f'{key} = {self.config[key]}')

        if not self.update_tam(recalculate=True):

            self.get_logger().error(
                'Invalid configuration, restoring previous config'
            )

            self.config = old_config
            self.update_tam(recalculate=True)

        response.success = True

        return response

    def get_config(self, request, response):

        response.tf_prefix = self.namespace
        response.base_link = self.config['base_link']

        response.thruster_frame_base = (
            self.config['thruster_frame_base']
        )

        response.thruster_topic_prefix = (
            self.config['thruster_topic_prefix']
        )

        response.thruster_topic_suffix = (
            self.config['thruster_topic_suffix']
        )

        response.timeout = self.config['timeout']

        response.max_thrust = self.config['max_thrust']

        response.n_thrusters = self.n_thrusters

        response.allocation_matrix = (
            self.configuration_matrix.flatten().tolist()
        )

        return response

    # =====================================================
    # Subscribers
    # =====================================================

    def input_callback(self, msg):

        if not self.ready:
            return

        force = numpy.array([
            msg.force.x,
            msg.force.y,
            msg.force.z
        ])

        torque = numpy.array([
            msg.torque.x,
            msg.torque.y,
            msg.torque.z
        ])

        self.publish_thrust_forces(force, torque)

        self.last_update = node.get_clock().now()

    def input_stamped_callback(self, msg):

        if not self.ready:
            return

        force = numpy.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z
        ])

        torque = numpy.array([
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z
        ])

        frame_id = msg.header.frame_id.split('/')[-1]

        self.publish_thrust_forces(
            force,
            torque,
            frame_id
        )

        self.last_update = node.get_clock().now()


def main(args=None):

    rclpy.init(args=args)

    try:

        node = ThrusterAllocatorNode()

        node.create_publisher()
        node.create_subscription()

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f'ThrusterAllocatorNode Exception: {e}')

    finally:

        rclpy.shutdown()

        print('Leaving ThrusterAllocatorNode')


if __name__ == '__main__':
    main()

