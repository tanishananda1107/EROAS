#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float64


class MoveSonar(Node):

    def __init__(self):

        super().__init__('move_sonar')

        self.publisher = self.create_publisher(
            Float64,
            '/rexrov2/sonar_joint_position_controller/command',
            10
        )

        self.angle = -1.0

        self.direction = 1.0

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.get_logger().info(
            'Move Sonar Started'
        )

    def timer_callback(self):

        msg = Float64()

        msg.data = self.angle

        self.publisher.publish(msg)

        self.angle += 0.05 * self.direction

        if self.angle > 1.0:

            self.direction = -1.0

        elif self.angle < -1.0:

            self.direction = 1.0


def main(args=None):

    rclpy.init(args=args)

    node = MoveSonar()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
