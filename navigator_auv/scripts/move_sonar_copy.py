#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float64


class MoveSonarCopy(Node):

    def __init__(self):

        super().__init__('move_sonar_copy')

        self.publisher = self.create_publisher(
            Float64,
            '/rexrov2/sonar_joint_position_controller/command',
            10
        )

        self.angle = 0.0

        self.timer = self.create_timer(
            0.2,
            self.timer_callback
        )

        self.get_logger().info(
            'Move Sonar Copy Started'
        )

    def timer_callback(self):

        msg = Float64()

        msg.data = self.angle

        self.publisher.publish(msg)

        self.angle += 0.1

        if self.angle > 1.57:

            self.angle = -1.57


def main(args=None):

    rclpy.init(args=args)

    node = MoveSonarCopy()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
