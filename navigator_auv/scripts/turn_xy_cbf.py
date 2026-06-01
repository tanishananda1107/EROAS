#!/usr/bin/env python3
# ROS 2 port
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SonarZeroPublisher(Node):
    def __init__(self):
        super().__init__('sonar_zero_publisher')
        self.pub   = self.create_publisher(Float64, '/rexrov2/sonar/moving', 10)
        self.timer = self.create_timer(0.1, self.publish_zero)   # 10 Hz

    def publish_zero(self):
        self.pub.publish(Float64(data=0.0))


def main():
    rclpy.init()
    node = SonarZeroPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
