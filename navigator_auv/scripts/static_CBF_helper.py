#!/usr/bin/env python3
# ROS 2 port
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


class CBFHelper(Node):
    def __init__(self):
        super().__init__('CBF_Helper')
        self.pub = self.create_publisher(WrenchStamped,
                                         '/rexrov2/thruster_manager/input_stamped', 10)
        self.create_subscription(WrenchStamped,
                                  '/rexrov2/thruster_manager/input_stamped_1',
                                  self.cb1, 10)
        self.create_subscription(WrenchStamped,
                                  '/rexrov2/thruster_manager/input_stamped_2',
                                  self.cb2, 10)
        self.last_msg1 = None

    def cb1(self, msg): self.last_msg1 = msg

    def cb2(self, msg):
        if self.last_msg1 is None:
            self.pub.publish(msg)
        else:
            self.pub.publish(self.last_msg1)
            self.last_msg1 = None


def main():
    rclpy.init()
    node = CBFHelper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
