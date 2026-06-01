#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelForwarder(Node):
    def __init__(self):
        super().__init__('cmd_vel_forwarder')
        self.pub  = self.create_publisher(Twist, '/rexrov2/cmd_vel', 1)
        self.create_subscription(Twist, '/rexrov2/cmd_vel_heading',
                                  self.cb, 10)
        self.latest = Twist()
        self.timer  = self.create_timer(1.0/100.0, self.publish)   # 100 Hz

    def cb(self, msg): self.latest = msg
    def publish(self):  self.pub.publish(self.latest)

def main():
    rclpy.init()
    node = CmdVelForwarder()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
