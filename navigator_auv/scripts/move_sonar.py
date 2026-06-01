#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class MoveSonar(Node):
    def __init__(self):
        super().__init__('move_sonar')
        self.pub   = self.create_publisher(Float64,
                         '/rexrov2/sonar_joint_position_controller/command', 10)
        self.timer = self.create_timer(0.1, self.tick)   # 10 Hz

    def tick(self):
        self.pub.publish(Float64(data=0.6))   # fixed position variant

def main():
    rclpy.init()
    node = MoveSonar()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
