
import numpy as np
from typing import Dict, List
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener
from geometry_msgs.msg import Twist, Accel
from nav_msgs.msg import Odometry

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_control')
        
        self.v_des = np.zeros(3)
        self.w_des = np.zeros(3)

        self.sub_cmd = self.create_subscription(
            Twist, 'cmd_vel', self.cb_cmd, 10)

        self.sub_odom = self.create_subscription(
            Odometry, 'odom', self.cb_odom, 10)

        self.pub = self.create_publisher(Accel, 'cmd_accel', 10)
        
    def cb_cmd(self, msg):
        self.v_des = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
        self.w_des = np.array([msg.angular.x, msg.angular.y, msg.angular.z][14D[K
msg.angular.z])

    def cb_odom(self, msg):

        v = msg.twist.twist.linear
        w = msg.twist.twist.angular

        v = np.array([v.x, v.y, v.z])
        w = np.array([w.x, w.y, w.z])

        err_v = self.v_des - v
        err_w = self.w_des - w

        cmd = Accel()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = err_v
        cmd.angular.x, cmd.angular.y, cmd.angular.z = err_w

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = VelocityController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

