
#!/usr/bin/env python3
import numpy as np
from typing import Any, Dict
from dataclasses import dataclass
import time
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist

@dataclass
class PositionUnderactuated(Node):
    def __post_init__(self):
        self.pos_des = np.zeros(3)
        self.quat_des = np.array([0, 0, 0, 1])

        self.sub_cmd = self.create_subscription(
            PoseStamped, 'cmd_pose', self.cb_cmd, 10)

        self.sub_odom = self.create_subscription(
            Odometry, 'odom', self.cb_odom, 10)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def cb_cmd(self, msg: Any) -> None:
        p = msg.pose.position
        q = msg.pose.orientation

        self.pos_des = np.array([p.x, p.y, p.z])
        self.quat_des = np.array([q.x, q.y, q.z, q.w])

    def cb_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        p = np.array([p.x, p.y, p.z])

        err = self.pos_des - p

        vx = 0.5 * np.linalg.norm(err[:2])
        vz = 0.5 * err[2]
        wz = 0.2 * np.arctan2(err[1], err[0])

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = 0.0
        cmd.linear.z = vz

        cmd.angular.z = wz

        self.pub.publish(cmd)


def main():
    rclpy.init()

    package_share_directory = get_package_share_directory('position_underac[45D[K
get_package_share_directory('position_underactuated')

    node = PositionUnderactuated()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

