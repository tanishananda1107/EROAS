
#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformationException

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

class PositionControllerNode(Node):

    def __init__(self):
        super().__init__('position_control')

        self.pos_des = np.zeros(3)
        self.quat_des = np.array([0, 0, 0, 1])
        self.initialized = False

        self.buffer = Buffer()
        self.sub_cmd_pose = self.create_subscription(
            PoseStamped, 'cmd_pose', self.cmd_pose_callback, 10)

        self.sub_odom = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        self.pid_pos = [1.0, 0.0, 0.0]
        self.pid_rot = [1.0, 0.0, 0.0]

    def cmd_pose_callback(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation

        self.pos_des = np.array([p.x, p.y, p.z])
        self.quat_des = np.array([q.x, q.y, q.z, q.w])

    def odom_callback(self, msg):

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        p = np.array([p.x, p.y, p.z])
        q = np.array([q.x, q.y, q.z, q.w])

        if not self.initialized:
            self.initialized = True
            self.pos_des = p
            self.quat_des = q

        pos_err = self.pos_des - p

        q_err = quaternion_multiply(quaternion_conjugate(q), self.quat_des)[14D[K
self.quat_des)
        rot_err = np.array(euler_from_quaternion(q_err))

        v_linear = self.pid_pos[0] * pos_err
        v_angular = self.pid_rot[0] * rot_err

        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = v_linear
        cmd.angular.x, cmd.angular.y, cmd.angular.z = v_angular

        self.pub_vel.publish(cmd)

    def get_clock(self):
        return self.get_clock()

def main():
    rclpy.init()
    node = PositionControllerNode()
    try:
        rclpy.spin(node)
    except rclpy.exceptions.ROSInterruptException as e:
        node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

