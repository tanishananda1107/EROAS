#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Accel

from nav_msgs.msg import Odometry

from tf_transformations import quaternion_matrix

from PID import PIDRegulator


class VelocityControllerNode(Node):

    def __init__(self):

        super().__init__('velocity_control')

        self.declare_parameter(
            'odom_vel_in_world',
            False
        )

        self.declare_parameter(
            'linear_p',1.0)

        self.declare_parameter(
            'linear_i',0.0)

        self.declare_parameter(
            'linear_d',0.0)

        self.declare_parameter(
            'linear_sat',1.0)

        self.declare_parameter(
            'angular_p',1.0)

        self.declare_parameter(
            'angular_i',0.0)

        self.declare_parameter(
            'angular_d',0.0)

        self.declare_parameter(
            'angular_sat',1.0)

        self.v_linear_des=np.zeros(3)

        self.v_angular_des=np.zeros(3)

        self.pid_linear=PIDRegulator(
            self.get_parameter(
                'linear_p').value,
            self.get_parameter(
                'linear_i').value,
            self.get_parameter(
                'linear_d').value,
            self.get_parameter(
                'linear_sat').value
        )

        self.pid_angular=PIDRegulator(
            self.get_parameter(
                'angular_p').value,
            self.get_parameter(
                'angular_i').value,
            self.get_parameter(
                'angular_d').value,
            self.get_parameter(
                'angular_sat').value
        )

        self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10
        )

        self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.pub=self.create_publisher(
            Accel,
            'cmd_accel',
            10
        )

    def cmd_callback(self,msg):

        self.v_linear_des=np.array([
            msg.linear.x,
            msg.linear.y,
            msg.linear.z
        ])

        self.v_angular_des=np.array([
            msg.angular.x,
            msg.angular.y,
            msg.angular.z
        ])

    def odom_callback(self,msg):

        v_linear=np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])

        v_angular=np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])

        if self.get_parameter(
            'odom_vel_in_world').value:

            q=msg.pose.pose.orientation

            R=quaternion_matrix(
                [q.x,q.y,q.z,q.w]
            )[0:3,0:3].T

            v_linear=R.dot(v_linear)

            v_angular=R.dot(v_angular)

        t=self.get_clock().now().nanoseconds/1e9

        a_lin=self.pid_linear.regulate(
            self.v_linear_des-v_linear,
            t
        )

        a_ang=self.pid_angular.regulate(
            self.v_angular_des-v_angular,
            t
        )

        out=Accel()

        out.linear.x=a_lin[0]
        out.linear.y=a_lin[1]
        out.linear.z=a_lin[2]

        out.angular.x=a_ang[0]
        out.angular.y=a_ang[1]
        out.angular.z=a_ang[2]

        self.pub.publish(out)


def main():

    rclpy.init()

    node=VelocityControllerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__=="__main__":
    main()
