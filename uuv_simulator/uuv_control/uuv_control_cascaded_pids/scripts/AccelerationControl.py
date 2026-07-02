#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Accel, Wrench


class AccelerationControllerNode(Node):

    def __init__(self):

        super().__init__('acceleration_control')

        self.declare_parameter('pid.mass',1.0)
        self.declare_parameter('pid.ixx', 1.0)
        self.declare_parameter('pid.ixy', 0.0)
        self.declare_parameter('pid.ixz', 0.0)
        self.declare_parameter('pid.iyy', 1.0)
        self.declare_parameter('pid.iyz', 0.0)
        self.declare_parameter('pid.izz', 1.0)

        self.mass = self.get_parameter(
            'pid.mass').value

        self.inertial_tensor=np.array([
            [
                self.get_parameter('pid.ixx').value,
                self.get_parameter('pid.ixy').value,
                self.get_parameter('pid.ixz').value
            ],
            [
                self.get_parameter('pid.ixy').value,
                self.get_parameter('pid.iyy').value,
                self.get_parameter('pid.iyz').value
            ],
            [
                self.get_parameter('pid.ixz').value,
                self.get_parameter('pid.iyz').value,
                self.get_parameter('pid.izz').value
            ]
        ])

        self.mass_inertial_matrix=np.vstack((
            np.hstack((self.mass*np.eye(3),np.zeros((3,3)))),
            np.hstack((np.zeros((3,3)),self.inertial_tensor))
        ))

        self.create_subscription(
            Accel,
            'cmd_accel',
            self.accel_callback,
            10
        )

        self.create_subscription(
            Accel,
            'cmd_force',
            self.force_callback,
            10
        )

        self.pub_force=self.create_publisher(
            Wrench,
            'thruster_manager/input',
            10
        )

    def force_callback(self,msg):

        wrench=Wrench()

        wrench.force.x=msg.linear.x
        wrench.force.y=msg.linear.y
        wrench.force.z=msg.linear.z

        wrench.torque.x=msg.angular.x
        wrench.torque.y=msg.angular.y
        wrench.torque.z=msg.angular.z

        self.pub_force.publish(wrench)

    def accel_callback(self,msg):

        accel=np.array([
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z
        ])

        tau=self.mass_inertial_matrix.dot(accel)

        wrench=Wrench()

        wrench.force.x=tau[0]
        wrench.force.y=tau[1]
        wrench.force.z=tau[2]

        wrench.torque.x=tau[3]
        wrench.torque.y=tau[4]
        wrench.torque.z=tau[5]

        self.pub_force.publish(wrench)


def main():

    rclpy.init()

    node=AccelerationControllerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__=="__main__":
    main()
