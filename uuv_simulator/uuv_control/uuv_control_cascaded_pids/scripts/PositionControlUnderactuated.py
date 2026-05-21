#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

import tf_transformations as trans

from PID import PIDRegulator


class PositionControllerNode(Node):

    def __init__(self):

        super().__init__(
            'position_control_underactuated'
        )

        self.pos_des = np.zeros(3)

        self.quat_des = np.array(
            [0,0,0,1]
        )

        self.initialized=False

        self.declare_parameters(
            namespace='',
            parameters=[

                ('forward_p',1.0),
                ('forward_i',0.0),
                ('forward_d',0.0),
                ('forward_sat',1.0),

                ('depth_p',1.0),
                ('depth_i',0.0),
                ('depth_d',0.0),
                ('depth_sat',1.0),

                ('heading_p',1.0),
                ('heading_i',0.0),
                ('heading_d',0.0),
                ('heading_sat',1.0)
            ]
        )

        self.create_pid()

        self.add_on_set_parameters_callback(
            self.parameter_callback
        )

        self.sub_pose = (
            self.create_subscription(
                PoseStamped,
                'cmd_pose',
                self.cmd_pose_callback,
                10
            )
        )

        self.sub_odom = (
            self.create_subscription(
                Odometry,
                'odom',
                self.odometry_callback,
                10
            )
        )

        self.pub_cmd_vel = (
            self.create_publisher(
                Twist,
                'cmd_vel',
                10
            )
        )

    def create_pid(self):

        self.pid_forward=PIDRegulator(
            self.get_parameter(
                'forward_p').value,

            self.get_parameter(
                'forward_i').value,

            self.get_parameter(
                'forward_d').value,

            self.get_parameter(
                'forward_sat').value
        )

        self.pid_depth=PIDRegulator(
            self.get_parameter(
                'depth_p').value,

            self.get_parameter(
                'depth_i').value,

            self.get_parameter(
                'depth_d').value,

            self.get_parameter(
                'depth_sat').value
        )

        self.pid_heading=PIDRegulator(
            self.get_parameter(
                'heading_p').value,

            self.get_parameter(
                'heading_i').value,

            self.get_parameter(
                'heading_d').value,

            self.get_parameter(
                'heading_sat').value
        )

    def parameter_callback(self,params):

        self.create_pid()

        return SetParametersResult(
            successful=True
        )

    def cmd_pose_callback(self,msg):

        p=msg.pose.position

        q=msg.pose.orientation

        self.pos_des=np.array(
            [p.x,p.y,p.z]
        )

        self.quat_des=np.array(
            [q.x,q.y,q.z,q.w]
        )

    def odometry_callback(self,msg):

        p=msg.pose.pose.position

        q=msg.pose.pose.orientation

        pos=np.array(
            [p.x,p.y,p.z]
        )

        quat=np.array(
            [q.x,q.y,q.z,q.w]
        )

        if not self.initialized:

            self.pos_des=pos

            self.quat_des=quat

            self.initialized=True

            return

        t=(
            msg.header.stamp.sec+
            msg.header.stamp.nanosec*1e-9
        )

        R=trans.quaternion_matrix(
            quat
        )[0:3,0:3]

        e_pos=(
            R.T @
            (self.pos_des-pos)
        )

        vz=self.pid_depth.regulate(
            e_pos[2],
            t
        )

        vx=self.pid_forward.regulate(
            np.linalg.norm(
                e_pos[0:2]
            ),
            t
        )

        heading=np.arctan2(
            e_pos[1],
            e_pos[0]
        )

        wz=self.pid_heading.regulate(
            heading,
            t
        )

        cmd=Twist()

        cmd.linear=Vector3(
            x=float(vx),
            y=0.0,
            z=float(vz)
        )

        cmd.angular=Vector3(
            x=0.0,
            y=0.0,
            z=float(wz)
        )

        self.pub_cmd_vel.publish(
            cmd
        )


def main(args=None):

    rclpy.init(args=args)

    node=PositionControllerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__=="__main__":
    main()
