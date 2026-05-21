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

        super().__init__('position_control')

        self.get_logger().info(
            'PositionControllerNode initialized')

        self.pos_des = np.zeros(3)

        self.quat_des = np.array(
            [0.0, 0.0, 0.0, 1.0]
        )

        self.initialized = False

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pos_p',1.0),
                ('pos_i',0.0),
                ('pos_d',0.0),
                ('pos_sat',1.0),

                ('rot_p',1.0),
                ('rot_i',0.0),
                ('rot_d',0.0),
                ('rot_sat',1.0)
            ]
        )

        self.create_pid()

        self.add_on_set_parameters_callback(
            self.parameter_callback
        )

        self.sub_pose = self.create_subscription(
            PoseStamped,
            'cmd_pose',
            self.cmd_pose_callback,
            10
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            10
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

    def create_pid(self):

        self.pid_pos = PIDRegulator(
            self.get_parameter(
                'pos_p').value,

            self.get_parameter(
                'pos_i').value,

            self.get_parameter(
                'pos_d').value,

            self.get_parameter(
                'pos_sat').value
        )

        self.pid_rot = PIDRegulator(
            self.get_parameter(
                'rot_p').value,

            self.get_parameter(
                'rot_i').value,

            self.get_parameter(
                'rot_d').value,

            self.get_parameter(
                'rot_sat').value
        )

    def parameter_callback(self, params):

        self.create_pid()

        return SetParametersResult(
            successful=True
        )

    def cmd_pose_callback(self,msg):

        p = msg.pose.position

        q = msg.pose.orientation

        self.pos_des = np.array(
            [p.x,p.y,p.z]
        )

        self.quat_des = np.array(
            [q.x,q.y,q.z,q.w]
        )

    def odometry_callback(self,msg):

        p = msg.pose.pose.position

        q = msg.pose.pose.orientation

        pos = np.array(
            [p.x,p.y,p.z]
        )

        quat = np.array(
            [q.x,q.y,q.z,q.w]
        )

        if not self.initialized:

            self.pos_des = pos
            self.quat_des = quat

            self.initialized = True

            return

        t = (
            msg.header.stamp.sec +
            msg.header.stamp.nanosec*1e-9
        )

        e_pos_world = self.pos_des - pos

        R = trans.quaternion_matrix(
            quat
        )[0:3,0:3]

        e_pos_body = (
            R.T @ e_pos_world
        )

        e_rot_quat = (
            trans.quaternion_multiply(
                trans.quaternion_conjugate(
                    quat
                ),
                self.quat_des
            )
        )

        if np.linalg.norm(
            e_pos_world[0:2]
        ) > 5.0:

            heading = math.atan2(
                e_pos_world[1],
                e_pos_world[0]
            )

            quat_goal = np.array([
                0,
                0,
                math.sin(
                    heading/2
                ),
                math.cos(
                    heading/2
                )
            ])

            e_rot_quat = (
                trans.quaternion_multiply(
                    trans.quaternion_conjugate(
                        quat
                    ),
                    quat_goal
                )
            )

        e_rot = np.array(
            trans.euler_from_quaternion(
                e_rot_quat
            )
        )

        v_linear = (
            self.pid_pos.regulate(
                e_pos_body,
                t
            )
        )

        v_angular = (
            self.pid_rot.regulate(
                e_rot,
                t
            )
        )

        cmd = Twist()

        cmd.linear = Vector3(
            x=float(v_linear[0]),
            y=float(v_linear[1]),
            z=float(v_linear[2])
        )

        cmd.angular = Vector3(
            x=float(v_angular[0]),
            y=float(v_angular[1]),
            z=float(v_angular[2])
        )

        self.pub_cmd_vel.publish(
            cmd
        )


def main(args=None):

    rclpy.init(args=args)

    node = PositionControllerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__=="__main__":
    main()
