import numpy as np
from copy import deepcopy

from nav_msgs.msg import Odometry
from tf_transformations import (
    quaternion_from_euler,
    quaternion_matrix
)

import tf2_ros


def cross_product_operator(x):
    return np.array([
        [0, -x[2], x[1]],
        [x[2], 0, -x[0]],
        [-x[1], x[0], 0]
    ])


class Vehicle:

    def __init__(self, node, inertial_frame_id='world'):

        self.node = node
        self._namespace = node.get_namespace()

        self._inertial_frame_id = inertial_frame_id

        self._body_frame_id = (
            'base_link'
            if inertial_frame_id == 'world'
            else 'base_link_ned'
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            node
        )

        self._mass = node.declare_parameter(
            "mass", 0.0
        ).value

        self._volume = node.declare_parameter(
            "volume", 0.0
        ).value

        self._density = node.declare_parameter(
            "density",
            1028.0
        ).value

        self._gravity = 9.81

        self._cog = node.declare_parameter(
            "cog",
            [0.0,0.0,0.0]
        ).value

        self._cob = node.declare_parameter(
            "cob",
            [0.0,0.0,0.0]
        ).value

        self._inertial = node.declare_parameter(
            "inertial",
            {
                "ixx":0.0,
                "iyy":0.0,
                "izz":0.0,
                "ixy":0.0,
                "ixz":0.0,
                "iyz":0.0
            }
        ).value

        self._M=np.zeros((6,6))

        self._M[0:3,0:3]=self._mass*np.eye(3)

        self._M[0:3,3:6]=(
            -self._mass*
            cross_product_operator(
                self._cog
            )
        )

        self._M[3:6,0:3]=(
            self._mass*
            cross_product_operator(
                self._cog
            )
        )

        self._M[3:6,3:6]=(
            self._calc_inertial_tensor()
        )

        self._Ma=np.zeros((6,6))

        self._Mtotal=self._M+self._Ma

        self._linear_damping=np.zeros((6,6))

        self._quad_damping=np.zeros(6)

        self._linear_damping_forward_speed=np.zeros((6,6))

        self._pose=dict(
            pos=np.zeros(3),
            rot=quaternion_from_euler(
                0,0,0
            )
        )

        self._vel=np.zeros(6)

        self._acc=np.zeros(6)

        self._g=np.zeros(6)

    @property
    def pos(self):
        return deepcopy(
            self._pose["pos"]
        )

    @property
    def quat(self):
        return deepcopy(
            self._pose["rot"]
        )

    @property
    def vel(self):
        return deepcopy(
            self._vel
        )

    @property
    def rotBtoI(self):
        return quaternion_matrix(
            self._pose["rot"]
        )[0:3,0:3]

    @property
    def rotItoB(self):
        return self.rotBtoI.T

    def _calc_inertial_tensor(self):

        return np.array([
            [
                self._inertial["ixx"],
                self._inertial["ixy"],
                self._inertial["ixz"]
            ],
            [
                self._inertial["ixy"],
                self._inertial["iyy"],
                self._inertial["iyz"]
            ],
            [
                self._inertial["ixz"],
                self._inertial["iyz"],
                self._inertial["izz"]
            ]
        ])

    def update_odometry(
        self,
        msg:Odometry
    ):

        self._pose["pos"]=np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        self._pose["rot"]=np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

        lin=np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])

        ang=np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])

        lin=np.dot(
            self.rotItoB,
            lin
        )

        ang=np.dot(
            self.rotItoB,
            ang
        )

        self._vel=np.hstack(
            (lin,ang)
        )
