# Copyright (c) 2016-2019 The UUV Simulator Authors.
# Licensed under Apache License 2.0

import numpy as np

from builtin_interfaces.msg import Time

from uuv_control_msgs.msg import (
    TrajectoryPoint as TrajectoryPointMsg
)

import geometry_msgs.msg as geometry_msgs

from tf_quaternion.transformations import (
    quaternion_from_euler,
    euler_from_quaternion,
    quaternion_matrix
)


class TrajectoryPoint(object):

    def __init__(
        self,
        t=0.0,
        pos=[0, 0, 0],
        quat=[0, 0, 0, 1],
        lin_vel=[0, 0, 0],
        ang_vel=[0, 0, 0],
        lin_acc=[0, 0, 0],
        ang_acc=[0, 0, 0]
    ):

        self._pos = np.array(pos)

        self._rot = np.array(quat)

        self._vel = np.hstack(
            (lin_vel, ang_vel)
        )

        self._acc = np.hstack(
            (lin_acc, ang_acc)
        )

        self._t = t

    @property
    def p(self):
        return self._pos

    @property
    def q(self):
        return self._rot

    @property
    def v(self):
        return self._vel[0:3]

    @property
    def w(self):
        return self._vel[3:]

    @property
    def a(self):
        return self._acc[0:3]

    @property
    def alpha(self):
        return self._acc[3:]

    @property
    def t(self):
        return self._t

    @t.setter
    def t(self, val):
        self._t = val

    @property
    def pos(self):
        return self._pos

    @pos.setter
    def pos(self, val):
        self._pos = np.array(val)

    @property
    def rot(self):

        rpy = euler_from_quaternion(
            self._rot
        )

        return np.array(
            [rpy[0], rpy[1], rpy[2]]
        )

    @rot.setter
    def rot(self, val):

        self._rot = quaternion_from_euler(
            *val
        )

    @property
    def rot_matrix(self):

        return quaternion_matrix(
            self._rot
        )[0:3, 0:3]

    @property
    def rotq(self):
        return self._rot

    @rotq.setter
    def rotq(self, q):
        self._rot = np.array(q)

    @property
    def vel(self):
        return self._vel

    @vel.setter
    def vel(self, v):
        self._vel = np.array(v)

    @property
    def acc(self):
        return self._acc

    @acc.setter
    def acc(self, a):
        self._acc = np.array(a)

    def to_message(self):

        msg = TrajectoryPointMsg()

        sec = int(self.t)

        nanosec = int(
            (self.t - sec) * 1e9
        )

        msg.header.stamp = Time(
            sec=sec,
            nanosec=nanosec
        )

        msg.pose.position = geometry_msgs.Vector3(
            x=float(self.p[0]),
            y=float(self.p[1]),
            z=float(self.p[2])
        )

        msg.pose.orientation = geometry_msgs.Quaternion(
            x=float(self.q[0]),
            y=float(self.q[1]),
            z=float(self.q[2]),
            w=float(self.q[3])
        )

        msg.velocity.linear = geometry_msgs.Vector3(
            x=float(self.v[0]),
            y=float(self.v[1]),
            z=float(self.v[2])
        )

        msg.velocity.angular = geometry_msgs.Vector3(
            x=float(self.w[0]),
            y=float(self.w[1]),
            z=float(self.w[2])
        )

        msg.acceleration.linear = geometry_msgs.Vector3(
            x=float(self.a[0]),
            y=float(self.a[1]),
            z=float(self.a[2])
        )

        msg.acceleration.angular = geometry_msgs.Vector3(
            x=float(self.alpha[0]),
            y=float(self.alpha[1]),
            z=float(self.alpha[2])
        )

        return msg
