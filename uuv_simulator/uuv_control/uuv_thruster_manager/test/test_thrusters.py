#!/usr/bin/env python3

import pytest
import numpy as np
import random

from tf_transformations import (
    quaternion_matrix,
    random_quaternion
)

from uuv_thrusters.models import Thruster


IDX = 0

TOPIC="/thruster"


AXES=[

np.array([1,0,0,0]),

np.array([0,1,0,0]),

np.array([0,0,1,0])

]


def get_force(pos,q,axis):

    thrust=quaternion_matrix(
        q
    ).dot(axis.T)[0:3]

    torque=np.cross(
        pos,
        thrust
    )

    return np.hstack(
        (
            thrust,
            torque
        )
    )


def test_thruster():

    for axis in AXES:

        pos=np.random.rand(3)

        q=random_quaternion()

        thruster=Thruster(

            index=IDX,

            topic=TOPIC,

            pos=pos,

            orientation=q,

            axis=axis

        )

        assert thruster.index==IDX

        assert thruster.topic==TOPIC
