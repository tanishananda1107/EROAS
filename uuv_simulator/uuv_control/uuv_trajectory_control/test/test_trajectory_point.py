#!/usr/bin/env python3

import numpy as np
import pytest

from uuv_trajectory_generator import TrajectoryPoint
from uuv_control_msgs.msg import TrajectoryPoint as TrajectoryPointMsg


def test_init_pos_vector():
    p = TrajectoryPoint()

    assert p.pos.size == 3
    assert np.array_equal(p.pos, [0, 0, 0])


def test_set_pos_vector():
    p = TrajectoryPoint()

    p.pos = [1, 2, 3]

    assert p.pos[0] == 1
    assert p.pos[1] == 2
    assert p.pos[2] == 3


def test_init_quat_vector():
    p = TrajectoryPoint()

    assert p.rotq.size == 4
    assert np.array_equal(p.rotq, [0, 0, 0, 1])


def test_to_message():
    p0 = TrajectoryPoint()

    p0.t = 1
    p0.pos = [1, 2, 3]
    p0.rotq = [0, 0, 1, 1]
    p0.vel = [1, 2, 3, 4, 5, 6]
    p0.acc = [1, 2, 3, 4, 5, 6]

    p1 = TrajectoryPoint()

    p1.from_message(p0.to_message())

    assert p0 == p1


def test_to_dict():
    p0 = TrajectoryPoint()

    p0.t = 1
    p0.pos = [1, 2, 3]
    p0.rotq = [0, 0, 1, 1]
    p0.vel = [1, 2, 3, 4, 5, 6]
    p0.acc = [1, 2, 3, 4, 5, 6]

    p1 = TrajectoryPoint()

    p1.from_dict(p0.to_dict())

    assert p0 == p1
