#!/usr/bin/env python3

import pytest
import rclpy
import numpy as np

from uuv_thrusters import ThrusterManager


REFERENCE_TAM = np.array([
    [1,0,0,0,0,0],
    [0.87758256,0,-0.47942554,
     0.47942554,
     0.47942554,
     0.87758256],

    [0.87758256,
     0.47942554,
     0,
     -0.47942554,
     0.87758256,
     -0.87758256]

]).T


@pytest.fixture(scope="module")
def manager():

    rclpy.init()

    mgr = ThrusterManager()

    yield mgr

    rclpy.shutdown()


def test_thrusters(manager):

    assert manager.n_thrusters == 3

    assert np.isclose(
        REFERENCE_TAM,
        manager.configuration_matrix
    ).all()


def test_force_processing(manager):

    for _ in range(10):

        gen_force = np.random.rand(6)*100

        thrust = manager.compute_thruster_forces(
            gen_force
        )

        ref = np.linalg.pinv(
            REFERENCE_TAM
        ).dot(gen_force)

        assert np.isclose(
            thrust,
            ref
        ).all()
