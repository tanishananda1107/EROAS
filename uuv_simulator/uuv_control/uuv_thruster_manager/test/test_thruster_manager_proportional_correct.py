
#!/usr/bin/env python3

import unittest
import numpy as np
from tf2_ros import TransformBroadcaster
import rclpy
from ament_index_python.packages import get_package_share_directory

rclpy.init()

MANAGER = ThrusterManager()

REFERENCE_TAM = np.array([
    [1, 0, 0, 0, 0, 0],
    [0.87758256, 0, -0.47942554, 0.47942554, 0.47942554, 0.87758256],
    [0.87758256, 0.47942554, 0, -0.47942554, 0.87758256, -0.87758256]
]).T


class TestThrusterManagerProportionalCorrect(unittest.TestCase):

    def test_initialization(self):

        self.assertEqual(MANAGER.namespace, '/test_vehicle/')

        self.assertEqual(
            MANAGER.config['thruster_topic_prefix'],
            'thrusters/'
        )

        self.assertEqual(
            MANAGER.config['thruster_frame_base'],
            'thruster_'
        )

        self.assertEqual(
            MANAGER.config['thruster_topic_suffix'],
            '/input'
        )

        self.assertEqual(MANAGER.config['timeout'], -1)
        self.assertEqual(MANAGER.config['max_thrust'], 1000.0)

        self.assertEqual(MANAGER.n_thrusters, 3)

        self.assertEqual(REFERENCE_TAM.shape, MANAGER.configuration_matrix.[29D[K
MANAGER.configuration_matrix.shape)

        self.assertTrue(np.isclose(REFERENCE_TAM, MANAGER.configuration_mat[25D[K
MANAGER.configuration_matrix).all())

    def test_thrusters(self):

        self.assertEqual(len(MANAGER.thrusters), 3)

        for i in range(len(MANAGER.thrusters)):
            self.assertEqual(
                MANAGER.thrusters[i].index,
                i
            )
            self.assertEqual(
                MANAGER.thrusters[i].topic,
                f'thrusters/{i}/input'
            )
            self.assertEqual(
                MANAGER.thrusters[i].LABEL,
                'proportional'
            )
            self.assertTrue(np.isclose(REFERENCE_TAM[:, i].flatten(), MANAG[5D[K
MANAGER.thrusters[i].tam_column).all())

    def test_processing_gen_forces(self):

        for _ in range(10):
            gen_force = np.random.rand(6) * 100
            thrust_forces = MANAGER.compute_thruster_forces(gen_force)
            ref_thrust_forces = np.linalg.pinv(REFERENCE_TAM).dot(gen_force[43D[K
np.linalg.pinv(REFERENCE_TAM).dot(gen_force)
            self.assertTrue(np.isclose(ref_thrust_forces, thrust_forces).al[17D[K
thrust_forces).all())

    def test_create_publisher(self):

        publisher = self.create_publisher()
        self.assertIsNotNone(publisher)

if __name__ == '__main__':
    unittest.main()
    rclpy.shutdown()

ROS2-specific equivalents.

