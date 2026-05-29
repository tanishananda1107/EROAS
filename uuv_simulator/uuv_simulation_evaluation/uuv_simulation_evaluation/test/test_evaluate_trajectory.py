import os
import shutil
import unittest

import numpy as np

from uuv_bag_evaluation import Evaluation


PKG = 'uuv_simulation_evaluation'

ROOT_PATH = os.path.join(os.path.dirname(__file__), 'test')
RESULTS_DIR = '/tmp/results'
ROSBAG = os.path.join(ROOT_PATH, 'recording.bag')


class TestEvaluateTrajectory(unittest.TestCase):

    def setUp(self):

        os.makedirs(RESULTS_DIR, exist_ok=True)

    def tearDown(self):

        if os.path.isdir(RESULTS_DIR):
            shutil.rmtree(RESULTS_DIR)

    def test_generate_kpis(self):

        self.assertTrue(os.path.exists(ROSBAG))

        sim_eval = Evaluation(ROSBAG, RESULTS_DIR)
        sim_eval.compute_kpis()

        self.assertIsInstance(sim_eval.get_kpis(), dict)

    def test_store_kpis(self):

        self.assertTrue(os.path.exists(ROSBAG))

        sim_eval = Evaluation(ROSBAG, RESULTS_DIR)
        sim_eval.compute_kpis()
        sim_eval.save_kpis()

        self.assertTrue(
            os.path.exists(
                os.path.join(RESULTS_DIR, 'computed_kpis.yaml')
            )
        )

    def test_store_images(self):

        sim_eval = Evaluation(ROSBAG, RESULTS_DIR)
        sim_eval.compute_kpis()
        sim_eval.save_evaluation()

        self.assertTrue(os.path.isdir(RESULTS_DIR))
