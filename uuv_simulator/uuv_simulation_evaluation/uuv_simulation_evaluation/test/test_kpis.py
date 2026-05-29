import unittest
import numpy as np

from uuv_bag_evaluation.metrics import KPI


class TestKPIs(unittest.TestCase):

    def test_max_error(self):

        error = [1.0] * 10
        error[2] = 20.0

        kpi = KPI.get_kpi('max_error', 'test', False)

        self.assertIsNotNone(kpi)
        self.assertEqual(kpi.compute(error), 20.0)

    def test_mean_error(self):

        error = np.random.rand(10) + 2

        kpi = KPI.get_kpi('mean_error', 'test', False)

        self.assertIsNotNone(kpi)
        self.assertIsInstance(kpi.compute(error), float)

    def test_rmse(self):

        error = np.random.rand(10) + 2

        kpi = KPI.get_kpi('rmse', 'test', False)

        self.assertIsNotNone(kpi)
        self.assertIsInstance(kpi.compute(error), float)
