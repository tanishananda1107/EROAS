import unittest
import numpy as np

from uuv_cost_function import (
    Constraint,
    PenaltyFunction,
    LogBarrierMethod,
    InverseBarrierMethod,
    DistancePenaltyFunction,
)


class TestConstraint(unittest.TestCase):

    def setUp(self):
        self.penalty_params_valid = dict(
            gain=1.0, offset=10.0, n=1.0, c=1.0
        )

        self.log_barrier_params = dict(
            gain=1.0, offset=10.0, c=1.0
        )

        self.inverse_barrier_params = dict(
            gain=1.0, offset=10.0, c=1.0
        )

        self.dist_penalty_params_float = dict(
            gain=1.0, offset=1.0, n=1.0, c=1.0
        )

        self.dist_penalty_params_list = dict(
            gain=1.0, offset=[1.0, 2.0, 3.0], n=1.0, c=1.0
        )

    def test_create_fcn(self):

        with self.assertRaises(Exception):
            Constraint.create('InvalidModel')

        self.assertIsInstance(
            Constraint.create('LogBarrierMethod', 't', 'x'),
            LogBarrierMethod,
        )

        self.assertIsInstance(
            Constraint.create('InverseBarrierMethod', 't', 'x'),
            InverseBarrierMethod,
        )

        self.assertIsInstance(
            Constraint.create('PenaltyFunction', 't', 'x'),
            PenaltyFunction,
        )

        self.assertIsInstance(
            Constraint.create('DistancePenaltyFunction', 't', 'x'),
            DistancePenaltyFunction,
        )

    def test_penalty(self):

        p = Constraint.create('PenaltyFunction', 't', 'x')
        p.from_dict(self.penalty_params_valid)

        self.assertEqual(p.compute(0), 0)

        self.assertGreater(
            p.compute(self.penalty_params_valid['offset'] + 1),
            0,
        )

    def test_distance_penalty(self):

        p = Constraint.create(
            'DistancePenaltyFunction', 't', 'x'
        )

        p.from_dict(self.dist_penalty_params_float)

        self.assertEqual(p.compute(1), 0)

        p = Constraint.create(
            'DistancePenaltyFunction', 't', 'x'
        )

        p.from_dict(self.dist_penalty_params_list)

        self.assertEqual(p.compute(1), 0)
        self.assertEqual(p.compute(2), 0)
        self.assertEqual(p.compute(3), 0)
