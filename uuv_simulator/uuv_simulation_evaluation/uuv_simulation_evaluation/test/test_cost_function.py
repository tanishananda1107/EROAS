import unittest
from uuv_cost_function import CostFunction


class TestCostFunction(unittest.TestCase):

    def setUp(self):

        self.cost_fcn = CostFunction()

        self.cost_fcn_params = dict(
            a=1.0, b=2.0, c=3.0
        )

        self.cost_fcn.from_dict(self.cost_fcn_params)

    def test_kpi_initialization(self):

        self.assertEqual(
            set(self.cost_fcn_params.keys()),
            set(self.cost_fcn.get_kpis().keys()),
        )

        for tag in self.cost_fcn_params:

            self.cost_fcn.set_kpi(tag, 0.0)
            self.cost_fcn.set_weight(tag, self.cost_fcn_params[tag])

            self.assertEqual(
                self.cost_fcn.get_weight(tag),
                self.cost_fcn_params[tag],
            )
