import yaml
import os
import sys
import logging
import numpy as np

from .constraint import Constraint


class CostFunction:

    def __init__(self, norm=1):

        self.logger = logging.getLogger('cost_function')

        if not self.logger.handlers:

            out_hdlr = logging.StreamHandler(sys.stdout)

            formatter = logging.Formatter(
                '%(asctime)s | %(levelname)s | %(module)s | %(message)s'
            )

            out_hdlr.setFormatter(formatter)
            out_hdlr.setLevel(logging.INFO)

            self.logger.addHandler(out_hdlr)

            os.makedirs('logs', exist_ok=True)

            file_hdlr = logging.FileHandler(
                os.path.join('logs', 'cost_function.log')
            )

            file_hdlr.setFormatter(formatter)
            file_hdlr.setLevel(logging.INFO)

            self.logger.addHandler(file_hdlr)

            self.logger.setLevel(logging.INFO)

        self.norm = np.inf if norm == 'inf' else norm

        self.kpis = {}
        self.weights = {}
        self.constraints = []

        self.export_data = {
            'norm': self.norm
        }

    def set_norm(self, norm):

        if norm == 'inf':
            self.norm = np.inf
        else:
            assert isinstance(norm, int)
            self.norm = norm

    def add_constraints(self, constraints):

        for c in constraints:

            if not self.add_constraint(
                c['type'],
                c['tag'],
                c['input_tag'],
                c['params']
            ):
                return False

        return True

    def add_constraint(self, fcn_name, tag, input_tag, params):

        try:
            c_fcn = Constraint.create(
                fcn_name,
                tag,
                input_tag
            )

            c_fcn.from_dict(params)

            self.constraints.append(c_fcn)

            self.logger.info(f'Constraint model <{fcn_name}> added')

            return True

        except Exception as e:

            self.logger.error(
                f'Error adding constraint {fcn_name}: {e}'
            )

            return False

    def is_kpi(self, tag):
        return tag in self.kpis

    def set_kpi(self, tag, value):
        self.kpis[tag] = value

    def set_weight(self, tag, weight):
        self.weights[tag] = weight

    def compute(self):

        costs = []

        self.logger.info('Calculating cost function')

        if len(self.weights) == 0:
            return 0.0

        w = 1.0 / len(self.weights.keys())

        for tag in sorted(self.weights.keys()):

            if self.kpis[tag] < 0:
                raise Exception(
                    f'KPI <{tag}> has invalid value={self.kpis[tag]}'
                )

            value = (
                w
                * self.weights[tag]
                * self.kpis[tag]
            )

            costs.append(value)

            self.export_data[f'weight_{tag}'] = float(
                w * self.weights[tag]
            )

            self.export_data[tag] = float(self.kpis[tag])

            self.export_data[f'cost_{tag}'] = float(value)

        total_cost = np.linalg.norm(
            costs,
            ord=self.norm
        )

        total_cost += self.compute_constraints()

        self.export_data['total_cost'] = float(total_cost)

        return total_cost

    def compute_constraints(self):

        value = 0.0

        for c in self.constraints:

            if c.input_tag not in self.kpis:

                raise Exception(
                    f'{c.input_tag} tag not in KPIs'
                )

            c_fcn = c.compute(
                self.kpis[c.input_tag]
            )

            self.export_data[c.tag] = float(c_fcn)

            value += c_fcn

        return value

    def save(self, output_dir='.'):

        assert os.path.isdir(output_dir)

        try:

            filename = os.path.join(
                output_dir,
                'cost_function.yaml'
            )

            with open(filename, 'w') as cf_file:
                yaml.safe_dump(
                    self.weights,
                    cf_file,
                    default_flow_style=False
                )

            for c in self.constraints:
                c.save(output_dir)

            return True

        except Exception as e:

            self.logger.error(
                f'Error storing configuration: {e}'
            )

            return False

    def get_data(self):
        return self.export_data


if __name__ == '__main__':

    cf = CostFunction()

    cf.set_weight('rmse_yaw', 10.0)

    cf.set_kpi('rmse_yaw', 10.0)
    cf.set_kpi('rmse_pitch', 12.0)

    print(cf.compute())
