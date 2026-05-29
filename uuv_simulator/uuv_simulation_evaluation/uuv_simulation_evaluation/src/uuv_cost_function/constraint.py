import numpy as np
import yaml
import os
import random


class Constraint:
    def __init__(self, tag='', input_tag=''):
        self.x = 0.0
        self.tag = tag
        self.input_tag = input_tag

        self.params = {
            'c': 0.0,
            'gain': 0.0,
            'offset': 0.0
        }

    @staticmethod
    def create(model_name, *args):
        for fcn in Constraint.__subclasses__():
            if model_name == fcn.__name__:
                return fcn(*args)

        raise Exception(f'Invalid constraint model: {model_name}')

    def from_dict(self, params):
        for tag in params:

            if tag == 'offset':
                assert isinstance(
                    params[tag], (float, int, list)
                ), f'Parameter <{tag}> is invalid'
            else:
                assert isinstance(
                    params[tag], (float, int)
                ), f'Parameter <{tag}> is invalid'

            assert tag in self.params, f'Invalid parameter tag={tag}'

            self.params[tag] = params[tag]

    def get_params(self):
        params = dict(self.params)

        params['function_name'] = self.__class__.__name__
        params['x'] = float(self.x)
        params['tag'] = self.tag
        params['input_tag'] = self.input_tag
        params['result'] = float(self.compute())

        return params

    def save(self, output_dir='.'):
        assert os.path.isdir(output_dir), 'Invalid output directory'

        try:
            filename = os.path.join(
                output_dir,
                f'{self.tag}_{self.input_tag}_{random.randint(0,1000)}.yaml'
            )

            with open(filename, 'w') as cf_file:
                yaml.safe_dump(
                    self.get_params(),
                    cf_file,
                    default_flow_style=False
                )

            return True

        except Exception as e:
            print(f'Error while storing constraint config: {e}')
            return False

    def compute(self, x=None):
        raise NotImplementedError()


class LogBarrierMethod(Constraint):
    def compute(self, x=None):

        if x is not None:
            self.x = x

        if self.x - self.params['offset'] > 0:
            return 0.0

        return (
            -1
            * self.params['c']
            * np.log(
                -1
                * self.params['gain']
                * (self.x - self.params['offset'])
            )
        )


class InverseBarrierMethod(Constraint):
    def compute(self, x=None):

        if x is not None:
            self.x = x

        d = self.params['gain'] * (
            self.x - self.params['offset']
        )

        if abs(d) < 1e-5:
            d = 1e-5 * np.sign(d)

        return -1 * self.params['c'] / d


class PenaltyFunction(Constraint):
    def __init__(self, tag='', input_tag=''):
        super().__init__(tag, input_tag)
        self.params['n'] = 0.0

    def compute(self, x=None):

        if x is not None:
            self.x = x

        if self.x - self.params['offset'] < 0:
            return 0

        return (
            self.params['c']
            * np.power(
                max(
                    0,
                    self.params['gain']
                    * (self.x - self.params['offset'])
                ),
                self.params['n']
            )
        )


class DistancePenaltyFunction(Constraint):
    def __init__(self, tag='', input_tag=''):
        super().__init__(tag, input_tag)
        self.params['n'] = 0.0

    def compute(self, x=None):

        if x is not None:
            self.x = x
        else:
            return 0

        if isinstance(self.params['offset'], list):

            return np.min([
                self.params['c']
                * np.power(
                    self.params['gain'] * np.abs(x - i),
                    self.params['n']
                )
                for i in self.params['offset']
            ])

        return (
            self.params['c']
            * np.power(
                self.params['gain']
                * np.abs(x - self.params['offset']),
                self.params['n']
            )
        )
