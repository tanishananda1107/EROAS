import numpy as np

from .thruster import Thruster


class ThrusterCustom(
    Thruster
):
    LABEL = 'custom'

    def __init__(
        self,
        *args,
        **kwargs
    ):
        super().__init__(
            *args
        )

        if (
            'input'
            not in kwargs
            or 'output'
            not in kwargs
        ):
            raise RuntimeError(
                'Thruster input/output sample points missing'
            )

        self._input = kwargs[
            'input'
        ]

        self._output = kwargs[
            'output'
        ]

    def get_command_value(
        self,
        thrust
    ):
        return np.interp(
            thrust,
            self._output,
            self._input
        )

    def get_thrust_value(
        self,
        command
    ):
        return np.interp(
            command,
            self._input,
            self._output
        )
