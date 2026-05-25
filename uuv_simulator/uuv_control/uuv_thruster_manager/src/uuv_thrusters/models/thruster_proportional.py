import numpy as np

from .thruster import Thruster


class ThrusterProportional(
    Thruster
):
    LABEL = 'proportional'

    def __init__(
        self,
        *args,
        **kwargs
    ):
        super().__init__(
            *args
        )

        if 'gain' not in kwargs:
            raise RuntimeError(
                'Thruster gain missing'
            )

        self._gain = kwargs[
            'gain'
        ]

        self._node.get_logger().info(
            'Thruster model'
        )

        self._node.get_logger().info(
            f'Gain={self._gain}'
        )

    def get_command_value(
        self,
        thrust
    ):
        return (
            np.sign(
                thrust
            )
            *
            np.sqrt(
                np.abs(
                    thrust
                )
                /
                self._gain
            )
        )

    def get_thrust_value(
        self,
        command
    ):
        return (
            self._gain
            *
            np.abs(
                command
            )
            *
            command
        )
