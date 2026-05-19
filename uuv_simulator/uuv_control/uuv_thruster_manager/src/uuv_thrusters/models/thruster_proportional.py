
import numpy as np

from .thruster import Thruster
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class ThrusterProportional(Thruster):

    LABEL = 'proportional'

    def __init__(self, node: Node, *args, **kwargs):
        super().__init__(*args)
        self._node = node

        if 'gain' not in kwargs:
            raise RuntimeError(
                'Thruster gain not given'
            )

        self._gain = kwargs['gain']

        self._node.get_logger().info(
            f'Thruster gain = {self._gain}'
        )
        self._buffer = Buffer()
        self._transform_listener = TransformListener(self._buffer)

    def get_command_value(self, thrust):
        return (
            np.sign(thrust)
            * np.sqrt(np.abs(thrust) / self._gain)
        )

    def get_thrust_value(self, command):
        return (
            self._gain
            * np.abs(command)
            * command
        )

Note that I removed the `catkin_python_setup()` and replaced it with the `i[2D[K
`install(PROGRAMS ...)` statement. I also updated the package dependencies [K
with `rclpy`, `tf` with `tf2_ros`, and removed the `rosbuild` dependency.

