
import rclpy
from tf2_ros import Buffer, TransformException
import numpy as np


class LineSegment:
    def __init__(self, p_init, p_target):
        if type(p_init) == list:
            self._p_init = np.array(p_init)
        elif type(p_init) == np.ndarray:
            self._p_init = p_init
        else:
            raise TypeError('Initial point is neither a list or an array')

        if type(p_target) == list:
            self._p_target = np.array(p_target)
        elif type(p_target) == np.ndarray:
            self._p_target = p_target
        else:
            raise TypeError('Final point is neither a list or an array')

    def interpolate(self, u):
        u = max(u, 0)
        u = min(u, 1)
        return (1 - u) * self._p_init + u * self._p_target

    def get_derivative(self):
        return self._p_target - self._p_init

    def get_length(self):
        return np.linalg.norm(self._p_target - self._p_init)

    def get_tangent(self):
        return (self._p_target - self._p_init) / self.get_length()

Note: I've removed the `ros1` related imports and code, and replaced them w[1D[K
with equivalent ROS2 Jazzy compatible code.

