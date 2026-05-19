
import numpy as np
from tf2_ros import transformations

import rclpy
from rclpy.node import Node

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped


class Thruster:
    """
    Base thruster model
    """

    LABEL = ''

    DEFAULT_AXIS = np.array([1, 0, 0, 0])

    def __init__(self, node: Node, index, topic, pos, orientation, axis=DEF[8D[K
axis=DEFAULT_AXIS):
        self._node = node
        self._index = index
        self._topic = topic

        self._pos = None
        self._orientation = None

        self._force_dist = None

        if pos is not None and orientation is not None:
            self._pos = pos
            self._orientation = orientation

            thrust_body = transformations.quaternion_matrix(orientation).do[49D[K
transformations.quaternion_matrix(orientation).dot(axis.transpose())[0:3]
            torque_body = np.cross(pos, thrust_body)

            self._force_dist = np.hstack((thrust_body, torque_body)).transp[20D[K
torque_body)).transpose()

        self._command = 0.0
        self._thrust = 0.0

        self._command_pub = self._node.create_publisher(FloatStamped, self.[5D[K
self._topic, 10)
        self._node.get_logger().info(f'Thruster #{self._index} - {self.LABE[10D[K
{self.LABEL} - {self._topic}')

    @property
    def index(self):
        return self._index

    @property
    def topic(self):
        return self._topic

    @property
    def tam_column(self):
        return self._force_dist

    @staticmethod
    def create_thruster(model_name, *args, **kwargs):
        for thruster in Thruster.__subclasses__():
            if model_name == thruster.LABEL:
                return thruster(*args, **kwargs)
        raise RuntimeError('Invalid thruster model')

    def get_command_value(self, thrust):
        raise NotImplementedError()

    def get_thrust_value(self, command):
        raise NotImplementedError()

    def get_curve(self, min_value, max_value, n_points):
        if min_value >= max_value or n_points <= 0:
            return [], []

        input_values = np.linspace(min_value, max_value, n_points)
        output_values = []

        for value in input_values:
            output_values.append(self.get_thrust_value(value))

        return input_values.tolist(), output_values

    def _calc_command(self):
        self._command = self.get_command_value(self._thrust)

    def _update(self, thrust):
        self._thrust = thrust
        self._calc_command()

    def publish_command(self, thrust):
        self._update(thrust)
        output = FloatStamped()
        output.header.stamp = self._node.get_clock().now().to_msg()
        output.data = float(self._command)
        self._command_pub.publish(output)

removed the `catkin_python_setup()` function. I also converted the publishe[8D[K
publishers and subscribers to use the new API, and updated the Python code [K
accordingly.

