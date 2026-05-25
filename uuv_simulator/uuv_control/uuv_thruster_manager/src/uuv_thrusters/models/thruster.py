import numpy as np

from rclpy.node import Node
from tf_transformations import quaternion_matrix

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped


class Thruster:
    LABEL = ''

    DEFAULT_AXIS = np.array([1, 0, 0, 0])

    def __init__(
        self,
        node: Node,
        index,
        topic,
        pos,
        orientation,
        axis=DEFAULT_AXIS
    ):
        self._node = node

        self._index = index
        self._topic = topic

        self._pos = None
        self._orientation = None
        self._force_dist = None

        if pos is not None and orientation is not None:
            self._pos = pos
            self._orientation = orientation

            thrust_body = quaternion_matrix(
                orientation
            ).dot(axis.T)[0:3]

            torque_body = np.cross(
                pos,
                thrust_body
            )

            self._force_dist = np.hstack(
                (
                    thrust_body,
                    torque_body
                )
            ).T

        self._command = 0.0
        self._thrust = 0.0

        self._command_pub = node.create_publisher(
            FloatStamped,
            topic,
            10
        )

        node.get_logger().info(
            f'Thruster #{index} - {self.LABEL} - {topic}'
        )

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
    def create_thruster(
        model_name,
        *args,
        **kwargs
    ):
        for thruster in Thruster.__subclasses__():
            if model_name == thruster.LABEL:
                return thruster(
                    *args,
                    **kwargs
                )

        raise RuntimeError(
            'Invalid thruster model'
        )

    def get_command_value(
        self,
        thrust
    ):
        raise NotImplementedError

    def get_thrust_value(
        self,
        command
    ):
        raise NotImplementedError

    def get_curve(
        self,
        min_value,
        max_value,
        n_points
    ):
        if (
            min_value >= max_value
            or n_points <= 0
        ):
            return [], []

        input_values = np.linspace(
            min_value,
            max_value,
            n_points
        )

        output_values = []

        for value in input_values:
            output_values.append(
                self.get_thrust_value(
                    value
                )
            )

        return (
            input_values.tolist(),
            output_values
        )

    def _calc_command(self):
        self._command = self.get_command_value(
            self._thrust
        )

    def _update(
        self,
        thrust
    ):
        self._thrust = thrust
        self._calc_command()

    def publish_command(
        self,
        thrust
    ):
        self._update(
            thrust
        )

        msg = FloatStamped()

        msg.header.stamp = (
            self._node.get_clock()
            .now()
            .to_msg()
        )

        msg.data = float(
            self._command
        )

        self._command_pub.publish(
            msg
        )
