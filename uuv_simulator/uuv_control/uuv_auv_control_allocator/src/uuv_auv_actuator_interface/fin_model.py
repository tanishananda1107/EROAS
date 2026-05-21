import rclpy
import numpy as np

from tf_quaternion.transformations import quaternion_matrix

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped


class FinModel:
    def __init__(self, node, index, pos, quat, topic):

        self.node = node

        self.id = index
        self.pos = pos
        self.quat = quat
        self.topic = topic

        self.rot = quaternion_matrix(quat)[0:3,0:3]

        unit_z = self.rot[:,2]

        surge_vel=np.array([1,0,0])

        fin_surge_vel = (
            surge_vel
            - np.dot(
                surge_vel,
                unit_z
            ) / np.linalg.norm(unit_z)**2 * unit_z
        )

        self.lift_vector = (
            -1
            * np.cross(
                unit_z,
                fin_surge_vel
            )
            / np.linalg.norm(
                np.cross(
                    unit_z,
                    fin_surge_vel
                )
            )
        )

        self.drag_vector = (
            -1
            * surge_vel
            / np.linalg.norm(
                surge_vel
            )
        )

        self.pub = node.create_publisher(
            FloatStamped,
            topic,
            10
        )

    def publish_command(self, delta):

        msg = FloatStamped()

        msg.header.stamp = (
            self.node.get_clock()
            .now()
            .to_msg()
        )

        msg.data=float(delta)

        self.pub.publish(msg)
