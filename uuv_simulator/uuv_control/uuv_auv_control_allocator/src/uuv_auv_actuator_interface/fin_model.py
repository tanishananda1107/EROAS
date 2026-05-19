
import rclpy
from tf2_ros import transformations as tft
from geometry_msgs.msg import Float32
from rclpy.node import Node


class FinModel:
    def __init__(self, node: Node, index, pos, quat, topic):
        self.id = index
        self.pos = pos
        self.quat = quat
        self.topic = topic
        self.rot = tft.quaternion_matrix(quat)[0:3, 0:3]

        unit_z = self.rot[:, 2]
        surge_vel = np.array([1, 0, 0])

        fin_surge_vel = surge_vel - (
            np.dot(surge_vel, unit_z) / np.linalg.norm(unit_z) ** 2
        ) * unit_z

        self.lift_vector = -np.cross(unit_z, fin_surge_vel)
        self.lift_vector /= np.linalg.norm(self.lift_vector)

        self.drag_vector = -surge_vel / np.linalg.norm(surge_vel)

        self.node = node
        self.pub = self.node.create_publisher(Float32, self.topic, 10)

    def publish_command(self, delta):
        msg = Float32()
        msg.data = float(delta)
        self.pub.publish(msg)

Note that I've replaced `rospy` with `rclpy`, `tf` with `tf2_ros`, and remo[4D[K
removed the `catkin_python_setup()` call.

