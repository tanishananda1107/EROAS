
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Wrench
from tf2_ros import Buffer, TransformListener

class ROVSFController(Node):

    def __init__(self):
        super().__init__('rov_sf_controller')

        self.pub = self.create_publisher(Wrench, 'cmd_wrench', 10)
        self.timer = self.create_timer(rclpy.duration_seconds(0.1), self.up[7D[K
self.update)

        self.Kd = np.eye(6)

    def update(self):
        e = np.zeros(6)
        de = np.zeros(6)

        s = de + e

        tau = self.Kd @ s

        msg = Wrench()
        msg.force.x, msg.force.y, msg.force.z = tau[:3]
        msg.torque.x, msg.torque.y, msg.torque.z = tau[3:]

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ROVSFController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

Changes made:

1. Replaced `rospy` with `rclpy`.
2. Imported `tf2_ros` for TF-related operations.
3. Removed `catkin_python_setup()` and replaced it with the equivalent `ame[4D[K
`ament_cmake` setup.
4. Updated package.xml to use `ament_cmake` instead of `catkin`.

