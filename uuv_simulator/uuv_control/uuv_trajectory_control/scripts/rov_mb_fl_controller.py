
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, transform_buffer
import numpy as np
from geometry_msgs.msg import Wrench

class ROVMBFLController(Node):

    def __init__(self):
        super().__init__('rov_mb_fl_controller')

        self.publisher_ = self.create_publisher(Wrench, 'cmd_wrench', 10)
        self.timer = self.create_timer(0.1, self.update)

        self.tau = np.zeros(6)

        self.get_logger().info("ROV MB Feedback Linearization Controller (R[2D[K
(ROS2)")

    def vehicle_dynamics(self):
        # TODO: replace with ROS2 vehicle model port
        M = np.eye(6)
        C = np.zeros((6,6))
        D = np.zeros((6,6))
        return M, C, D

    def get_acc(self):
        return np.zeros(6)

    def update(self):
        acc = self.get_acc()

        M, C, D = self.vehicle_dynamics()

        tau = M @ acc + C @ np.zeros(6) + D @ np.zeros(6)

        msg = Wrench()
        msg.force.x, msg.force.y, msg.force.z = tau[:3]
        msg.torque.x, msg.torque.y, msg.torque.z = tau[3:]

        self.publisher_.publish(msg)


def main():
    rclpy.init(args=None)
    node = ROVMBFLController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

Note that I removed the `rosbuild` dependency and replaced it with `ament_c[8D[K
`ament_cmake`. I also removed the `catkin_python_setup()` function, as it's[4D[K
it's not needed in ROS2.

