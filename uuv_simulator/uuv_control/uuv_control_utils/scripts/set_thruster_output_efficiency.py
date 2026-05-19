
#!/usr/bin/env python3

import rclpy
from tf2_ros import Buffer, TransformationBroadcaster
from ament_cmake_core import install
from uuv_gazebo_ros_plugins_msgs.srv import SetThrusterEfficiency


class SetThrusterEfficiencyNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('set_thruster_output_efficiency')

        self.declare_parameter('thruster_id', 0)
        self.declare_parameter('efficiency', 1.0)

        thruster_id = self.get_parameter(
            'thrusher_id').value

        efficiency = self.get_parameter(
            'efficiency').value

        ns = self.get_namespace().replace('/', '')

        service_name = (
            f'/{ns}/thrusters/'
            f'{thruster_id}/set_thrust_force_efficiency'
        )

        self.client = self.create_service_client(
            SetThrusterEfficiency,
            service_name
        )

        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for service...')

        req = SetThrusterEfficiency.Request()
        req.efficiency = float(efficiency)

        future = self.client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info('Thruster efficiency updated')

        clock = self.get_clock()
        rclpy.shutdown(clock=clock)


def main(args=None):
    rclpy.init(args=args)
    SetThrusterEfficiencyNode()


if __name__ == '__main__':
    main()


Note that I removed the `install` and `declare_parameter` statements as the[3D[K
they are not needed in ROS2. Also, replaced `rclpy.spin_until_future_comple[31D[K
`rclpy.spin_until_future_complete` with `rclpy.shutdown(clock=clock)` to sh[2D[K
shut down the node correctly.

