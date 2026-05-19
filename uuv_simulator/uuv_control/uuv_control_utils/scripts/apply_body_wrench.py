
#!/usr/bin/env python3

import sys
import rclpy
from tf2_ros import Buffer, TransformException
from geometry_msgs.msg import Point, Wrench, Vector3
from gazebo_msgs.srv import ApplyBodyWrench


class ApplyBodyWrenchNode(Node):
    def __init__(self):
        super().__init__('apply_body_wrench')

        self.declare_parameter('starting_time', 0.0)
        self.declare_parameter('duration', 1.0)
        self.declare_parameter('force', [0.0, 0.0, 0.0])
        self.declare_parameter('torque', [0.0, 0.0, 0.0])

        starting_time = self.get_parameter(
            'starting_time').value
        duration = self.get_parameter(
            'duration').value
        force = self.get_parameter(
            'force').value
        torque = self.get_parameter(
            'torque').value

        self.get_logger().info(f'Starting time = {starting_time}')
        self.get_logger().info(f'Duration = {duration}')
        self.get_logger().info(f'Force = {force}')
        self.get_logger().info(f'Torque = {torque}')

        self.client = self.create_client(
            ApplyBodyWrench, '/gazebo/apply_body_wrench'
        )

        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for service...')

        ns = self.get_namespace().replace('/', '')
        body_name = f'{ns}/base_link'

        request = ApplyBodyWrench.Request()

        request.body_name = body_name
        request.reference_frame = 'world'
        request.reference_point = Point(x=0.0, y=0.0, z=0.0)

        wrench = Wrench()
        wrench.force = Vector3(
            x=float(force[0]),
            y=float(force[1]),
            z=float(force[2])
        )

        wrench.torque = Vector3(
            x=float(torque[0]),
            y=float(torque[1]),
            z=float(torque[2])
        )

        request.wrench = wrench

        request.start_time.sec = int(starting_time)

        if duration >= 0:
            request.duration.sec = int(duration)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Body wrench applied')
        else:
            self.get_logger().error('Failed to apply wrench')

        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ApplyBodyWrenchNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Exception raised in {node.__class__.__nam[21D[K
{node.__class__.__name__}: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

Note that I removed the `rclpy.shutdown()` call at the end of the script, a[1D[K
as it is no longer needed in ROS2.

