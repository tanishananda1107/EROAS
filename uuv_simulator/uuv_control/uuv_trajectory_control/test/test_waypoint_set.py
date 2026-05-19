
#!/usr/bin/env python
import sys
import unittest
from uuv_waypoints import Waypoint, WaypointSet

from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TestWaypointSet(Node):
    def __init__(self):
        super().__init__('test_waypoint_set')
        self.get_logger().info('Initializing...')

    def test_init(self):
        wp_set = WaypointSet()
        self.assertEqual(wp_set.num_waypoints, 0, 'Waypoint list is not emp[3D[K
empty')

    def test_invalid_params_helix(self):
        wp_set = WaypointSet()
        self.assertFalse(wp_set.generate_helix(
            radius=-1.0,
            center=None,
            num_points=-1,
            max_forward_speed=0.0,
            delta_z=1,
            num_turns=-1,
            theta_offset=0.0,
            heading_offset=0.0), 'Invalid parameters have been wrongly inst[4D[K
instantiated')

    def test_invalid_params_circle(self):
        wp_set = WaypointSet()
        self.assertFalse(wp_set.generate_circle(
            radius=-1,
            center=None,
            num_points=-1,
            max_forward_speed=0,
            theta_offset=0.0,
            heading_offset=0.0), 'Invalid parameters have been wrongly inst[4D[K
instantiated')

    def test_add_repeated_waypoint(self):
        wp = Waypoint(x=1, y=2, z=3, max_forward_speed=1)
        wp_set = WaypointSet()
        self.assertTrue(wp_set.add_waypoint(wp),
            'Error occured while adding waypoint to empty set')
        self.assertFalse(wp_set.add_waypoint(wp),
            'Repeated waypoint wrongfully added')

    def test_publish(self):
        publisher = self.create_publisher(Waypoint, 'waypoints_topic', 10)
        wp = Waypoint(x=1, y=2, z=3, max_forward_speed=1)
        publisher.publish(wp)

if __name__ == '__main__':
    import unittest
    unittest.main()

Please note that I have assumed the `Waypoint` and `WaypointSet` classes ar[2D[K
are identical in both ROS 1 and ROS 2. If there are any changes needed to t[1D[K
these classes, you will need to modify them accordingly.

