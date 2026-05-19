
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Path
from uuv_control_msgs.msg import Trajectory, TrajectoryPoint, WaypointSet
import ament_index

class TrajectoryMarkerPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_marker_publisher')

        self.trajectory = None

        self.create_subscription(
            Trajectory,
            'trajectory',
            self.trajectory_callback,
            10
        )

        self.path_pub = self.create_publisher(Path, 'trajectory_marker', 10[2D[K
10)
        self.marker_pub = self.create_publisher(Marker, 'reference_marker',[19D[K
'reference_marker', 10)

        self.timer = self.create_timer(0.5, self.publish_markers)

    def trajectory_callback(self, msg):
        self.trajectory = msg

    def publish_markers(self):

        path = Path()
        path.header.frame_id = 'world'
        path.header.stamp = self.get_clock().now().to_msg()

        if self.trajectory is not None:
            for pnt in self.trajectory.points:

                pose = PoseStamped()
                pose.header = pnt.header
                pose.pose = pnt.pose

                path.poses.append(pose)

        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryMarkerPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

