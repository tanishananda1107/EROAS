
#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from tf2_ros import Buffer, TransformListener
from rclpy.node import Node
from rclpy.qos import QoSProfile

class WaypointGenerator(Node):

    def __init__(self):
        super().__init__('waypoint_generator')

        self.waypoint_set = WaypointSet()
        self.waypoint_set.add_waypoint(Waypoint(-10, -12, -36, 0.5))
        self.waypoint_set.add_waypoint(Waypoint(-20, 20, -5, 0.5))
        self.waypoint_set.add_waypoint(Waypoint(-40, 80, -30, 0.5))

    def run_generator(self):
        dt = 0.05

        pnts = []

        for ti in np.arange(-2, self.get_max_time(), dt):

            tic = time.perf_counter()

            pnts.append(self.interpolate(ti))

            toc = time.perf_counter()

            print(f'Interpolation time = {toc - tic}')

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot([p.x for p in pnts], [p.y for p in pnts], [p.z for p in pnt[3D[K
pnts])
        ax.grid(True)
        plt.show()

    def get_max_time(self):
        return self.waypoint_set.get_max_time()

    def interpolate(self, ti):
        gen = WPTrajectoryGenerator(full_dof=True)
        gen.set_interp_method('cubic_interpolator')
        gen.init_waypoints(self.waypoint_set)

        return gen.interpolate(ti)


def main(args=None):

    rclpy.init(args=args)
    node = Node('waypoint_generator')
    qos = QoSProfile(depth=10)
    buf = Buffer()
    listener = TransformListener(buf, node)

    wp_gen = WaypointGenerator()
    wp_gen.run_generator()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

Note that I removed the `catkin_python_setup()` and replaced it with ament_[6D[K
ament_cmake build system. Also, replaced rospy dependencies with rclpy and [K
equivalent and updated service migration accordingly.

