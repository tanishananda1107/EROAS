#!/usr/bin/env python3
# ROS 2 / gz-sim 8 port
# gz-sim 8 uses gz.msgs and the /world/<name>/create service (SpawnEntity)
import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import Odometry
from gz_msgs.srv import SpawnEntity   # gz-sim 8


class MarkerSpawner(Node):
    def __init__(self):
        super().__init__('marker_spawner')
        self.spawn_cli  = self.create_client(SpawnEntity, '/world/default/create')
        self.last_pose  = None
        self.dist_thr   = 0.75
        self.marker_idx = 0
        self.create_subscription(Odometry, '/rexrov2/pose_gt', self.pose_callback, 10)

    @staticmethod
    def _dist(p1, p2):
        return math.sqrt((p1.position.x-p2.position.x)**2 +
                         (p1.position.y-p2.position.y)**2 +
                         (p1.position.z-p2.position.z)**2)

    def pose_callback(self, msg):
        pose = msg.pose.pose
        if self.last_pose and self._dist(pose, self.last_pose) < self.dist_thr:
            return
        self.last_pose = pose
        self._spawn(pose)

    def _spawn(self, pose):
        if not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            return
        name = f'marker_{self.marker_idx}'
        self.marker_idx += 1
        sdf = f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <visual name='v'>
        <geometry><sphere><radius>0.25</radius></sphere></geometry>
        <material><ambient>0.5 1 0.5 1</ambient></material>
      </visual>
    </link>
    <pose>{pose.position.x} {pose.position.y} {pose.position.z} 0 0 0</pose>
  </model>
</sdf>"""
        req = SpawnEntity.Request()
        req.xml = sdf
        self.spawn_cli.call_async(req)


def main():
    rclpy.init()
    node = MarkerSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
