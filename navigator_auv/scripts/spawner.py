#!/usr/bin/env python3
# ROS 2 / gz-sim 8 port
# gz-sim 8 uses gz.msgs and the /world/<name>/create service (SpawnEntity)
import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import Odometry
from ros_gz_interfaces.srv import SpawnEntity


class MarkerSpawner(Node):
    def __init__(self):
        super().__init__('marker_spawner')
        self.declare_parameter('world_name', 'oceans_waves')
        self.declare_parameter('pose_topic', '/rexrov2/pose_gt')
        self.declare_parameter('distance_threshold', 0.22)
        self.declare_parameter('marker_radius', 0.14)
        self.declare_parameter('initial_delay', 1.5)

        world_name = self.get_parameter('world_name').value
        pose_topic = self.get_parameter('pose_topic').value

        self.spawn_cli  = self.create_client(SpawnEntity, f'/world/{world_name}/create')
        self.last_pose  = None
        self.dist_thr   = float(self.get_parameter('distance_threshold').value)
        self.radius     = float(self.get_parameter('marker_radius').value)
        self.initial_delay_ns = int(
            float(self.get_parameter('initial_delay').value) * 1e9)
        self.first_pose_time = None
        self.marker_idx = 0
        self.spawn_pending = False
        self.create_subscription(Odometry, pose_topic, self.pose_callback, 10)

    @staticmethod
    def _dist(p1, p2):
        return math.hypot(p1.position.x - p2.position.x,
                          p1.position.y - p2.position.y)

    def pose_callback(self, msg):
        pose = msg.pose.pose
        now = self.get_clock().now()
        if self.first_pose_time is None:
            self.first_pose_time = now
        if (now - self.first_pose_time).nanoseconds < self.initial_delay_ns:
            self.last_pose = pose
            return
        if self.spawn_pending:
            return
        if self.last_pose is None:
            self.last_pose = pose
            return
        if self.last_pose and self._dist(pose, self.last_pose) < self.dist_thr:
            return
        self._spawn(self.last_pose, pose)

    def _spawn(self, marker_pose, next_pose):
        if not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('Waiting for Gazebo entity-create service')
            return

        dx = next_pose.position.x - marker_pose.position.x
        dy = next_pose.position.y - marker_pose.position.y
        distance = math.hypot(dx, dy)
        if distance < 1e-3:
            self.last_pose = next_pose
            return

        name = f'trail_{self.marker_idx}'
        self.marker_idx += 1
        yaw = math.atan2(dy, dx)
        length = distance + 2.0 * self.radius
        center_x = 0.5 * (marker_pose.position.x + next_pose.position.x)
        center_y = 0.5 * (marker_pose.position.y + next_pose.position.y)
        center_z = 0.5 * (marker_pose.position.z + next_pose.position.z)

        sdf = f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <visual name='tube'>
        <geometry>
          <cylinder>
            <radius>{self.radius}</radius>
            <length>{length}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.0 0.55 0.35 1</ambient>
          <diffuse>0.0 0.95 0.45 1</diffuse>
          <emissive>0.0 0.18 0.10 1</emissive>
        </material>
        <cast_shadows>false</cast_shadows>
      </visual>
      <visual name='start_cap'>
        <pose>0 0 {-0.5 * length} 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>{self.radius}</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.0 0.55 0.35 1</ambient>
          <diffuse>0.0 0.95 0.45 1</diffuse>
          <emissive>0.0 0.18 0.10 1</emissive>
        </material>
        <cast_shadows>false</cast_shadows>
      </visual>
      <visual name='end_cap'>
        <pose>0 0 {0.5 * length} 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>{self.radius}</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.0 0.55 0.35 1</ambient>
          <diffuse>0.0 0.95 0.45 1</diffuse>
          <emissive>0.0 0.18 0.10 1</emissive>
        </material>
        <cast_shadows>false</cast_shadows>
      </visual>
    </link>
    <pose>{center_x} {center_y} {center_z} 0 1.57079632679 {yaw}</pose>
  </model>
</sdf>"""
        req = SpawnEntity.Request()
        req.entity_factory.name = name
        req.entity_factory.sdf = sdf
        self.spawn_pending = True
        future = self.spawn_cli.call_async(req)
        future.add_done_callback(
            lambda completed, vehicle_pose=next_pose, marker_name=name:
                self._spawn_done(completed, vehicle_pose, marker_name)
        )

    def _spawn_done(self, future, pose, name):
        self.spawn_pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to spawn {name}: {exc}')
            return
        if not response.success:
            self.get_logger().warning(f'Gazebo rejected {name}')
            return
        self.last_pose = pose


def main():
    rclpy.init()
    node = MarkerSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
