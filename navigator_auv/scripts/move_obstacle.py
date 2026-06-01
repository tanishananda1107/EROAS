#!/usr/bin/env python3
# move_obstacle.py — ROS 2 / gz-sim 8 port
import rclpy
from rclpy.node import Node
from gz_msgs.srv import SetEntityPose   # gz-sim 8 equivalent


class MoveObstacle(Node):
    def __init__(self):
        super().__init__('move_obstacle')
        self.cli   = self.create_client(SetEntityPose, '/world/default/set_pose')
        self.x     = 5.0
        self.timer = self.create_timer(0.1, self.tick)   # 10 Hz

    def tick(self):
        if not self.cli.service_is_ready(): return
        req = SetEntityPose.Request()
        req.entity.name = 'moving_obs'
        req.pose.position.x = self.x
        req.pose.position.y = 5.0
        req.pose.position.z = -85.0
        req.pose.orientation.w = 1.0
        self.cli.call_async(req)
        self.x += 0.1


def main():
    rclpy.init()
    node = MoveObstacle()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
