#!/usr/bin/env python3
# ROS 2 port
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import tf_transformations as tft
from matplotlib.offsetbox import OffsetImage, AnnotationBbox


class PosePlotter(Node):
    def __init__(self, bg_path, plane_path, ship_path):
        super().__init__('pose_plotter')
        self.bg    = mpimg.imread(bg_path)
        self.plane = mpimg.imread(plane_path)
        self.ship  = mpimg.imread(ship_path)
        self.x_data, self.y_data = [], []
        self.plane_present = self.shipwreck_present = False

        self.create_subscription(Odometry, '/rexrov2/pose_gt',    self.pose_callback,     10)
        self.create_subscription(Float64,  '/rexrov/obstacle',    self.obstacle_callback, 10)

        self.fig, self.ax = plt.subplots()
        self.ax.imshow(self.bg, extent=[-100,100,-70,270], aspect='equal')
        self.line, = self.ax.plot([], [], 'b-', label='Trajectory')
        self.ax.plot([-55,-55,55,55],[160,-50,-50,200],'r--',label='Desired Track')
        for lbl,xi,yi in zip(['S','A','B','C'],[-55,-55,55,55],[160,-50,-50,200]):
            self.ax.annotate(lbl,(xi,yi),xytext=(5,-5),textcoords='offset points',color='red')
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.legend(); self.ax.grid()
        self.timer = self.create_timer(0.1, self.update_plot)
        plt.ion(); plt.show()

    def pose_callback(self, msg):
        self.x_data.append(msg.pose.pose.position.x)
        self.y_data.append(msg.pose.pose.position.y)

    def obstacle_callback(self, msg):
        self.plane_present    = (msg.data == 1)
        self.shipwreck_present= (msg.data == 2)

    def update_plot(self):
        self.line.set_xdata(self.x_data)
        self.line.set_ydata(self.y_data)
        if self.plane_present:
            ab = AnnotationBbox(OffsetImage(self.plane, zoom=0.15),(55,7),frameon=False)
            self.ax.add_artist(ab)
        if self.shipwreck_present:
            ab = AnnotationBbox(OffsetImage(self.ship, zoom=0.15),(55,174),frameon=False)
            self.ax.add_artist(ab)
        plt.draw(); plt.pause(0.01)


def main():
    rclpy.init()
    node = PosePlotter(
        '/home/user/soil_sand.jpg',
        '/home/user/plane-removebg-preview.png',
        '/home/user/ship-removebg-preview.png')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
