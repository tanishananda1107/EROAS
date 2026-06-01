#!/usr/bin/env python3
# ROS 2 Jazzy + Gazebo Harmonic

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


class YOLOHeading(Node):
    def __init__(self):
        super().__init__('yolo_heading')

        # YOLOv8 nano — swap for larger model as needed
        self.model = YOLO('yolov8n.pt')

        self.bridge = CvBridge()
        self.img_width = None
        self.img_height = None

        self.subscription = self.create_subscription(
            Image,
            '/rexrov2/rexrov2/camera/image_raw',
            self.callback,
            1
        )
        self.image_publisher = self.create_publisher(Image, '/new/detected_objects', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/rexrov2/cmd_vel', 10)

        self.get_logger().info("YOLOHeading node started.")

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.img_width = data.width
        self.img_height = data.height

        results = self.model(cv_image, verbose=False)

        closest_object = None
        min_distance = float('inf')

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                label = f'{self.model.names[cls]} {conf:.2f}'

                x_center = (x1 + x2) / 2.0
                y_center = (y1 + y2) / 2.0

                distance = abs(self.img_width / 2 - x_center)
                if distance < min_distance:
                    min_distance = distance
                    closest_object = (x_center, y_center, label)

                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(
                    cv_image, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2
                )

        image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        image_msg.header = data.header
        self.image_publisher.publish(image_msg)

        if closest_object:
            x_center, y_center, _ = closest_object
            self.adjust_heading(x_center, y_center)

    def adjust_heading(self, object_x, object_y):
        error_heading = self.img_width / 2 - object_x
        error_height = self.img_height / 2 - object_y

        k_p = 0.005
        k_h = 0.05

        twist = Twist()
        twist.angular.z = k_p * error_heading
        twist.linear.z = k_h * error_height
        self.cmd_vel_publisher.publish(twist)

        self.get_logger().info(
            f"Heading error: {error_heading:.4f}, Z error: {error_height:.4f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = YOLOHeading()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
