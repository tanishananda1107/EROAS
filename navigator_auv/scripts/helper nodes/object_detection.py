#!/usr/bin/env python3
# ROS 2 Jazzy + Gazebo Harmonic
# Uses ultralytics YOLOv8 (recommended for ROS 2 Jazzy / Python 3.12)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # YOLOv8 nano — swap for yolov8s.pt, yolov8m.pt etc. as needed
        self.model = YOLO('yolov8n.pt')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/rexrov2/rexrov2/camera/image_raw',
            self.callback,
            1
        )
        self.publisher = self.create_publisher(Image, '/new/detected_objects', 10)
        self.get_logger().info("YOLODetector node started.")

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        results = self.model(cv_image, verbose=False)

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                label = f'{self.model.names[cls]} {conf:.2f}'

                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(
                    cv_image, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2
                )

        image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        image_msg.header = data.header
        self.publisher.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
