import os
import math
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from wombat_msgs.srv import DetectObjects
from std_msgs.msg import Float64MultiArray

from ament_index_python.packages import get_package_share_directory


RECT_COLOR = (255, 0, 0)
RECT_THICKNESS = 2

CONFIDENCE_THRESHOLD = 0.5

INTEL_REALSENSE_COLOR_TOPIC = "/camera/camera/color/image_raw"
INTEL_REALSENSE_DEPTH_TOPIC = "/camera/camera/depth/image_rect_raw"
DEPTH_SCALING_FACTOR_TO_M = 1e-3

FRAME_CENTER = 320


class YoloVisionNode(Node):
    def __init__(self):
        super().__init__('yolo_vision')

        self.service = self.create_service(
            DetectObjects, "detect_objects", self.detect_callback
        )

        self.bridge = CvBridge()

        self.cv_image_colour = None
        self.cv_image_depth = None

        package_share_directory = get_package_share_directory("wombat_nav")
        models_file = os.path.join(
            package_share_directory, 'config', 'yolo_weights.pt')

        self.model = YOLO(model=models_file)

        self.get_logger().info("Vision node initialized")

    def colour_callback(self, data):
        try:
            self.cv_image_colour = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, data):
        try:
            self.cv_image_depth = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)

    def detect_callback(self, request, response):

        results = self.model.predict(
            self.cv_image_colour, stream=True, verbose=False)
        for result in results:
            boxes = result.boxes

            xywh = boxes.xywh.cpu().numpy()
            confidences = boxes.conf.cpu().numpy()

            detected_objects = []

            for i in range(xywh.shape[0]):
                if confidences[i] < CONFIDENCE_THRESHOLD:
                    continue

                x = int(xywh[i, 0])
                y = int(xywh[i, 1])
                w = int(xywh[i, 2])
                h = int(xywh[i, 3])

                distance_pitched = self.cv_image_depth[x,
                                                       y] * DEPTH_SCALING_FACTOR_TO_M
                distance = distance_pitched / math.cos(0.439976)
                angle = -((x - FRAME_CENTER) / 640 * (86 * math.pi / 180))

                angle_deg = angle * 180 / math.pi
                self.get_logger().info(f"distance: {distance}")
                self.get_logger().info(f"angle deg: {angle_deg}")

                detected_objects.append(float(distance))
                detected_objects.append(float(angle))

                frame = cv.rectangle(
                    img=frame,
                    pt1=(x-w//2, y-h//2),
                    pt2=(x+w//2, y+h//2),
                    color=RECT_COLOR,
                    thickness=RECT_THICKNESS)

        response.detections = Float64MultiArray(data=detected_objects)
        cv.imshow("Frame", frame)
        cv.waitKey(1)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloVisionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
