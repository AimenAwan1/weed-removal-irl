import rclpy
from rclpy.node import Node
from rclpy.lifecycle import State, TransitionCallbackReturn

from sensor_msgs.msg import Image

import cv2
import numpy as np

CAMERA_RAW_IMAGE_TOPIC = "raw_camera_image"


class VisionVisualizer(Node):
    def __init__(self):
        super().__init__("vision_visualizer_node")
        self.create_subscription(
            Image, CAMERA_RAW_IMAGE_TOPIC, self.camera_raw_image_callback, 4
        )

    def camera_raw_image_callback(self, msg: Image):
        if msg.encoding != "bgr8":
            self.get_logger().error(
                f'Received image on topic "{CAMERA_RAW_IMAGE_TOPIC}" is not encoded with bgr8'
            )
            return

        frame = (
            np.asarray(msg.data.tolist())
            .reshape((msg.height, msg.width, 3))
            .astype(np.uint8)
        )

        self.get_logger().info(f"Received frame shape: {frame.shape}")

        cv2.imshow("raw", frame)
        cv2.waitKey(1)  # required to show image


def main(args=None):
    rclpy.init(args=args)

    vision_visualizer_node = VisionVisualizer()
    rclpy.spin(vision_visualizer_node)

    vision_visualizer_node.destroy_node()
    rclpy.shutdown()
