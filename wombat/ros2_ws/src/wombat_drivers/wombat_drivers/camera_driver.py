import rclpy

from rclpy.node import Node
from rclpy.lifecycle import State, TransitionCallbackReturn

from sensor_msgs.msg import Image

import cv2

CAMERA_ID = 0

CAMERA_UPDATE_PERIOD_MS = 10
CAMERA_RAW_IMAGE_TOPIC = "raw_camera_image"


class CameraDriver(Node):
    def __init__(self):
        super().__init__("camera_driver_node")

        self.cap = cv2.VideoCapture(CAMERA_ID)
        self.create_timer(CAMERA_UPDATE_PERIOD_MS / 1000, self.camera_timer_callback)

        self.image_publisher = self.create_publisher(Image, CAMERA_RAW_IMAGE_TOPIC, 10)

    def camera_timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        
        self.get_logger().info(f"Frame shape: {frame.shape}")
        
        # encoded as [height, width, 3]
        frame_width = frame.shape[1]
        frame_height = frame.shape[0]

        image_msg = Image()
        image_msg.height = frame_height
        image_msg.width = frame_width
        image_msg.encoding = "bgr8"
        image_msg.is_bigendian = False
        image_msg.step = frame_width
        image_msg.data = frame.data.tobytes()

        self.get_logger().info(f"frame data bytes len: {len(image_msg.data)}")

        self.image_publisher.publish(image_msg)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.cap.release()


def main(args=None):
    rclpy.init(args=args)

    camera_driver_node = CameraDriver()
    rclpy.spin(camera_driver_node)

    camera_driver_node.destroy_node()
    rclpy.shutdown()
