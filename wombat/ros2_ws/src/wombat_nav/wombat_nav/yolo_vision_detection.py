import os
import math
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from ultralytics import YOLO

import rclpy
import rclpy.callback_groups
import rclpy.executors
from rclpy.node import Node
from wombat_msgs.srv import DetectObjects
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from sensor_msgs.msg import Image

from ament_index_python.packages import get_package_share_directory

import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


RECT_COLOR = (255, 0, 0)
RECT_THICKNESS = 2

CONFIDENCE_THRESHOLD = 0.5

INTEL_REALSENSE_COLOR_TOPIC = "/camera/color/image_raw"
INTEL_REALSENSE_DEPTH_TOPIC = "/camera/depth/image_rect_raw"
DEPTH_SCALING_FACTOR_TO_M = 1e-3

VISION_SAMPLING_RATE_HZ = 5
VISION_SAMPLING_TOPIC = "yolo_labeled_image"
FRAME_ID = "camera_link"

PITCH_ANGLE_DEG = 35
CAMERA_FOV = 86

# camera properties (ideally should read from camera_info topic)
CAMERA_KU = 606.4159545898438
CAMERA_KV = 605.8991088867188
CAMERA_U0 = 318.94219970703125
CAMERA_V0 = 252.5470733642578

CAMERA_FRAME = "camera_color_optical_frame"
WORLD_FRAME = "base_link_odom"


class YoloVisionNode(Node):
    def __init__(self):
        super().__init__('yolo_vision')
        self.global_count_ = 0
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.service = self.create_service(
            DetectObjects, "detect_objects", self.detect_callback,
            callback_group=self.callback_group
        )

        self.bridge = CvBridge()

        self.cv_image_colour = None
        self.cv_image_depth = None

        package_share_directory = get_package_share_directory("wombat_nav")
        models_file = os.path.join(
            package_share_directory, 'config', 'yolo_weights_model_2.pt')

        self.model = YOLO(model=models_file)

        self.get_logger().info("Vision node initialized")

        self.create_subscription(
            Image, INTEL_REALSENSE_COLOR_TOPIC, self.colour_callback, 4, callback_group=self.callback_group)
        self.create_subscription(
            Image, INTEL_REALSENSE_DEPTH_TOPIC, self.depth_callback, 4, callback_group=self.callback_group)

        self.create_timer(1.0 / VISION_SAMPLING_RATE_HZ,
                          self.vision_sampling_timer_callback)
        self.yolo_labeled_publisher = self.create_publisher(
            Image, VISION_SAMPLING_TOPIC, 4)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

    def vision_sampling_timer_callback(self):
        if self.cv_image_colour is None or self.cv_image_depth is None:
            return

        self.get_logger().info("Running detection on callback!")

        frame, _ = self.run_detection(
            self.cv_image_colour, self.cv_image_depth)

        image = Image()
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = FRAME_ID
        image.height = frame.shape[0]
        image.width = frame.shape[1]
        image.encoding = "bgr8"
        image.is_bigendian = False
        image.step = image.width * 3
        image.data = frame.data.tobytes()
        self.yolo_labeled_publisher.publish(image)

    def detect_callback(self, request, response):
        self.get_logger().info("Entered detection callback!")

        _, detected_objects = self.run_detection(
            self.cv_image_colour, self.cv_image_depth)

        self.get_logger().info("Finished object detection!")

        if len(detected_objects) == 0:
            self.get_logger().info("No objects were found!")

        camera_frame_positions = []
        for distance, u, v in detected_objects:
            self.get_logger().info(
                f"Found object at distance={distance}, u={u}, v={v}")

            # computes distance in typical camera frame
            z = distance
            x = distance / CAMERA_KU * (u - CAMERA_U0)
            y = distance / CAMERA_KV * (v - CAMERA_V0)

            point = PointStamped()
            point.point.x = x
            point.point.y = y
            point.point.z = z

            camera_frame_positions.append(
                point
            )

        # get transform from the camera frame to the world frame
        try:
            t = self.tf_buffer.lookup_transform(
                WORLD_FRAME, CAMERA_FRAME, rclpy.time.Time())
        except:
            self.get_logger().error(
                f"failed to look up transform from camera frame {CAMERA_FRAME} to world frame {WORLD_FRAME}")
            return None  # fails

        # transforms each point
        transformed_positions = []
        for camera_frame_position in camera_frame_positions:
            world_frame_position = tf2_geometry_msgs.do_transform_point(
                camera_frame_position, t)

            # only need the x,y positions
            world_x = world_frame_position.point.x
            world_y = world_frame_position.point.y

            self.get_logger().info(f"World frame: x={world_x}, y={world_y}, z={world_frame_position.point.z}")
            transformed_positions.extend([world_x, world_y])

        print(transformed_positions)

        # self.get_logger().info(f"{transformed_positions}")
        # self.get_logger().info(f"{type(transformed_positions)}")
        # self.get_logger().info(f"{type(transformed_positions[0])}")

        # no numpy shenanigans
        response.detections.data = [float(val) for val in transformed_positions]
        return response

    def run_detection(self, color_frame, depth_frame):
        color_frame = color_frame.copy()  # creates a copy of the frame
        depth_frame = depth_frame.copy()

        results = self.model.predict(
            color_frame, stream=True, verbose=True)

        print(f"color frame shape: {color_frame.shape}")
        print(f"depth frame shape: {depth_frame.shape}")

        frame_width = color_frame.shape[0]
        frame_center = frame_width//2

        detected_objects = []

        for result in results:
            boxes = result.boxes

            xywh = boxes.xywh.cpu().numpy()
            confidences = boxes.conf.cpu().numpy()

            for i in range(xywh.shape[0]):
                if confidences[i] < CONFIDENCE_THRESHOLD:
                    continue

                print(f"confidence: {confidences[i]}")

                x = int(xywh[i, 0])
                y = int(xywh[i, 1])
                w = int(xywh[i, 2])
                h = int(xywh[i, 3])

                distance_pitched = depth_frame[y,
                                               x] * DEPTH_SCALING_FACTOR_TO_M
                distance = distance_pitched / \
                    math.cos(math.radians(PITCH_ANGLE_DEG))
                angle = (x - frame_center) / frame_width * \
                    (math.radians(CAMERA_FOV))

                angle_deg = angle * 180 / math.pi
                self.get_logger().info(f"distance pitched: {distance_pitched}")
                self.get_logger().info(f"distance: {distance}")
                self.get_logger().info(f"angle deg: {angle_deg}")

                detection = (float(distance), xywh[i, 0], xywh[i, 1])
                detected_objects.append(detection)

                # detected_objects.append(float(distance))
                # detected_objects.append(float(angle))

                color_frame = cv.rectangle(
                    img=color_frame,
                    pt1=(x-w//2, y-h//2),
                    pt2=(x+w//2, y+h//2),
                    color=RECT_COLOR,
                    thickness=RECT_THICKNESS)

        return color_frame, detected_objects


def main(args=None):
    rclpy.init(args=args)
    node = YoloVisionNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
