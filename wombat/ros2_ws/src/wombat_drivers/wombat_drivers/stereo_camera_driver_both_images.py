import rclpy
from rclpy.node import Node, QoSProfile
import rclpy.time

from sensor_msgs.msg import Image, CameraInfo

import argparse
import cv2 as cv
import numpy as np

# camera configuration
CAMERA_FILE = "/dev/video2"
CAMERA_FPS = 30
CAMERA_RESOLUTION_X = 1600
CAMERA_RESOLUTION_Y = 600
CAMERA_FRAME_WIDTH = CAMERA_RESOLUTION_X // 2
CAMERA_FRAME_HEIGHT = CAMERA_RESOLUTION_Y

# image topic configuration
LEFT_CAMERA_INFO_TOPIC = "left/camera_info"
LEFT_CAMERA_IMAGE_RECT_TOPIC = "left/image_rect_color"
RIGHT_CAMERA_INFO_TOPIC = "right/camera_info"
RIGHT_CAMERA_IMAGE_RECT_TOPIC = "right/image_rect_color"

FRAME_ID = "camera_link"

# camera intrinsics
# these should ideally be parametrized but do this after if time permitting

# left camera
LEFT_CMTX = np.asarray(
    [
        [445.57261062314484, 0.0, 438.61173157687153],
        [0.0, 445.0452181722068, 261.0113152340737],
        [0.0, 0.0, 1.0],
    ]
)
LEFT_DIST = np.asarray(
    [
        0.007897877581497419,
        0.004819006269263674,
        -0.003462243675465205,
        -0.006716838607812541,
        -0.010283980624587765,
    ]
)
LEFT_ROT = np.asarray([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
LEFT_TRANSLATE = np.asarray([[0.0], [0.0], [0.0]])

# right camera
RIGHT_CMTX = np.asarray(
    [
        [448.709511107283, 0.0, 464.4920363621636],
        [0.0, 449.0174706756223, 267.76524208323667],
        [0.0, 0.0, 1.0],
    ]
)
RIGHT_DIST = np.asarray(
    [
        0.014225183134587135,
        -0.008648840923063178,
        -0.005587122643171826,
        -0.007907796339861127,
        0.0001369880928167401,
    ]
)
RIGHT_ROT = np.asarray(
    [
        [0.9998644746153682, -0.000311548008895979, 0.016460113607498973],
        [0.0004183981007683558, 0.9999788625128324, -0.006488410481092999],
        [-0.01645774423069253, 0.006494418017038709, 0.9998434703439619],
    ]
)
RIGHT_TRANSLATE = np.asarray(
    [[-5.976137772117957], [0.03663837200847426], [0.5600893156850932]]
) / 100  # conversion from cm to m

class StereoCameraDriver(Node):
    def __init__(self, display_frames=False):
        super().__init__("stereo_camera_driver_both_images_node")

        # setup capture from the depth camera
        self.cap = cv.VideoCapture(CAMERA_FILE)
        self.cap.set(cv.CAP_PROP_FPS, CAMERA_FPS)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, CAMERA_RESOLUTION_X)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, CAMERA_RESOLUTION_Y)
        self.create_timer(1.0 / CAMERA_FPS, self.camera_timer_callback)

        # compute new camera matrices
        self.frame_size = (CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT)
        self.left_new_mtx, _ = cv.getOptimalNewCameraMatrix(
            LEFT_CMTX, LEFT_DIST, self.frame_size, 1, self.frame_size
        )
        self.right_new_mtx, _ = cv.getOptimalNewCameraMatrix(
            RIGHT_CMTX, RIGHT_DIST, self.frame_size, 1, self.frame_size
        )

        # compute the full homogenous transformation matrix (with only top 3 rows)
        self.left_transform = np.concatenate((LEFT_ROT, LEFT_TRANSLATE), axis=1)
        self.right_transform = np.concatenate((RIGHT_ROT, RIGHT_TRANSLATE), axis=1)

        # publishers
        self.left_camera_info_publisher = self.create_publisher(
            CameraInfo, LEFT_CAMERA_INFO_TOPIC, 10
        )
        self.right_camera_info_publisher = self.create_publisher(
            CameraInfo, RIGHT_CAMERA_INFO_TOPIC, 10
        )
        self.left_camera_image_publisher = self.create_publisher(
            Image, LEFT_CAMERA_IMAGE_RECT_TOPIC, 4
        )
        self.right_camera_image_publisher = self.create_publisher(
            Image, RIGHT_CAMERA_IMAGE_RECT_TOPIC, 4
        )

        self.display_frames = display_frames
        self.get_logger().info(f"Displaying frames? {self.display_frames}")

    def camera_timer_callback(self):
        # read the camera frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error(f"Failed to read depth camera at '{CAMERA_FILE}'")
            return

        # separate into left and right images
        left = frame[:, :CAMERA_FRAME_WIDTH]
        right = frame[:, CAMERA_FRAME_WIDTH:]

        # correct distortions within the camera frame
        mapx, mapy = cv.initUndistortRectifyMap(
            LEFT_CMTX, LEFT_DIST, None, self.left_new_mtx, self.frame_size, 5
        )
        left_undistorted = cv.remap(left, mapx, mapy, cv.INTER_LINEAR)

        mapx, mapy = cv.initUndistortRectifyMap(
            RIGHT_CMTX, RIGHT_DIST, None, self.right_new_mtx, self.frame_size, 5
        )
        right_undistorted = cv.remap(right, mapx, mapy, cv.INTER_LINEAR)

        if self.display_frames:
            cv.imshow("Left Image (Undistorted)", left_undistorted)
            cv.imshow("Right Image (Undistorted)", right_undistorted)

        # publish camera info
        stamp = self.get_clock().now().to_msg()

        left_camera_info = CameraInfo()
        left_camera_info.header.stamp = stamp
        left_camera_info.header.frame_id = FRAME_ID
        left_camera_info.height = CAMERA_FRAME_HEIGHT
        left_camera_info.width = CAMERA_FRAME_WIDTH
        left_camera_info.k = LEFT_CMTX.flatten()
        left_camera_info.p = self.left_transform.flatten()
        self.left_camera_info_publisher.publish(left_camera_info)

        right_camera_info = CameraInfo()
        right_camera_info.header.stamp = stamp
        right_camera_info.header.frame_id = FRAME_ID
        right_camera_info.height = CAMERA_FRAME_HEIGHT
        right_camera_info.width = CAMERA_FRAME_WIDTH
        right_camera_info.k = RIGHT_CMTX.flatten()
        right_camera_info.p = self.right_transform.flatten()
        self.right_camera_info_publisher.publish(right_camera_info)

        # publish corrected left and right images
        left_image = Image()
        left_image.header.stamp = stamp
        left_image.header.frame_id = FRAME_ID
        left_image.height = CAMERA_FRAME_HEIGHT
        left_image.width = CAMERA_FRAME_WIDTH
        left_image.encoding = "bgr8"
        left_image.is_bigendian = False
        left_image.step = CAMERA_FRAME_WIDTH * 3
        left_image.data = left_undistorted.data.tobytes()
        self.left_camera_image_publisher.publish(left_image)

        right_image = Image()
        right_image.header.stamp = stamp
        right_image.header.frame_id = FRAME_ID
        right_image.height = CAMERA_FRAME_HEIGHT
        right_image.width = CAMERA_FRAME_WIDTH
        right_image.encoding = "bgr8"
        right_image.is_bigendian = False
        right_image.step = CAMERA_FRAME_WIDTH * 3
        right_image.data = right_undistorted.data.tobytes()
        self.right_camera_image_publisher.publish(right_image)

        if self.display_frames:
            if cv.waitKey(1) & 0xFF == 27:  # escape
                self.cap.release()
                cv.destroyAllWindows()

                self.destroy_node()
                rclpy.shutdown()


def main(args=None, display_frames=True):
    rclpy.init(args=args)

    stereo_camera_driver_node = StereoCameraDriver(display_frames)
    rclpy.spin(stereo_camera_driver_node)

    stereo_camera_driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="stereo_camera_driver")
    parser.add_argument(
        "display_frames",
        type=bool,
        default=True,
        help="render the disparity maps in gui",
    )
    args = parser.parse_args()
    main(args=None, display_frames=args.display_frames)
