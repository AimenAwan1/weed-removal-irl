import rclpy
from rclpy.node import Node
import rclpy.time

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField

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

FRAME_ID = "/map"

DEPTH_CAMERA_CLOUD_TOPIC = "stereo_cloud"
DEPTH_CAMERA_CLOUD_FRAME_ID = "/map"

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
)

# specific camera properties obtained from the matrices above
BASELINE = np.abs(RIGHT_TRANSLATE.flatten()[0])
FOCAL_LEN = LEFT_CMTX[0, 0]
U0 = LEFT_CMTX[0, 2]
V0 = LEFT_CMTX[1, 2]

# sgbm parameters
min_disparity = 11
num_disparities = 64  # multiple of 16
block_size = 11
p1 = 224  # usually 8*3*block_size**2
p2 = 1614  # usually 32*3*block_size**2
disp12_max_diff = 12
pre_filter_cap = 24
uniqueness_ratio = 5
speckle_window_size = 200
speckle_range = 2
mode = cv.STEREO_SGBM_MODE_SGBM  # default mode

# guided upsampling parameters
guided_radius = 40
guided_epsilon = 0.12

# restrictions on the point cloud
min_depth = 0.05  # m
max_depth = 2.0  # m

# random downsampling of the cloud
cloud_downsample_fraction = 0.02

ksize = 5
sigma_x = 0.0

# if displaying frames, allows tuning of parameters
cv.namedWindow('SGBM Parameters')
# algorithm parameters
cv.createTrackbar('Min Disparity', 'SGBM Parameters', min_disparity, 100, lambda x: None)
cv.createTrackbar('Num Disparities', 'SGBM Parameters', num_disparities, 64, lambda x: None)
cv.createTrackbar('Block Size', 'SGBM Parameters', block_size, 21, lambda x: None)
cv.createTrackbar('P1', 'SGBM Parameters', p1, 2000, lambda x: None)
cv.createTrackbar('P2', 'SGBM Parameters', p2, 6000, lambda x: None)
cv.createTrackbar('Disp12 Max Diff', 'SGBM Parameters', disp12_max_diff, 50, lambda x: None)
cv.createTrackbar('Pre Filter Cap', 'SGBM Parameters', pre_filter_cap, 63, lambda x: None)
cv.createTrackbar('Uniqueness Ratio', 'SGBM Parameters', uniqueness_ratio, 50, lambda x: None)
cv.createTrackbar('Speckle Window Size', 'SGBM Parameters', speckle_window_size, 200, lambda x: None)
cv.createTrackbar('Speckle Range', 'SGBM Parameters', speckle_range, 100, lambda x: None)
cv.createTrackbar('Mode', 'SGBM Parameters', 0, 3, lambda x: None)  # 0 for SGBM, 1 for HH, 2 for SGBM_3WAY, 3 for HH4
# guided upsampling filter
cv.createTrackbar('Guided Filter Radius', 'SGBM Parameters', guided_radius, 50, lambda x: None)  # Radius for guided filter
cv.createTrackbar('Guided Filter Epsilon', 'SGBM Parameters', int(guided_epsilon*100), 100, lambda x: None)  # Epsilon for guided filter
# gaussian blur
cv.createTrackbar('Gaussian ksize', 'SGBM Parameters', ksize, 31, lambda x: None)
cv.createTrackbar('Gaussian sigma x', 'SGBM Parameters', int(sigma_x*10), 10*10, lambda x: None)

class StereoCameraDriver(Node):
    def __init__(self, display_frames=False):
        super().__init__("stereo_camera_driver_node")

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
        self.left_camera_image_publisher = self.create_publisher(
            Image, LEFT_CAMERA_IMAGE_RECT_TOPIC, 10
        )
        self.cloud_publisher = self.create_publisher(
            PointCloud2, DEPTH_CAMERA_CLOUD_TOPIC, 4
        )

        # large computational cost so cache these arrays of u and v used in computing
        # the x and y positions of each point within the point cloud
        self.u_vals = np.tile(
            np.arange(CAMERA_FRAME_WIDTH) + 1, (CAMERA_FRAME_HEIGHT, 1)
        )
        self.v_vals = np.tile(
            (np.arange(CAMERA_FRAME_HEIGHT) + 1).reshape((CAMERA_FRAME_HEIGHT, 1)),
            (1, CAMERA_FRAME_WIDTH),
        )

        self.display_frames = display_frames

    def camera_timer_callback(self):
        # read the camera frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error(f"Failed to read depth camera at '{CAMERA_FILE}'")
            return
        
        if self.display_frames:
            # load the new parameters for sgbm and the guided updsampling
            
            min_disparity = cv.getTrackbarPos('Min Disparity', 'SGBM Parameters')
            num_disparities = cv.getTrackbarPos('Num Disparities', 'SGBM Parameters')
            block_size = cv.getTrackbarPos('Block Size', 'SGBM Parameters')
            p1 = cv.getTrackbarPos('P1', 'SGBM Parameters')
            p2 = cv.getTrackbarPos('P2', 'SGBM Parameters')
            disp12_max_diff = cv.getTrackbarPos('Disp12 Max Diff', 'SGBM Parameters')
            pre_filter_cap = cv.getTrackbarPos('Pre Filter Cap', 'SGBM Parameters')
            uniqueness_ratio = cv.getTrackbarPos('Uniqueness Ratio', 'SGBM Parameters')
            speckle_window_size = cv.getTrackbarPos('Speckle Window Size', 'SGBM Parameters')
            speckle_range = cv.getTrackbarPos('Speckle Range', 'SGBM Parameters')
            mode = cv.getTrackbarPos('Mode', 'SGBM Parameters')
            
            guided_radius = cv.getTrackbarPos('Guided Filter Radius', 'SGBM Parameters')
            guided_epsilon = cv.getTrackbarPos('Guided Filter Epsilon', 'SGBM Parameters') / 100.0  # Divide by 100 for a reasonable range

            # ksize must be odd
            ksize = cv.getTrackbarPos('Gaussian ksize', 'SGBM Parameters')
            ksize = 2*ksize + 1
            print(f'Using ksize={ksize}')

            sigma_x = float(cv.getTrackbarPos('Gaussian sigma x', 'SGBM Parameters'))

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

        left_undistorted_blurred = cv.GaussianBlur(left_undistorted, ksize=(ksize,ksize), sigmaX=sigma_x)
        right_undistorted_blurred = cv.GaussianBlur(right_undistorted, ksize=(ksize,ksize), sigmaX=sigma_x)

        if self.display_frames:
            cv.imshow("Left Image (Undistorted, Blurred)", left_undistorted_blurred)
            cv.imshow("Right Image (Undistorted, Blurred)", right_undistorted_blurred)

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

        # publish corrected left and right images
        left_image = Image()
        left_image.header.stamp = stamp
        left_image.header.frame_id = FRAME_ID
        left_image.height = CAMERA_FRAME_HEIGHT
        left_image.width = CAMERA_FRAME_WIDTH
        left_image.encoding = "bgr8"
        left_image.is_bigendian = False
        left_image.step = CAMERA_FRAME_WIDTH * 3
        left_image.data = left_undistorted_blurred.data.tobytes()
        self.left_camera_image_publisher.publish(left_image)

        # convert to grayscale
        left_grayscale = cv.cvtColor(left_undistorted_blurred, cv.COLOR_BGR2GRAY)
        right_grayscale = cv.cvtColor(right_undistorted_blurred, cv.COLOR_BGR2GRAY)

        # compute disparity map using the sgbm algorithm
        left_matcher = cv.StereoSGBM.create(
            minDisparity=min_disparity,
            numDisparities=num_disparities,
            blockSize=block_size,
            P1=p1,
            P2=p2,
            disp12MaxDiff=disp12_max_diff,
            preFilterCap=pre_filter_cap,
            uniquenessRatio=uniqueness_ratio,
            speckleWindowSize=speckle_window_size,
            speckleRange=speckle_range,
            mode=mode,
        )
        left_disparity = left_matcher.compute(left_grayscale, right_grayscale)

        if self.display_frames:
            render_max_disparity = FOCAL_LEN * BASELINE / (min_depth*100) # note converison to cm
            render_min_disparity = FOCAL_LEN * BASELINE / (max_depth*100)

            # print(f'Render max disparity: {render_max_disparity}')
            # print(f'Render min disparity: {render_min_disparity}')

            left_disparity_adjusted = np.copy(left_disparity)

            # print(left_disparity_adjusted)

            # print(left_disparity_adjusted < render_min_disparity)

            # left_disparity_adjusted[left_disparity_adjusted > render_max_disparity] = 0
            # left_disparity_adjusted[left_disparity_adjusted < render_min_disparity] = 0

            left_disparity_normalized = cv.normalize(
                left_disparity_adjusted, None, 0, 255, cv.NORM_MINMAX
            )
            left_disparity_normalized = np.uint8(left_disparity_normalized)
            cv.imshow("Left Disparity Map", left_disparity_normalized)

        # opencv scales the true disparity by 16
        left_disparity = np.float32(left_disparity) / 16.0

        # upscale the depth map using the original image for guidance
        left_disparity_guided = cv.ximgproc.guidedFilter(
            left_undistorted_blurred, left_disparity, guided_radius, guided_epsilon
        )

        if self.display_frames:
            left_disparity_guided_normalized = cv.normalize(
                left_disparity_guided, None, 0, 255, cv.NORM_MINMAX
            )
            left_disparity_guided_normalized = np.uint8(
                left_disparity_guided_normalized
            )
            cv.imshow("Left Disparity Map (Guided)", left_disparity_guided_normalized)

        # generates a point cloud from the disparity map

        # downsamples the point cloud to become more manageable
        cloud_max_disparity = FOCAL_LEN * BASELINE / min_depth
        cloud_min_disparity = FOCAL_LEN * BASELINE / max_depth

        disparity_cutoff_mask = np.logical_and(
            left_disparity_guided <= cloud_max_disparity,
            left_disparity_guided >= cloud_min_disparity,
        )
        random_cutoff_mask = (
            np.random.uniform(size=left_disparity_guided.shape)
            <= cloud_downsample_fraction
        )
        mask = np.logical_and(disparity_cutoff_mask, random_cutoff_mask)

        cloud_disparity = np.clip(
            left_disparity_guided[mask].flatten(), a_min=1e-3, a_max=None
        )
        cloud_z = np.clip(
            FOCAL_LEN * BASELINE / cloud_disparity, a_min=min_depth, a_max=max_depth
        )
        cloud_x = cloud_z / FOCAL_LEN * (self.u_vals[mask] - U0).flatten()
        cloud_y = cloud_z / FOCAL_LEN * (self.v_vals[mask] - V0).flatten()

        print(f"Number of cloud points: {len(cloud_disparity)}")

        # generate the point cloud msg using an unordered array
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = stamp
        cloud_msg.header.frame_id = "camera_link"
        cloud_msg.height = 1  # required for the unordered array
        cloud_msg.width = len(cloud_z)
        cloud_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = len(cloud_msg.fields) * 4
        cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step
        cloud_msg.is_dense = True  # all invalid points have been removed
        cloud_msg.data = np.column_stack((cloud_x, cloud_y, cloud_z)).data.tobytes()

        self.cloud_publisher.publish(cloud_msg)

        if self.display_frames:
            if cv.waitKey(1) & 0xFF == 27:  # escape
                self.cap.release()
                cv.destroyAllWindows()

                self.destroy_node()
                rclpy.shutdown()


def main(args=None, display_frames=False):
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
        default=False,
        help="render the disparity maps in gui",
    )
    args = parser.parse_args()
    main(args=None, display_frames=args.display_frames)
