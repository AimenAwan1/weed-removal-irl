import rclpy

from rclpy.node import Node

import rclpy.time
from sensor_msgs.msg import Image, CameraInfo

import cv2 as cv
import numpy as np

import time

# camera configuration
CAMERA_FILE = "/dev/video2"
CAMERA_FPS = 30
CAMERA_RESOLUTION_X = 1600
CAMERA_RESOLUTION_Y = 600
CAMERA_FRAME_WIDTH = CAMERA_RESOLUTION_X // 2
CAMERA_FRAME_HEIGHT = CAMERA_RESOLUTION_Y

# image topic configuration
LEFT_CAMERA_INFO_TOPIC = "left/camera_info"
LEFT_CAMERA_IMAGE_RECT_TOPIC = "left/image_rect"
RIGHT_CAMERA_INFO_TOPIC = "right/camera_info"
RIGHT_CAMERA_IMAGE_RECT_TOPIC = "right/image_rect"

FRAME_ID = "stereo-camera"


# DEPTH_CAMERA_LEFT_IMG_TOPIC = "stereo_left"
# DEPTH_CAMERA_DISPARITY_MAP_TOPIC = "stereo_disparity"
# DEPTH_CAMERA_CLOUD_TOPIC = "stereo_cloud"
# DEPTH_CAMERA_CLOUD_FRAME_ID = "/map"

# camera intrinsics
# these should ideally be parametrized but do this after if time permitting

# left camera
# intrinsics
LEFT_CMTX = np.asarray([
        [445.57261062314484, 0.0, 438.61173157687153],
        [0.0, 445.0452181722068, 261.0113152340737],
        [0.0, 0.0, 1.0],])
# spherical calibration
LEFT_DIST = np.asarray([
        0.007897877581497419,
        0.004819006269263674,
        -0.003462243675465205,
        -0.006716838607812541,
        -0.010283980624587765])
LEFT_ROT = np.asarray([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
LEFT_TRANSLATE = np.asarray([[0.0], [0.0], [0.0]]) 

# right camera
RIGHT_CMTX = np.asarray([
        [448.709511107283, 0.0, 464.4920363621636],
        [0.0, 449.0174706756223, 267.76524208323667],
        [0.0, 0.0, 1.0],])
RIGHT_DIST = np.asarray([
        0.014225183134587135,
        -0.008648840923063178,
        -0.005587122643171826,
        -0.007907796339861127,
        0.0001369880928167401,])
RIGHT_ROT = np.asarray([
    [0.9998644746153682, -0.000311548008895979, 0.016460113607498973 ],
    [0.0004183981007683558, 0.9999788625128324, -0.006488410481092999 ],
    [-0.01645774423069253, 0.006494418017038709, 0.9998434703439619 ]])
RIGHT_TRANSLATE = np.asarray([
    [-5.976137772117957 ],
    [0.03663837200847426 ],
    [0.5600893156850932 ]
])

# disparity calculations
DISPARITY_WSIZE = 21
DISPARITY_NUM_MAX = 96
BASELINE = 5.976137772117957

# disparity filtering
WLS_SIGMA = 1.1
WLS_LAMBDA = 8000.0
NUM_PREV_CAPTURES = 2

# point cloud transmission characteristics (m)
# POINT_CLOUD_MIN_DIST = 0.1
# POINT_CLOUD_MAX_DIST = 2


class StereoCameraDriver(Node):
    def __init__(self):
        super().__init__("stereo_camera_driver_node")

        self.cap = cv.VideoCapture(CAMERA_FILE)
        self.cap.set(cv.CAP_PROP_FPS, CAMERA_FPS)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, CAMERA_RESOLUTION_X)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, CAMERA_RESOLUTION_Y)

        self.create_timer(1.0 / CAMERA_FPS, self.camera_timer_callback)

        # self.cloud_publisher = self.create_publisher(
        #     PointCloud, DEPTH_CAMERA_CLOUD_TOPIC, 4
        # )

        # compute new camera matrices
        self.frame_size = (CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT)
        self.left_new_mtx, _ = cv.getOptimalNewCameraMatrix(LEFT_CMTX, LEFT_DIST, self.frame_size, 1, self.frame_size)
        self.right_new_mtx, _ = cv.getOptimalNewCameraMatrix(RIGHT_CMTX, RIGHT_DIST, self.frame_size, 1, self.frame_size)

        # compute the full homogenous transformation matrix (with only top 3 rows)
        self.left_transform = np.concatenate((LEFT_ROT, LEFT_TRANSLATE), axis=1)
        self.right_transform = np.concatenate((RIGHT_ROT, RIGHT_TRANSLATE), axis=1)

        self.prev_left_disparities = np.zeros((NUM_PREV_CAPTURES,CAMERA_FRAME_HEIGHT,CAMERA_FRAME_WIDTH))
        self.prev_right_disparities = np.zeros((NUM_PREV_CAPTURES,CAMERA_FRAME_HEIGHT,CAMERA_FRAME_WIDTH))

        # publishers
        self.left_camera_info_publisher = self.create_publisher(CameraInfo, LEFT_CAMERA_INFO_TOPIC, 10)
        self.right_camera_info_publisher = self.create_publisher(CameraInfo, RIGHT_CAMERA_INFO_TOPIC, 5)

        self.left_camera_image_publisher = self.create_publisher(Image, LEFT_CAMERA_IMAGE_RECT_TOPIC, 10)
        self.right_camera_image_publisher = self.create_publisher(Image, RIGHT_CAMERA_IMAGE_RECT_TOPIC, 5)



    def camera_timer_callback(self):
        # read the camera frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error(f"Failed to read depth camera at '{CAMERA_FILE}'")
            return
        
        # separate into left and right images
        left = frame[:,:CAMERA_FRAME_WIDTH]
        right = frame[:,CAMERA_FRAME_WIDTH:]

        # correct distortions within the camera frame
        mapx, mapy = cv.initUndistortRectifyMap(LEFT_CMTX, LEFT_DIST, None, self.left_new_mtx, self.frame_size, 5)
        left_undistorted = cv.remap(left, mapx, mapy, cv.INTER_LINEAR)

        mapx, mapy = cv.initUndistortRectifyMap(RIGHT_CMTX, RIGHT_DIST, None, self.right_new_mtx, self.frame_size, 5)
        right_undistorted = cv.remap(right, mapx, mapy, cv.INTER_LINEAR)

        # # publish camera info
        # stamp = self.get_clock().now().to_msg()

        # left_camera_info = CameraInfo()
        # left_camera_info.header.stamp = stamp
        # left_camera_info.header.frame_id = FRAME_ID
        # left_camera_info.height = CAMERA_FRAME_HEIGHT
        # left_camera_info.width = CAMERA_FRAME_WIDTH
        # left_camera_info.k = LEFT_CMTX.flatten()
        # left_camera_info.p = self.left_transform.flatten()

        # right_camera_info = CameraInfo()
        # right_camera_info.header.stamp = stamp
        # right_camera_info.header.frame_id = FRAME_ID
        # right_camera_info.height = CAMERA_FRAME_HEIGHT
        # right_camera_info.width = CAMERA_FRAME_WIDTH
        # right_camera_info.k = RIGHT_CMTX.flatten()
        # right_camera_info.p = self.right_transform.flatten()

        # self.left_camera_info_publisher.publish(left_camera_info)
        # self.right_camera_info_publisher.publish(right_camera_info)

        # # publish corrected left and right images
        # left_image = Image()
        # left_image.header.stamp = stamp
        # left_image.header.frame_id = FRAME_ID
        # left_image.height = CAMERA_FRAME_HEIGHT
        # left_image.width = CAMERA_FRAME_WIDTH
        # left_image.encoding = "bgr8"
        # left_image.is_bigendian = False
        # left_image.step = CAMERA_FRAME_WIDTH * 3
        # left_image.data = left_undistorted.data.tobytes()

        # right_image = Image()
        # right_image.header.stamp = stamp
        # right_image.header.frame_id = FRAME_ID
        # right_image.height = CAMERA_FRAME_HEIGHT
        # right_image.width = CAMERA_FRAME_WIDTH
        # right_image.encoding = "bgr8"
        # right_image.is_bigendian = False
        # right_image.step = CAMERA_FRAME_WIDTH * 3
        # right_image.data = right_undistorted.data.tobytes()

        # self.left_camera_image_publisher.publish(left_image)
        # self.right_camera_image_publisher.publish(right_image)

        self.get_logger().info("Processed camera frame...")

        # convert to grayscale
        left_grayscale = cv.cvtColor(left_undistorted, cv.COLOR_BGR2GRAY)
        right_grayscale = cv.cvtColor(right_undistorted, cv.COLOR_BGR2GRAY)

        # # compute disparity map

        channels = 1
        min_disparity = 11
        num_disparities = 32
        block_size = 3
        p1 = 8*channels*block_size*block_size
        p2 = 32*channels*block_size*block_size
        disp_12_max_diff = 5
        pre_filter_cap = 5
        uniqueness_ratio = 10
        speckle_window_size = 50
        speckle_range = 5
        mode = 0  # normal sgbm mode for now


        left_matcher = cv.StereoSGBM.create(min_disparity, num_disparities, block_size, p1, p2, disp_12_max_diff, pre_filter_cap, uniqueness_ratio, speckle_window_size, speckle_range, mode)
        right_matcher = cv.ximgproc.createRightMatcher(left_matcher)

        left_disparity = left_matcher.compute(left_grayscale, right_grayscale)
        right_disparity = right_matcher.compute(right_grayscale, left_grayscale)

        cv.imshow("left", left)
        cv.imshow("left grayscale", left_grayscale)

        disp_left_disparity = left_disparity / np.max(left_disparity)
        cv.imshow("left disparity", disp_left_disparity)

        # # low pass filter the disparities
        # kernel = np.ones((3,3))/9
        # left_disparity_low_pass = cv.filter2D(left_disparity,-1,kernel)
        # right_disparity_low_pass = cv.filter2D(right_disparity,-1,kernel)

        # include the frame in the moving average
        for i in range(NUM_PREV_CAPTURES-1):
            self.prev_left_disparities[i,:,:] = self.prev_left_disparities[i+1,:,:]
            self.prev_right_disparities[i,:,:] = self.prev_right_disparities[i+1,:,:]
        self.prev_left_disparities[NUM_PREV_CAPTURES-1,:,:] = left_disparity
        self.prev_right_disparities[NUM_PREV_CAPTURES-1,:,:] = right_disparity

        avg_left_disparity = self.prev_left_disparities.sum(axis=0) / self.prev_left_disparities.shape[0]
        avg_right_disparity = self.prev_right_disparities.sum(axis=0) / self.prev_right_disparities.shape[0]

        preconversion = np.float32(avg_left_disparity / np.max(np.clip(avg_left_disparity, a_min=1, a_max=None)))
        disparity_grayscale_img = cv.cvtColor(preconversion, cv.COLOR_GRAY2BGR)
        disparity_grayscale_img_int = np.uint8(disparity_grayscale_img*255)

        print(disparity_grayscale_img_int)
        cv.imshow("avg left disparity", cv.applyColorMap(disparity_grayscale_img_int, cv.COLORMAP_JET))

        # filter/upscale disparity map using the camera image
        wls_filter = cv.ximgproc.createDisparityWLSFilter(left_matcher)

        lmbda = 20000 # regularization during filtering -> larger forces filtered disparity to adhere to source edges
        sigma_color = 2.0  # sensitivity to source image edges -> low value better adherence to source edges
        depth_discontinuity_radius = 2 # defines size of low confidence regions around discontinuities
        lrc_thresh = 24  # this value basically always good (threshold disparity diff in left consistency check)

        wls_filter.setLambda(lmbda)
        wls_filter.setSigmaColor(sigma_color)
        wls_filter.setDepthDiscontinuityRadius(depth_discontinuity_radius)
        wls_filter.setLRCthresh(lrc_thresh)
        filtered_left_disparity = wls_filter.filter(avg_left_disparity, left_grayscale, disparity_map_right=avg_right_disparity)

        preconversion = np.float32(filtered_left_disparity / np.max(np.clip(filtered_left_disparity, a_min=1, a_max=None)))
        disparity_grayscale_img = cv.cvtColor(preconversion, cv.COLOR_GRAY2BGR)
        disparity_grayscale_img_int = np.uint8(disparity_grayscale_img*255)

        print(disparity_grayscale_img_int)
        cv.imshow("filtered disparity", cv.applyColorMap(disparity_grayscale_img_int, cv.COLORMAP_JET))

        # cv.imshow("filtered", np.clip(filtered_left_disparity, 0, 1600))

        # depth = LEFT_CMTX[0,0]*BASELINE / np.clip(filtered_left_disparity, a_min=1,a_max=None)
        # cv.imshow("depth", 1-depth)

        

        # # error occurs in depth computation if the disparity is 0
        # clipped_avg_disparity = np.clip(avg_disparity, a_min=1, a_max=None)
        # avg_depth = LEFT_CMTX[0,0]*BASELINE / clipped_avg_disparity

        # # NOTE: the following uses homogenous coordinates in SE(3)

        # parametrize the camera matrix into a homogenous matrix
        # K = np.array([
        #     [LEFT_CMTX[0,0], 0, 0, LEFT_CMTX[0,2]],
        #     [0, LEFT_CMTX[1,1], 0, LEFT_CMTX[1,2]],
        #     [0, 0, 1, 0],
        #     [0, 0, 0, 1]])
        # R = np.eye(4) # temporary camera transformation matrix in world space

        # # need to apply to points to enure measurement distance is along z
        # Ry_correction = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])

        # u_vals = np.tile((np.arange(CAMERA_FRAME_WIDTH)+1),(CAMERA_FRAME_HEIGHT,1))
        # v_vals = np.tile((np.arange(CAMERA_FRAME_HEIGHT)+1).reshape((CAMERA_FRAME_HEIGHT,1)),(1,CAMERA_FRAME_WIDTH))

        # camera_p = np.row_stack((
        #     u_vals.flatten(),
        #     v_vals.flatten(),
        #     np.ones((CAMERA_FRAME_HEIGHT, CAMERA_FRAME_WIDTH)).flatten(),
        #     np.ones((CAMERA_FRAME_HEIGHT, CAMERA_FRAME_WIDTH)).flatten()))
        # # note the conversion between meters to centimeters
        # world_p = Ry_correction @ np.linalg.inv(K @ R) @ (avg_depth.flatten() * camera_p) / 1_00

        # cloud_msg = PointCloud()
        # cloud_msg.points = []
        # for i in range(world_p.shape[1]):
        #     # horribly inefficient, update later to increase processing speed
        #     point = Point32()
        #     point.x = world_p[0,i]
        #     if point.x < POINT_CLOUD_MIN_DIST or point.x > POINT_CLOUD_MAX_DIST:
        #         continue
        #     point.y = world_p[1,i]
        #     point.z = world_p[2,i]
        #     cloud_msg.points.append(point)
        # cloud_msg.header.frame_id = DEPTH_CAMERA_CLOUD_FRAME_ID

        # self.cloud_publisher.publish(cloud_msg)
        
        # self.get_logger().info("Published new cloud from depth camera!")

        # display (temporary)
        # max_visual_depth = 1000 # 10 m
        # visual_map = np.clip(avg_depth, a_min=0, a_max=max_visual_depth)/max_visual_depth
        # visual_map = 1 - visual_map # recolor so light is closer
        # cv.imshow("visual", visual_map)
        # cv.imshow("raw", left)
        # cv.imshow("original", left_disparity/np.max(left_disparity))
        # cv.imshow("filtered", left_disparity_low_pass/np.max(left_disparity_low_pass))
        # cv.imshow("left_filt", filtered_left_disparity)
        # cv.imshow("left_grayscale", left_grayscale)

        if cv.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv.destroyAllWindows()

            self.destroy_node()
            rclpy.shutdown()

        # generate world position estimates for all 
        
def main(args=None):
    rclpy.init(args=args)

    stereo_camera_driver_node = StereoCameraDriver()
    rclpy.spin(stereo_camera_driver_node)

    stereo_camera_driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()