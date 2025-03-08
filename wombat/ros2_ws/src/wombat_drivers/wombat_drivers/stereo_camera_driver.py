import rclpy

from rclpy.node import Node
from rclpy.lifecycle import State

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

import cv2 as cv
import numpy as np

# camera configuration
CAMERA_FILE = "/dev/video2"
CAMERA_FPS = 30
CAMERA_RESOLUTION_X = 1600
CAMERA_RESOLUTION_Y = 600
CAMERA_FRAME_WIDTH = CAMERA_RESOLUTION_X // 2
CAMERA_FRAME_HEIGHT = CAMERA_RESOLUTION_Y

# node configuration
DEPTH_CAMERA_LEFT_IMG_TOPIC = "stereo_left"
DEPTH_CAMERA_DISPARITY_MAP_TOPIC = "stereo_disparity"
DEPTH_CAMERA_CLOUD_TOPIC = "stereo_cloud"
DEPTH_CAMERA_CLOUD_FRAME_ID = "/map"

# camera intrinsics
# these should ideally be parametrized but do this after if time permitting

# left camera
# intrinsics
LEFT_CMTX = np.asarray(
    [
        [445.57261062314484, 0.0, 438.61173157687153],
        [0.0, 445.0452181722068, 261.0113152340737],
        [0.0, 0.0, 1.0],
    ]
)
# spherical calibration
LEFT_DIST = np.asarray(
    [
        0.007897877581497419,
        0.004819006269263674,
        -0.003462243675465205,
        -0.006716838607812541,
        -0.010283980624587765,
    ]
)

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

BASELINE = 5.976137772117957

# disparity calculations
DISPARITY_WSIZE = 21
DISPARITY_NUM_MAX = 64

# disparity filtering
WLS_SIGMA = 1.5
WLS_LAMBDA = 80000.0
NUM_PREV_CAPTURES = 2


class StereoCameraDriver(Node):
    def __init__(self):
        super().__init__("stereo_camera_driver_node")

        self.cap = cv.VideoCapture(CAMERA_FILE)
        self.cap.set(cv.CAP_PROP_FPS, CAMERA_FPS)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, CAMERA_RESOLUTION_X)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, CAMERA_RESOLUTION_Y)

        self.create_timer(1.0 / CAMERA_FPS, self.camera_timer_callback)

        self.cloud_publisher = self.create_publisher(
            PointCloud, DEPTH_CAMERA_CLOUD_TOPIC, 4
        )

        # compute new camera matrices
        self.frame_size = (CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT)
        self.left_new_mtx, _ = cv.getOptimalNewCameraMatrix(LEFT_CMTX, LEFT_DIST, self.frame_size, 1, self.frame_size)
        self.right_new_mtx, _ = cv.getOptimalNewCameraMatrix(RIGHT_CMTX, RIGHT_DIST, self.frame_size, 1, self.frame_size)

        self.prev_disparities = np.zeros((NUM_PREV_CAPTURES,CAMERA_FRAME_HEIGHT,CAMERA_FRAME_WIDTH))


    def camera_timer_callback(self):
        # read the camera frame
        ret, frame = self.cap.read()
        if not ret:
            return
        
        # separate into left and right images
        left = frame[:,:CAMERA_FRAME_WIDTH]
        right = frame[:,CAMERA_FRAME_WIDTH:]

        # correct distortions within the camera frame
        mapx, mapy = cv.initUndistortRectifyMap(LEFT_CMTX, LEFT_DIST, None, self.left_new_mtx, self.frame_size, 5)
        left_undistorted = cv.remap(left, mapx, mapy, cv.INTER_LINEAR)

        mapx, mapy = cv.initUndistortRectifyMap(RIGHT_CMTX, RIGHT_DIST, None, self.right_new_mtx, self.frame_size, 5)
        right_undistorted = cv.remap(right, mapx, mapy, cv.INTER_LINEAR)

        # convert to grayscale
        left_grayscale = cv.cvtColor(left_undistorted, cv.COLOR_BGR2GRAY)
        right_grayscale = cv.cvtColor(right_undistorted, cv.COLOR_BGR2GRAY)

        # compute disparity map
        left_matcher = cv.StereoBM.create(DISPARITY_NUM_MAX, DISPARITY_WSIZE)
        right_matcher = cv.ximgproc.createRightMatcher(left_matcher)

        left_disparity = left_matcher.compute(left_grayscale, right_grayscale)
        right_disparity = right_matcher.compute(right_grayscale, left_grayscale)

        # low pass filter the disparities
        kernel = np.ones((3,3))/9
        left_disparity_low_pass = cv.filter2D(left_disparity,-1,kernel)
        right_disparity_low_pass = cv.filter2D(right_disparity,-1,kernel)

        # filter/upscale disparity map using the camera image
        wls_filter = cv.ximgproc.createDisparityWLSFilter(left_matcher)
        wls_filter.setLambda(WLS_LAMBDA)
        wls_filter.setSigmaColor(WLS_SIGMA)
        filtered_left_disparity = wls_filter.filter(left_disparity_low_pass, left_grayscale, disparity_map_right=right_disparity_low_pass)

        # include the frame in the moving average
        for i in range(NUM_PREV_CAPTURES-1):
            self.prev_disparities[i,:,:] = self.prev_disparities[i+1,:,:]
        self.prev_disparities[NUM_PREV_CAPTURES-1,:,:] = filtered_left_disparity

        avg_disparity = np.sum(self.prev_disparities,axis=0)/self.prev_disparities.shape[0]
        
        # error occurs in depth computation if the disparity is 0
        clipped_avg_disparity = np.clip(avg_disparity, a_min=1, a_max=None)
        avg_depth = LEFT_CMTX[0,0]*BASELINE / clipped_avg_disparity

        # # NOTE: the following uses homogenous coordinates in SE(3)

        # # parametrize the camera matrix into a homogenous matrix
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
        #     point = Point32()
        #     point.x = world_p[0,i]
        #     point.y = world_p[1,i]
        #     point.z = world_p[2,i]
        #     cloud_msg.points.append(point)
        # cloud_msg.header.frame_id = DEPTH_CAMERA_CLOUD_FRAME_ID

        # self.cloud_publisher.publish(cloud_msg)
        
        self.get_logger().info("Published new cloud from depth camera!")

        # display (temporary)
        max_visual_depth = 200
        visual_map = np.clip(avg_depth, a_min=0, a_max=max_visual_depth)/max_visual_depth
        visual_map = 1 - visual_map # recolor so light is closer
        cv.imshow("visual", visual_map)
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