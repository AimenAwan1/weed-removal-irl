#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import cv2 as cv
import math

# BRG color values
#yellow
UPPER_MATCH_COLOR = np.array([153, 255, 255])
LOWER_MATCH_COLOR = np.array([30, 128, 170])
#white
#LOWER_MATCH_COLOR = np.array([150,150,150])
#UPPER_MATCH_COLOR = np.array([250, 250, 250])

KNOWN_WIDTH_CM = 11  

OUTPUT_FILE = "detected_objects.txt"


# full resolution of the combined video feed from the depth camera
CAPTURE_RESOLUTION_X = 1600
CAPTURE_RESOLUTION_Y = 600

# single frame resolution
FRAME_WIDTH = CAPTURE_RESOLUTION_X//2
FRAME_HEIGHT = CAPTURE_RESOLUTION_Y

# left camera calibration
left_cmtx = np.asarray([
    [418.5556439873077, 0.0, 450.6651951082074],
    [0.0, 419.95102791138595, 286.0582452479318 ],
    [0.0, 0.0, 1.0] 
])
left_dist = np.asarray([
    -0.034339229817381384, -0.021817740794353273, 0.010405799829536749, -0.0003029112433404186, 0.03348575874823924 
])

# right camera calibration
right_cmtx = np.asarray([
    [417.3828537200518, 0.0, 476.96338990042244 ],
    [0.0, 422.3780509163499, 313.4542652200738 ],
    [0.0, 0.0, 1.0 ]
])
right_dist = np.asarray([
    -0.03072749884705801, -0.039628999396646006, 0.01814319227853198, -0.001784780200632385, 0.030838128082203216 
])

camera_focal_length = left_cmtx[0,0]
baseline = 5.792650427310503 


def merge_rectangles(rects, threshold=30.0):
    
    merged = []

    while rects:
        x, y, w, h = rects.pop(0)
        merged_rect = (x, y, x+w, y+h)

        i = 0
        while i < len(rects):
            x2, y2, w2, h2 = rects[i]
            other_rect = (x2, y2, x2+w2, y2+h2)

            if (max(merged_rect[0], other_rect[0]) - min(merged_rect[2], other_rect[2]) <= threshold and
                max(merged_rect[1], other_rect[1]) - min(merged_rect[3], other_rect[3]) <= threshold):
    
                merged_rect = (
                    min(merged_rect[0], other_rect[0]),
                    min(merged_rect[1], other_rect[1]),
                    max(merged_rect[2], other_rect[2]),
                    max(merged_rect[3], other_rect[3])
                )
                rects.pop(i)
        else:
            i += 1

        merged.append(merged_rect)
    return merged

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.publisher = self.create_publisher(Float64MultiArray, 'detected_objects', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        cap = cv.VideoCapture('/dev/video2')
        cap.set(cv.CAP_PROP_FPS, 20)
        cap.set(cv.CAP_PROP_FRAME_WIDTH, CAPTURE_RESOLUTION_X)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, CAPTURE_RESOLUTION_Y)
        
        # compute new camera matrices
        self.left_new_mtx, _ = cv.getOptimalNewCameraMatrix(left_cmtx,left_dist,(FRAME_WIDTH,FRAME_HEIGHT),1,(FRAME_WIDTH,FRAME_HEIGHT))
        self.right_new_mtx, _ = cv.getOptimalNewCameraMatrix(right_cmtx,right_dist,(FRAME_WIDTH,FRAME_HEIGHT),1,(FRAME_WIDTH,FRAME_HEIGHT))

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return
        
        # compute the depth map

        # split the capture into the left and right camera images
        left = frame[:,0:FRAME_WIDTH]
        right = frame[:,FRAME_WIDTH:]

        mapx,mapy = cv.initUndistortRectifyMap(left_cmtx,left_dist,None,self.left_new_mtx,(FRAME_WIDTH,FRAME_HEIGHT),5)
        left_undistorted = cv.remap(left,mapx,mapy,cv.INTER_LINEAR)

        mapx,mapy = cv.initUndistortRectifyMap(right_cmtx,right_dist,None,self.right_new_mtx,(FRAME_WIDTH,FRAME_HEIGHT),5)
        right_undistorted = cv.remap(right,mapx,mapy,cv.INTER_LINEAR)

        left_grayscale = cv.cvtColor(left_undistorted, cv.COLOR_BGR2GRAY)
        right_grayscale = cv.cvtColor(right_undistorted, cv.COLOR_BGR2GRAY)

        stereo = cv.StereoBM.create(numDisparities=32, blockSize=15)
        disparity = stereo.compute(left_grayscale, right_grayscale)

        depth = abs(camera_focal_length*baseline/disparity)

        # perform object detection in left camera frame

        mask = cv.inRange(left, LOWER_MATCH_COLOR, UPPER_MATCH_COLOR)
        edges = cv.Canny(mask, 100, 200)
        contours, _ = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        rect_img = left
        rects = [cv.boundingRect(cnt) for cnt in contours]
        merged_rects = merge_rectangles(rects)

        detected_objects = []
        frame_center = left.shape[1] // 2
    
        for (x1, y1, x2, y2) in merged_rects:
            w = x2 - x1
            h = y2 - y1
            if w < 30 or h < 30:
                continue

            cx = (x1 + x2)//2
            cy = (y1 + y2)//2

            distance = depth[cy,cx]

            dx = cx - frame_center
            angle = math.atan2(dx, camera_focal_length)
            detected_objects.extend((distance, angle))

            rect_img = cv.rectangle(rect_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv.putText(rect_img, f"{distance:.2f} cm", (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        msg = Float64MultiArray()
        msg.data = detected_objects
        self.publisher.publish(msg)

        cv.imshow('Mask', mask)
        cv.imshow('Edges', edges)
        cv.imshow('Rect Img', rect_img)

    def destroy(self):
        if self.cap:
            self.cap.release()
        cv.destroyAllWindows()
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        while rclpy.ok(): # while the node isn't shut down
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node has stopped cleanly.')
    except SystemExit:
        node.get_logger().info('Node is complete.')
    except BaseException as exc:
        type = exc.__class__.__name__
        node.get_logger().error(f'{type} exception in node has occured.')
        raise # raise without argument = raise the last exception
    finally:
        node.destroy()
        rclpy.shutdown() 

if __name__ == "__main__":
    main()