#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import cv2 as cv
import math
import pyrealsense2 as rs
from collections import deque

# BRG color values
#yellow
UPPER_MATCH_COLOR = np.array([140, 260, 255])
LOWER_MATCH_COLOR = np.array([110, 200, 160])
#white
#LOWER_MATCH_COLOR = np.array([150,150,150])
#UPPER_MATCH_COLOR = np.array([250, 250, 250])


def merge_rectangles(rects, threshold=30.0):
    merged = []
    rects = deque(rects)  # Use deque for efficient popping from the front

    while rects:
        x, y, w, h = rects.popleft()
        merged_rect = (x, y, x + w, y + h)
        i = 0

        while i < len(rects):
            x2, y2, w2, h2 = rects[i]
            other_rect = (x2, y2, x2 + w2, y2 + h2)

            if (max(merged_rect[0], other_rect[0]) - min(merged_rect[2], other_rect[2]) <= threshold and
                max(merged_rect[1], other_rect[1]) - min(merged_rect[3], other_rect[3]) <= threshold):

                merged_rect = (
                    min(merged_rect[0], other_rect[0]),
                    min(merged_rect[1], other_rect[1]),
                    max(merged_rect[2], other_rect[2]),
                    max(merged_rect[3], other_rect[3])
                )
                rects.remove(rects[i])  # More efficient than pop(i), avoids shifting elements
            else:
                i += 1  # Only increment if not merging

        merged.append(merged_rect)
    
    return merged


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_detection_calibrate_color')
        self.publisher = self.create_publisher(Float64MultiArray, 'detected_objects', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        
    def timer_callback(self):
        frame = self.pipeline.wait_for_frames()
        depth = frame.get_depth_frame()
        colour = frame.get_color_frame()

        if not depth or not colour:
            self.get_logger().error('Failed to capture frame')
            return
        
        self.get_logger().info('Reading camera frame')

        colour_image = np.asanyarray(colour.get_data())
        depth_image = np.asanyarray(depth.get_data()) * 0.001

        mask = cv.inRange(colour_image, LOWER_MATCH_COLOR, UPPER_MATCH_COLOR)
        edges = cv.Canny(mask, 100, 200)
        contours, _ = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        rect_img = colour_image.copy()
        rects = [cv.boundingRect(cnt) for cnt in contours]
        merged_rects = merge_rectangles(rects)

        detected_objects = []
        frame_center = colour_image.shape[1] // 2
    
        for (x1, y1, x2, y2) in merged_rects:
            #if w < 30 or h < 30:
            #    continue

            distance = depth_image[(y1 + y2)//2, (x1 + x2)//2]
            angle = (((x1 + x2)// 2 - frame_center) / 640  * (86 * math.pi / 180))
            
            self.get_logger().info(f"distance: {distance}")
            self.get_logger().info(f"angle: {angle}")

            detected_objects.append(float(distance))
            detected_objects.append(float(angle))

            rect_img = cv.rectangle(rect_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv.putText(rect_img, f"{distance:.2f} m", (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        msg = Float64MultiArray()
        msg.data = detected_objects
        self.publisher.publish(msg)
        self.get_logger().info('Published data')

        cv.imshow('Mask', mask)
        cv.imshow('Edges', edges)
        cv.imshow('Rect Img', rect_img)
        cv.waitKey(1)

    def destroy(self):
        self.pipeline.stop()
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
