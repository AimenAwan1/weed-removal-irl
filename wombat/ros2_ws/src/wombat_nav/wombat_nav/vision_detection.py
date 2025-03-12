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
LOWER_MATCH_COLOR = np.array([30, 120, 0])
#white
#LOWER_MATCH_COLOR = np.array([150,150,150])
#UPPER_MATCH_COLOR = np.array([250, 250, 250])

KNOWN_WIDTH_CM = 11  
FOCAL_LENGTH = 886.9545454545455

OUTPUT_FILE = "detected_objects.txt"

def merge_rectangles(self, rects, threshold=30.0):
    
    self.get_logger().info(' merge 1')
    merged = []
    self.get_logger().info(' merge 2')

    while rects:
        x, y, w, h = rects.pop(0)
        self.get_logger().info(' merge 3')
        merged_rect = (x, y, x+w, y+h)
        self.get_logger().info(' merge 4')

        i = 0
        while i < len(rects):
            x2, y2, w2, h2 = rects[i]
            other_rect = (x2, y2, x2+w2, y2+h2)
            self.get_logger().info(' merge 5')

            if (max(merged_rect[0], other_rect[0]) - min(merged_rect[2], other_rect[2]) <= threshold and
                max(merged_rect[1], other_rect[1]) - min(merged_rect[3], other_rect[3]) <= threshold):
                self.get_logger().info(' merge 6')
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
        self.get_logger().info(' merge 7')
    return merged

def calculate_distance(focal_length, real_width, width_in_pixels):
    return (real_width * focal_length) / width_in_pixels

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_detection')
        self.publisher = self.create_publisher(Float64MultiArray, 'detected_objects', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.cap = cv.VideoCapture('/dev/video0')
        self.cap.set(cv.CAP_PROP_FPS, 20)
        
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return
        
        self.get_logger().info('Reading camera frame')
        mask = cv.inRange(frame, LOWER_MATCH_COLOR, UPPER_MATCH_COLOR)
        edges = cv.Canny(mask, 100, 200)
        contours, _ = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        rect_img = frame.copy()
        rects = [cv.boundingRect(cnt) for cnt in contours]
        merged_rects = rects #merge_rectangles(self, rects)

        detected_objects = []
        frame_center =frame.shape[1] // 2
    
        for (x1, y1, x2, y2) in merged_rects:
            w = x2 - x1
            h = y2 - y1
            if w < 30 or h < 30:
                continue

            aspect_ratio = h / w
            theta = np.arctan(aspect_ratio)
            corrected_width = w / np.cos(theta)

            distance = calculate_distance(FOCAL_LENGTH, KNOWN_WIDTH_CM, corrected_width)
            angle = ((x1 + w / 2 - frame_center) * (math.pi / 3))
            detected_objects.extend((distance, angle))

            rect_img = cv.rectangle(rect_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv.putText(rect_img, f"{distance:.2f} cm", (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


        msg = Float64MultiArray()
        msg.data = detected_objects
        self.publisher.publish(msg)
        self.get_logger().info('Published data')

        #cv.imshow('Mask', mask)
        #cv.imshow('Edges', edges)
        #cv.imshow('Rect Img', rect_img)

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
