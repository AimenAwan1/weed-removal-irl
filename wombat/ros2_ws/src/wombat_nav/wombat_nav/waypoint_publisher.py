import rclpy
from rclpy.node import Node
import subprocess
import time
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose

#DETECTION_SCRIPT = "/home/aimen/weed-removal-irl/wombat/vision/flag_detection_video_new_alg.py"
DETECTED_OBJECTS_FILE = "/home/aimen/weed-removal-irl/wombat/vision/detected_objects.txt"

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.waypoint_pub = self.create_publisher(PoseArray, 'waypoints', 10)

        self.waypoints = []

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.timer = self.create_timer(1.0, self.update_waypoints)
    
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

    def update_waypoints(self):
        try:
            with open(DETECTED_OBJECTS_FILE, "r") as f:
                lines = f.readlines()
        except FileNotFoundError:
            self.get_logger().warn("Detection file not found.")
            return
        
        for line in lines:
            try:
                distance, angle = map(float, line.split())

                waypoint_x = self.robot_x + distance * math.cos(self.robot_theta + angle)
                waypoint_y = self.robot_y + distance * math.sin(self.robot_theta + angle)

                self.waypoints.append((waypoint_x, waypoint_y))
            except ValueError:
                continue

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "map"

        for x, y in self.waypoints:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose_array.poses.append(pose)

        self.waypoint_pub.publish(pose_array)
        self.get_logger().info(f"Published {len(self.waypoints)} waypoints.")

    def euler_from_quaternion(self, quat):
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw
    
def main(args=None):
    rclpy.init(args=args)
    
    node = WaypointPublisher()
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
        node.destroy_node()
        rclpy.shutdown() 

if __name__=='__main__':
    main()
        