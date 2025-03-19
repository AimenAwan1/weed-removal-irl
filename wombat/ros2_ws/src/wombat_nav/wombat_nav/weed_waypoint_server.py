import os
import yaml
import rclpy
from rclpy.node import Node
import subprocess
import math
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray

from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        self.subscription = self.create_subscription(Odometry, '/robot_position', self.robot_position_callback, 10)
        self.target_pub = self.create_publisher(Point, '/target_position', 10)
        self.create_subscription(Float64MultiArray, '/detected_objects', self.detection_callback, 10)

        self.service = self.create_service()
        
        #current robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

    def robot_position_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        #euler from quaternion
        q = msg.pose.pose.orientation
        self.robot_theta = Rotation.from_quat(
                [
                    q.x,
                    q.y,
                    q.z,
                    q.w,
                ]
            ).as_euler("xyz", degrees=True)[2]
        
        #TF

    def detection_callback(self, msg: Float64MultiArray):
        
        branch_points = []
        data = msg.data
        for i in range (0, len(data), 2):
            distance = data[i]
            angle = data[i+1]

            new_x = self.robot_x + distance * math.cos(self.robot_theta + angle)
            new_y = self.robot_y + distance * math.sin(self.robot_theta + angle)

            branch_points.append((new_x, new_y))

            self.get_logger().info(f"Received branch waypoint: ({new_x:.2f}, {new_y:.2f})")
        self.branch_waypoints = branch_points
    
    def publish_target(self, target):
        #replace with client to waypoint action server
        point = Point()
        point.x = target[0]
        point.y = target[1]
        self.target_pub.publish(point)
        self.get_logger().info(f"Publishing target: ({point.x}, {point.y})")
    
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