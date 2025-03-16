#supposed to do everything (does'nt work)
import os
import yaml
import rclpy
from rclpy.node import Node
import math
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        self.subscription = self.create_subscription(Odometry, '/robot_position', self.robot_position_callback, 10)
        self.target_pub = self.create_publisher(Point, '/target_position', 10)
        self.client = self.create_client(Trigger, 'detect_objects')
        self.create_subscription(Float64MultiArray, '/detected_objects', self.detection_callback, 10)

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for vision service...")
        
        #current robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        #load pre-defined waypoints from yaml file
        self.main_waypoints = self.load_waypoints()
        self.get_logger().info(f"Loaded main waypoints: {self.main_waypoints}")
        self.current_main_index = 0

        #list for waypoints from camera detection
        self.branch_waypoints = []
        self.current_branch_index = 0

        self.state = "MOVE_MAIN"

        self.waypoint_threshold = 0.1
        self.create_timer(1.0, self.timer_callback)

    def load_waypoints(self):
        package_share_directory = get_package_share_directory('wombat_nav')
        yaml_file = os.path.join(package_share_directory, 'config', 'main_waypoints.yaml')
        try:
            with open(yaml_file, 'r') as f:
                data = yaml.safe_load(f)
            return data.get('main_waypoints', [])
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            return []

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

    def request_object_detection(self):
        self.get_logger().info("requesting object detection")
        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.object_detection_callback)

    def object_detection_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Vision service responded: {response.message}")
            else:
                self.get_logger().warn("Vision service failed.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.state = "MOVE_MAIN"

    def detection_callback(self, msg: Float64MultiArray):
        if self.state == "DETECT":
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

    def timer_callback(self):
        if self.state == "MOVE_MAIN":
            if self.current_main_index >= len(self.main_waypoints):
                self.get_logger().info('All main waypoints reached.')
                return
            
            target = self.main_waypoints[self.current_main_index]
            #If main target reached, run object detection, else send target
            if self.is_target_reached(target):
                self.get_logger().info(f"reached main waypoint {self.current_main_index}: {target}")
                self.state = "DETECT"
                self.detection_start_time = self.get_clock().now()
                self.request_object_detection()
            else:
                self.publish_target(target)

        #if objects detected, move to branch waypoints, else continue with main waypoints
        elif self.state == 'DETECT':
            now = self.get_clock().now()
            dt = (now - self.detection_start_time).nanoseconds / 1e9
            if dt > 1.0:
                if self.branch_waypoints:
                    self.get_logger().info("Branch waypoint detected")
                    self.state = "MOVE_BRANCH"
                    self.current_branch_index = 0
                    self.publish_target(self.branch_waypoints[self.current_branch_index])
                else:
                    self.current_main_index += 1
                    self.state = "MOVE_MAIN"
                    if self.current_main_index < len(self.main_waypoints):
                        self.publish_target(self.main_waypoints[self.current_main_index])
                    else: 
                        self.get_logger().info("Finished navigating main waypoints")
            
        elif self.state == "MOVE_BRANCH":
            #if branch waypoints done, go back to main waypoints
            if self.current_branch_index >= len(self.branch_waypoints):
                self.get_logger().info("Completed branch waypoints, resuming main waypoints")
                self.branch_waypoints = []
                self.state = "MOVE_MAIN" 
                self.current_main_index += 1
                if self.current_main_index < len(self.main_waypoints):
                    self.publish_target(self.main_waypoints[self.current_main_index])
                else:
                    self.get_logger().info("Finished navigating main waypoints")
                return

            #cycle through rest of branch waypoints
            target = self.branch_waypoints[self.current_branch_index]
            if self.is_target_reached(target):
                self.get_logger().info(f"Reached branch waypoint {self.current_branch_index}: {target}")
                
                self.current_branch_index += 1
                if self.current_branch_index < len(self.branch_waypoints):
                    self.publish_target(self.branch_waypoints[self.current_branch_index])
                else:
                    self.branch_waypoints = []
                    self.state = "MOVE_MAIN"
                    self.current_main_index += 1
                    if self.current_main_index < len(self.main_waypoints):
                        self.publish_target(self.main_waypoints[self.current_main_index])
                    else:
                        self.get_logger().info("Finished navigating main waypoints")
            else: 
                self.publish_target(target)
    
    def is_target_reached(self, target):
        tx, ty = target
        distance = math.sqrt((tx - self.robot_x) ** 2 + (ty - self.robot_y) ** 2)
        return distance < self.waypoint_threshold
    
    def publish_target(self, target):
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