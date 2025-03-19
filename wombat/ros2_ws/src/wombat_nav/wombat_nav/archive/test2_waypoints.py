#move to main waypoint, trigger camera node, create new branch way points to travel to
import os
import yaml
import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from wombat_msgs.srv import DetectObjects

class WaypointPublisherTestTwo(Node):
    def __init__(self):
        super().__init__('waypoint_publisher_test_two')

        self.subscription = self.create_subscription(Odometry, '/robot_position', self.robot_position_callback, 10)
        self.target_pub = self.create_publisher(Point, '/target_position', 10)
        self.client = self.create_client(DetectObjects, 'detect_objects')
        
        #current robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        #load pre-defined waypoints from yaml file
        self.main_waypoints = self.load_waypoints()
        self.get_logger().info(f"Loaded main waypoints: {self.main_waypoints}")
        self.current_main_index = 0
        self.current_branch_index = 0

        self.waypoint_threshold = 0.05
        self.branch_waypoints = []

        self.is_following_branch_waypoint = False

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

    def timer_callback(self):
        #if not more waypoints, return
        if self.current_main_index >= len(self.main_waypoints):
            self.get_logger().info('All main waypoints reached.')
            rclpy.shutdown()
            
        target = self.main_waypoints[self.current_main_index]
        
        #if main waypoint reached, trigger object detection, 
        #if branch waypoint reached, go to next branch waypoint
        #else keep sending target
        if self.is_target_reached(target):
            if not self.is_following_branch_waypoint:
                self.get_logger().info(f"reached main waypoint {self.current_main_index}: {target}")  
                self.trigger_object_detection()
            else:
                self.navigate_branch_waypoints()
        else:
            self.publish_target(target)

    def trigger_object_detection(self):
        self.branch_waypoints = []  # do this so its empty if it fails

        if not self.client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error('Object detection service is not available')
            return
        
        try:
            request = DetectObjects.Request()
            response = self.client.call(request)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return

        if response.success:
            self.get_logger().info("Object detection completed successfully")
        else:
            self.get_logger().warn("Object detection failed")
            return
        
        # service call returned successfully

        for i in range(0, len(response.data), 2):
            distance = response.data[i]
            angle = response.data[i+1]

            new_x = self.robot_x + distance * math.cos(math.radians(self.robot_theta + angle))
            new_y = self.robot_y + distance * math.sin(math.radians(self.robot_theta + angle))
            self.get_logger().info("calculated branch way points: {new_x}, {new_y}")
            self.branch_waypoints.append((new_x, new_y))

        self.is_following_branch_waypoint = True

        self.current_branch_index = 0  # reset this index as there are new branch waypoints after detection
        self.navigate_branch_waypoints()  # triggers following the first branch waypoint        

    def navigate_branch_waypoints(self):
        if self.current_branch_index < len(self.branch_waypoints):
            self.publish_target(self.branch_waypoints[self.current_branch_index])
            
            if self.is_target_reached(self.branch_waypoints[self.current_branch_index]):
                current_branch_index += 1
                self.get_logger().info('incremented branch waypoint')

        else:
            self.get_logger().info(f"Finished branch waypoints for main waypoint {self.current_main_index}")
            self.current_main_index += 1
            self.get_logger().info('incremented main waypoint')
            self.is_following_branch_waypoint = False  # indicates that we have finished the branch waypoints 
            
            if self.current_main_index < len(self.main_waypoints):
                self.publish_target(self.main_waypoints[self.current_main_index])
    
def main(args=None):
    rclpy.init(args=args)
    
    node = WaypointPublisherTestTwo()
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