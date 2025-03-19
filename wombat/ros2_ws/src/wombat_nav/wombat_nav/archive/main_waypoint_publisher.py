#move to hardcoded main waypoints
import os
import yaml
import rclpy
from rclpy.node import Node
import math
from time import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        self.subscription = self.create_subscription(Odometry, '/robot_position', self.robot_position_callback, 10)
        self.target_pub = self.create_publisher(Point, '/target_position', 10)
        
        #current robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        #load pre-defined waypoints from yaml file
        self.main_waypoints = self.load_waypoints()
        self.get_logger().info(f"Loaded main waypoints: {self.main_waypoints}")
        self.current_main_index = 0

        self.waypoint_threshold = 0.1

        self.last_time_reached = time()
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

    def timer_callback(self):
        if self.current_main_index >= len(self.main_waypoints):
            self.get_logger().info('All main waypoints reached.')
            return
            
        target = self.main_waypoints[self.current_main_index]
        
        if self.is_target_reached(target):
            current_time = time()
            if current_time - self.last_time_reached >= 10: 
                self.get_logger().info(f"reached main waypoint {self.current_main_index}: {target}")
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