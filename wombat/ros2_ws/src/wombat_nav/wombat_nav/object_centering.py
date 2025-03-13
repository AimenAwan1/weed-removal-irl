import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
import math
from nav_msgs.msg import Odometry

class ObjectCentering(Node):
    def __init__(self):
        super().__init__('object_centering')
        self.get_logger().info('Initializing object centering node')
        
        self.cmd_vel_left_pub = self.create_publisher(Float32,'cmd_left_wheel_vel_radps',10)
        self.cmd_vel_right_pub = self.create_publisher(Float32,'cmd_right_wheel_vel_radps',10)
        
        self.create_subscription(Float64MultiArray, 'detected_objects', self.detection_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        self.kp_angular = 0.05
        self.kp_linear = 0.1
        self.angle_tolerance = 0.05

        self.wheel_separation = 0.5348
        self.wheel_radius = 0.254 

        self.target_locked = False
        self.target_distance = None
        self.target_angle = None
        self.movement_threshold = 0.05
        self.current_position = None
        self.last_position = (0.0, 0.0)
        
    def detection_callback(self, msg: Float64MultiArray):
        if not msg.data:
            self.get_logger().info('No objects detected, stopping robot.')
            self.stop_robot()
            return
        
        if self.target_locked:
            return
        
        detected_objects = msg.data
        closest_distance = 10000.0
        closest_angle = 0

        for i in range(0, len(detected_objects), 2):
            if detected_objects[i] < closest_distance:
                closest_distance = detected_objects[i]
                closest_angle = detected_objects[i+1]

        self.target_distance = closest_distance
        self.target_angle = closest_angle
        self.target_locked = True
        self.last_position = self.get_robot_position()

        angle_error = self.target_angle
        if abs(angle_error) < self.angle_tolerance:
            angular_velocity = 0.0
        else:
            angular_velocity = self.kp_angular * angle_error

        linear_velocity = 0.1 if closest_distance > 0.1 else 0.0

        left_wheel_velocity = linear_velocity/self.wheel_radius-angular_velocity*self.wheel_separation/(2*self.wheel_radius)
        right_wheel_velocity = linear_velocity/self.wheel_radius+angular_velocity*self.wheel_separation/(2*self.wheel_radius) 

        self.cmd_vel_left_pub.publish(Float32(data=left_wheel_velocity))
        self.cmd_vel_right_pub.publish(Float32(data=right_wheel_velocity))

        if self.has_moved():
            self.target_locked = False
    
    def has_moved(self):
        current_position = self.get_robot_position()
        dx = current_position[0] - self.last_position[0]
        dy = current_position[1] - self.last_position[1]
        distance_moved = math.sqrt(dx**2 + dy**2)
        return distance_moved > 0.1

    def get_robot_position(self):
        if self.current_position is None:
            return (0.0, 0.0)
        return (self.current_position.x, self.current_position.y)
    
    def odom_callback(self, msg: Odometry):
        self.current_position = msg.pose.pose.position

    def stop_robot(self):
        self.cmd_vel_left_pub.publish(Float32(data=0.0))
        self.cmd_vel_right_pub.publish(Float32(data=0.0))

def main(args=None):

    rclpy.init(args=args)
    node = ObjectCentering()
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
