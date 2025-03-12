import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion
import math

class FakeOdometry(Node):
    def __init__(self):
        super().__init__('fake_odometry')
        
        self.wheel_separation = 0.5348 
        self.wheel_radius = 0.254
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.last_time = self.get_clock().now().nanoseconds

        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        self.create_subscription(Float32, "left_wheel_speed_radps", self.left_wheel_callback, 10)
        self.create_subscription(Float32, "right_wheel_speed_radps", self.right_wheel_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.02, self.update_odometry)

    def left_wheel_callback(self, msg: Float32):
        self.left_wheel_velocity = msg.data

    def right_wheel_callback(self, msg: Float32):
        self.right_wheel_velocity = msg.data
    
    def update_odometry(self):
        current_time = self.get_clock().now().nanoseconds
        dt = (current_time - self.last_time) / 1e9
        if dt < 1e-6:
            return
        self.last_time = current_time

        v = self.wheel_radius * (self.left_wheel_velocity + self.right_wheel_velocity) / 2
        w = self.wheel_radius * (self.right_wheel_velocity - self.left_wheel_velocity) / self.wheel_separation        

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        qw = math.cos(self.theta / 2)
        qx = 0.0
        qy = 0.0
        qz = math.sin(self.theta / 2)

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)
        
def main(args=None):
    rclpy.init(args=args)
    node = FakeOdometry()
    try:
        rclpy.spin(node)
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