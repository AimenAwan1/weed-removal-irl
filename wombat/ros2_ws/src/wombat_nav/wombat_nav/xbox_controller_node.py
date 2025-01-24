import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class XboxControllerNode(Node):
    def __init__(self):
        super().__init__('xbox_controller_node')

        self.velocity_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/chassis_vel',
            10
        )

    def joy_callback(self, msg):
        twist = Twist()

        twist.linear.x = 0.5 * msg.axes[1]  #left stick vertical
        twist.angular.z = 1.0 * msg.axes[3] #right stick horizontal

        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'Published linear velocity: {twist.linear.x}, angular velocity: {twist.angular.z}')
        
def main(args=None):
    rclpy.init(args=args)
    node = XboxControllerNode()
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