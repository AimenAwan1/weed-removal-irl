import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry

class OdomTransform(Node):
    def __init__(self):
        super().__init__('odom_transform')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):

    rclpy.init(args=args)
    node = OdomTransform()
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
