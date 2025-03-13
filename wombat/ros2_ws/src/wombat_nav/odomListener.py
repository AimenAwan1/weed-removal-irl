import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.creat_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        
        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z

def main(args=None):

    rclpy.init(args=args)
    node = OdomListener()
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
