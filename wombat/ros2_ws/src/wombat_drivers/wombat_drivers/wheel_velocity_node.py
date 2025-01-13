import rclpy
from rclpy.node import Node
from wombat_drivers import MotorDriver
from wombat_msgs.msg import WheelVelocities

class WheelVelocityNode(Node):
    def __init__(self):
        super().__init__('wheel_velocity_node')
        self.velocity_subscriber = self.create_subscription(
            WheelVelocities,
            'wheel_velocity',
            self.wheel_velocity_callback,
            10
        )
        self.motor_driver = MotorDriver(base_pwm=50)

    def wheel_velocity_callback(self, msg):
        self.get_logger().info(f'Received wheel velocities. Left: {msg.left_velocity}, Right: {msg.right_velocity}')
        
        self.motor_driver.set_speed(msg.left_velocity, msg.right_velocity)

def main(args=None):
    rclpy.init(args=args)
    node = WheelVelocityNode()
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