import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray

class ObjectCentering(Node):
    def __init__(self):
        super().__init__('object_centering')
        self.get_logger().info('Initializing object centering node')
        self.cmd_vel_left_pub = self.create_publisher(Float32,'cmd_left_wheel_vel_radps',10)
        self.cmd_vel_right_pub = self.create_publisher(Float32,'cmd_right_wheel_vel_radps',10)
        
        self.create_subscription(Float64MultiArray, 'detected_objects', self.detection_callback, 10)

        self.frame_center = 300
        self.kp_angular = 0.005
        self.kp_linear = 0.1

        self.wheel_separation = 0.5348
        self.wheel_radius = 0.254 
        
    def detection_callback(self, msg: Float64MultiArray):
        if not msg.data:
            self.get_logger().info('No objects detected, stopping robot.')
            self.stop_robot()
            return

        detected_objects = msg.data   
        closest_object_index = detected_objects.index(min(detected_objects[::2]))
        object_angle = detected_objects[closest_object_index + 1]

        angle_error = object_angle
        angular_velocity = self.kp_angular * angle_error

        closest_distance = detected_objects[closest_object_index]
        linear_velocity = 0.1 if closest_distance > 0.2 else 0.0

        left_wheel_velocity = linear_velocity/self.wheel_radius-angular_velocity*self.wheel_separation/(2*self.wheel_radius)
        right_wheel_velocity = linear_velocity/self.wheel_radius+angular_velocity*self.wheel_separation/(2*self.wheel_radius) 

        self.cmd_vel_left_pub.publish(Float32(data=left_wheel_velocity))
        self.cmd_vel_right_pub.publish(Float32(data=right_wheel_velocity))

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
