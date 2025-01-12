#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class RobotMover(object):

    def __init__(self, value_BASE_PWM, value_MULTIPLIER_STANDARD, value_MULTIPLIER_PIVOT, value_simple_mode):
        rospy.Subscriber("/morpheus_bot/cmd_vel", Twist, self.cmd_vel_callback)
        self.motor_driver = MotorDriver( i_BASE_PWM=value_BASE_PWM,
                                         i_MULTIPLIER_STANDARD=value_MULTIPLIER_STANDARD,
                                         i_MULTIPLIER_PIVOT=value_MULTIPLIER_PIVOT,
                                         simple_mode=value_simple_mode)
        rospy.wait_for_service('/raspicam_node/start_capture')

        rospy.loginfo("RobotMover Started...")


    def cmd_vel_callback(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Decide Speed
        self.motor_driver.set_cmd_vel(linear_speed, angular_speed)


    def listener(self):
        rospy.spin()



def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsController()
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