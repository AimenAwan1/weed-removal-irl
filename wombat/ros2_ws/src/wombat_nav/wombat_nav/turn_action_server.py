import time
import numpy as np

from scipy.spatial.transform import Rotation

import rclpy
from rclpy.action import ActionServer, CancelResponse
import rclpy.callback_groups
import rclpy.executors
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry

from wombat_msgs.action import TurnAction

TURN_ACTION = "turn_action"
TURN_ACTION_FEEDBACK_HZ = 2

ODOMETRY_TOPIC = "/robot_position"

CHASSIS_VEL_TOPIC = "/chassis_vel"

# NOTE: these are the same gains as in the waypoint action server
KP_ANGULAR = 1.4
KV_ANGULAR = 0.0  # 0.1/1
KI_ANGULAR = 0.0  # 0.5/2

CONTROL_LOOP_TIMER_HZ = 30

ALIGNMENT_ERROR_THRESHOLD_RAD = np.pi/24 # np.pi/6  # 30 degrees in alignment


class TurnActionServer(Node):

    def __init__(self):
        super().__init__("waypoint_action_server")
        self.global_count_ = 0
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        # run initialization
        self.current_ang = 0

        self.target_ang = self.current_ang
        self.controller_enabled = False

        self.reset_error()

        self._action_server = ActionServer(
            self, TurnAction, TURN_ACTION, self.execute_callback,
            callback_group=self.callback_group,
            cancel_callback=self.cancel_callback
        )

        self.odom_subscription = self.create_subscription(
            Odometry, ODOMETRY_TOPIC, self.odom_callback, 10,
            callback_group=self.callback_group
        )
        self.chassis_vel_publisher = self.create_publisher(
            Twist, CHASSIS_VEL_TOPIC, 10)
        self.create_timer(1.0 / CONTROL_LOOP_TIMER_HZ,
                          self.control_loop_timer_callback)

    def reset_error(self):
        self.error = np.array([0, 0])

        self.prev_error_angular = 0
        self.integral_error_angular = 0

    def odom_callback(self, msg: Odometry):
        # self.get_logger().info("Received odom callback")
        q = msg.pose.pose.orientation
        self.current_ang = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler(
            "xyz", degrees=False
        )[2]

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        zero_speed = Twist()
        self.chassis_vel_publisher.publish(zero_speed)
        return CancelResponse.ACCEPT

    def control_loop_timer_callback(self):
        if self.controller_enabled:
            self.get_logger().info('control loop active')
            
            curr_time = self.get_clock().now()
            dt = (curr_time - self.prev_time).nanoseconds / 1e9
            self.prev_time = curr_time

            # angular velocity
            target_ang = self.target_ang  # copies to apply transformation for wraparound handling

            if np.abs(target_ang) > 2*np.pi/3:
                error_angular = (
                    target_ang+2*np.pi) % (2*np.pi) - (self.current_ang+2*np.pi) % (2*np.pi)
            else:
                error_angular = target_ang - self.current_ang
            error_angular = np.clip(
                error_angular, a_min=-np.pi/6, a_max=np.pi/6)

            # if np.abs(error_angular) >= np.pi:
            #      error_angular = -1*np.sign(error_angular)*(2*np.pi - np.abs(error_angular))

            self.get_logger().info(
                f"error_angular={error_angular}, target_angle={target_ang}, current_ang={self.current_ang}")
            self.integral_error_angular += error_angular * dt
            self.get_logger().info(
                f"integral error: {self.integral_error_angular}")
            deriv_error_angular = (
                error_angular - self.prev_error_angular) / dt
            self.get_logger().info(
                f"deriv_error_angular: {deriv_error_angular}")
            self.prev_error_angular = error_angular

            w = (
                KP_ANGULAR * error_angular
                + KV_ANGULAR * deriv_error_angular
                + KI_ANGULAR * self.integral_error_angular
            )

            self.get_logger().info("before chassis vel publish")
            self.get_logger().info(f"w: {w}, type={type(w)}")

            chassis_vel_msg = Twist()
            chassis_vel_msg.angular.z = float(w)
            self.chassis_vel_publisher.publish(chassis_vel_msg)

            self.get_logger().info("after chassis vel publish")

            # has reached the correct location
            if np.linalg.norm(error_angular) < ALIGNMENT_ERROR_THRESHOLD_RAD:
                self.controller_enabled = False

    def execute_callback(self, goal_handle):
        self.get_logger().info("Starting navigation to waypoint")

        self.target_ang = goal_handle.request.target.data

        self.controller_enabled = True
        self.prev_time = self.get_clock().now()
        self.reset_error()

        while self.controller_enabled:
            # send updates on the progress of the control loop

            feedback_msg = TurnAction.Feedback()
            feedback_msg.current.data = self.current_ang
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info("after publish feedback")

            time.sleep(1.0 / TURN_ACTION_FEEDBACK_HZ)

            self.get_logger().info("after time sleep")

        self.get_logger().info("terminated control loop")

        zero_speed = Twist()
        self.chassis_vel_publisher.publish(zero_speed)

        # finally reached waypoint
        goal_handle.succeed()
        result_msg = TurnAction.Result()
        result_msg.final.data = self.current_ang
        result_msg.error.data = self.target_ang - self.current_ang

        return result_msg


def main(args=None):
    rclpy.init(args=args)
    node = TurnActionServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
