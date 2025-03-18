import time
import numpy as np

from scipy.spatial.transform import Rotation

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry

from wombat_msgs.action import WaypointAction

WAYPOINT_ACTION = "waypoint_action"
WAYPOINT_FEEDBACK_RATE_HZ = 2

WAYPOINT_ERROR_DIST_THRESHOLD = 0.05

ODOMETRY_TOPIC = "/robot_position"

CHASSIS_VEL_TOPIC = "/chassis_vel"

KP_LINEAR = 5.0
KV_LINEAR = 0.001
KI_LINEAR = 0.1

KP_ANGULAR = 5.0
KV_ANGULAR = 0.01
KI_ANGULAR = 0.2


class WaypointActionServer(Node):

    def __init__(self):
        super().__init__("waypoint_action_server")

        # run initialization
        self.current_position = Point(x=0.0, y=0.0)
        self.current_ang = 0

        self.target_position = self.current_position
        self.waypoint_nav_enabled = False

        self.reset_error()

        super._action_server = ActionServer(
            self, WaypointAction, WAYPOINT_ACTION, self.execute_callback
        )
        self.odom_subscription = self.create_subscription(
            Odometry, ODOMETRY_TOPIC, self.odom_callback, 10
        )
        self.chassis_vel_publisher = self.create_publisher(Twist, CHASSIS_VEL_TOPIC, 10)

    def reset_error(self):
        self.error = np.array([0, 0])
        self.prev_error_linear = 0
        self.prev_error_angular = 0
        self.integral_error_linear = 0
        self.integral_error_angular = 0

    def odom_callback(self, msg: Odometry):
        self.current_position = msg.pose.pose.position

        q = msg.pose.pose.orientation
        self.current_ang = Rotation.from_quat([q.w, q.x, q.y, q.z]).as_euler(
            "xyz", degrees=False
        )[2]

    def timer_callback(self):
        if self.waypoint_nav_enabled:
            self.error = np.array(
                [
                    self.target_position.x - self.current_position.x,
                    self.target_position.y - self.current_position.y,
                ]
            )

            curr_time = self.get_clock().now()
            dt = (curr_time - self.prev_time).nanoseconds / 1e9
            self.prev_time = curr_time

            # linear velocity
            error_linear = np.linalg.norm(self.error)
            self.integral_error_linear += error_linear * dt
            deriv_error_linear = (error_linear - self.prev_error_linear) / dt
            self.prev_error_linear = error_linear

            v = (
                KP_LINEAR * error_linear
                + KV_LINEAR * deriv_error_linear
                + KI_LINEAR * self.integral_error_linear
            )

            # angular velocity
            error_angular = np.arctan2(self.error[0], self.error[1]) - self.current_ang
            self.integral_error_angular += error_angular * dt
            deriv_error_angular = (error_angular - self.prev_error_angular) / dt
            self.prev_error_angular = error_angular

            w = (
                KP_ANGULAR * error_angular
                + KV_ANGULAR * deriv_error_angular
                + KI_ANGULAR * self.integral_error_angular
            )

            chassis_vel_msg = Twist()
            chassis_vel_msg.linear.x = v
            chassis_vel_msg.angular.z = w
            self.chassis_vel_publisher.publish(chassis_vel_msg)

            # has reached the correct location
            if np.linalg.norm(self.error) < WAYPOINT_ERROR_DIST_THRESHOLD:
                self.waypoint_nav_enabled = False

    def execute_callback(self, goal_handle):
        self.get_logger().info("Starting navigation to waypoint")

        self.target_position = goal_handle.request.target

        self.waypoint_nav_enabled = True
        self.prev_time = self.get_clock().now()
        self.reset_error()

        while self.waypoint_nav_enabled:
            # track progress of control loop

            feedback_msg = WaypointAction.Feedback()
            feedback_msg.current = self.current_position
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(1.0 / WAYPOINT_FEEDBACK_RATE_HZ)

        # finally reached waypoint
        goal_handle.succeed()
        result_msg = WaypointAction.Result()
        result_msg.final = self.current_position
        result_msg.error = Point(x=self.error[0], y=self.error[1])


def main(args=None):
    rclpy.init(args=args)
    node = WaypointActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
