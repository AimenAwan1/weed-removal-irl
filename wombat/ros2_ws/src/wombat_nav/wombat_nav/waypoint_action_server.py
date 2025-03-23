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

from wombat_msgs.action import WaypointAction

from utilities.error_angle import compute_err_angle 

WAYPOINT_ACTION = "waypoint_action"
WAYPOINT_ACTION_FEEDBACK_HZ = 2

WAYPOINT_ERROR_DIST_THRESHOLD = 0.03

ODOMETRY_TOPIC = "/robot_position"

CHASSIS_VEL_TOPIC = "/chassis_vel"

KP_LINEAR = 1.0 # 0.8
KV_LINEAR = 0.0  # 0.001
KI_LINEAR = 0.0  # 0.5/2

KP_ANGULAR = 1.4
KV_ANGULAR = 0.0  # 0.1/1
KI_ANGULAR = 0.0  # 0.5/2

CONTROL_LOOP_TIMER_HZ = 30

INITIAL_CONTROLLER_ALIGNMENT_RAD = np.pi/24 # np.pi/6  # 30 degrees in alignment
# once this close turn off angle controller (prevents jumping)
DISTANCE_TILL_ANGLE_SHUTOFF_M = 0.2

MAX_SPEED_LIN_MS = 0.25


class WaypointActionServer(Node):

    def __init__(self):
        super().__init__("waypoint_action_server")
        self.global_count_ = 0
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        # run initialization
        self.current_position = Point(x=0.0, y=0.0)
        self.current_ang = 0

        self.target_position = self.current_position
        self.waypoint_nav_enabled = False

        self.reset_error()

        self._action_server = ActionServer(
            self, WaypointAction, WAYPOINT_ACTION, self.execute_callback,
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
        self.aligned_with_direction = False

        self.prev_error_linear = 0
        self.prev_error_angular = 0
        self.integral_error_linear = 0
        self.integral_error_angular = 0

    def odom_callback(self, msg: Odometry):
        # self.get_logger().info("Received odom callback")
        self.current_position = msg.pose.pose.position

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
        if self.waypoint_nav_enabled:
            self.get_logger().info('control loop active')
            self.error = np.array(
                [
                    self.target_position.x - self.current_position.x,
                    self.target_position.y - self.current_position.y,
                ]
            )
            self.get_logger().info(f'current error: {self.error}')

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
            current_error_angle = np.arctan2(
                self.error[1],
                self.error[0])

            error_angular = compute_err_angle(self.current_ang, current_error_angle)
            # if np.abs(current_error_angle) > 2*np.pi/3:
            #     error_angular = (
            #         current_error_angle+2*np.pi) % (2*np.pi) - (self.current_ang+2*np.pi) % (2*np.pi)
            # else:
            #     error_angular = current_error_angle - self.current_ang
            error_angular = np.clip(
                error_angular, a_min=-np.pi/6, a_max=np.pi/6)

            # if np.abs(error_angular) >= np.pi:
            #      error_angular = -1*np.sign(error_angular)*(2*np.pi - np.abs(error_angular))

            self.get_logger().info(
                f"error_angular={error_angular}, current_error_angle={current_error_angle}, current_ang={self.current_ang}")
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

            # checks controller stages

            # disable the linear controller until angle alignment with the target position is
            # within an allowable distance (prevents situations where unpredictable behavior
            # occurs due to a waypoint being behind the robot)
            if np.abs(error_angular) < INITIAL_CONTROLLER_ALIGNMENT_RAD:
                self.aligned_with_direction = True
            v = v if self.aligned_with_direction else 0.0
            v = np.clip(v, a_min=0, a_max=MAX_SPEED_LIN_MS)

            # prevents jumping when a bit of overshoot results in the error vector inverting
            # causing the robot to begin spinning around before shutting off as within allowances
            w = w if error_linear > DISTANCE_TILL_ANGLE_SHUTOFF_M else 0.0

            self.get_logger().info("before chassis vel publish")

            self.get_logger().info(f"v: {v}, type={type(v)}")
            self.get_logger().info(f"w: {w}, type={type(w)}")

            chassis_vel_msg = Twist()
            chassis_vel_msg.linear.x = float(v)
            chassis_vel_msg.angular.z = float(w)
            self.chassis_vel_publisher.publish(chassis_vel_msg)

            self.get_logger().info("after chassis vel publish")

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
            # send updates on the progress of the control loop

            feedback_msg = WaypointAction.Feedback()
            feedback_msg.current = self.current_position
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info("after publish feedback")

            time.sleep(1.0 / WAYPOINT_ACTION_FEEDBACK_HZ)

            self.get_logger().info("after time sleep")

        self.get_logger().info("terminated control loop")

        zero_speed = Twist()
        self.chassis_vel_publisher.publish(zero_speed)

        # finally reached waypoint
        goal_handle.succeed()
        result_msg = WaypointAction.Result()
        result_msg.final = self.current_position
        result_msg.error = Point(x=self.error[0], y=self.error[1])

        return result_msg


def main(args=None):
    rclpy.init(args=args)
    node = WaypointActionServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
