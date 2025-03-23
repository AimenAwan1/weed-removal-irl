import numpy as np
from scipy.spatial.transform import Rotation

import rclpy.callback_groups

import rclpy.executors
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse

from wombat_msgs.action import WeedsSprayAction, WaypointAction
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


WEEDS_SPRAY_ACTION = "weeds_spray_action"
WEED_SEPARATION_DIST = 0.1
CHASSIS_WIDTH_M = 0.4508

DISTANCE_EPSILON = 1e-3

WAYPOINT_ACTION = "waypoint_action"  # action server to allow movement


class WeedsSprayActionServer(Node):
    def __init__(self):
        super().__init__("weeds_spray_action_server")
        self.global_count = 0
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.current_position = Point(x=0.0, y=0.0)
        self.current_ang = 0

        self._action_server = ActionServer(
            self,
            WeedsSprayAction,
            WEEDS_SPRAY_ACTION,
            self.execute_callback,
            callback_group=self.callback_group,
            cancel_callback=self.cancel_callback,
        )

        self.waypoint_client = ActionClient(
            self, WaypointAction, WAYPOINT_ACTION, callback_group=self.callback_group
        )

        self.get_logger().info("Initialized the weeds spray action server...")

    def odom_callback(self, msg: Odometry):
        self.current_position = msg.pose.pose.position

        q = msg.pose.pose.orientation
        self.current_ang = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler(
            "xyz", degrees=False
        )[2]

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info("Starting weeds spraying actions!")

        # obtain the various locations of the weeds
        weed_positions = goal_handle.request.weed_locations.data
        weed_positions = np.array(weed_positions).reshape((len(weed_positions) // 2, 2))

        self.get_logger().info(f"Received weed positions:\n{weed_positions}")

        self.send_feedback(goal_handle, 0)

        total_num_weeds = weed_positions.shape[0]

        for i in range(weed_positions.shape[0]):
            weed_pos = weed_positions[i, :]
            self.get_logger().info(f"Navigating to waypoint at: {weed_pos}")

            # generate the augmented waypoint close to the weed location along
            # a path starting at the current location of the weed
            current_pos = np.array([self.current_position.x, self.current_position.y])
            error_to_target = weed_pos - current_pos
            dist_to_target = np.linalg.norm(error_to_target)

            if dist_to_target < DISTANCE_EPSILON:
                # must move along an arbitrary direction to get within range of the waypoint
                error_to_target_dir = np.array([1.0, 0.0])
            else:
                # can move to a location along the axis joining the current location and weed
                error_to_target_dir = error_to_target / dist_to_target

            dist_to_auxiliarly = dist_to_target - (CHASSIS_WIDTH_M / 2 + WEED_SEPARATION_DIST)
            auxiliary_point = current_pos + dist_to_auxiliarly * error_to_target_dir

            # navigate to this point prior to activating the sprayer
            self.get_logger().info(f"Navigating to auxilliary point: {auxiliary_point}")
            waypoint_goal = WaypointAction.Goal()
            waypoint_goal.target = Point(x=auxiliary_point[0], y=auxiliary_point[1])
            self.waypoint_client.send_goal(waypoint_goal)
            self.get_logger().info("Reached auxiliary point")

            self.get_logger().info("ACTIVATING SPRAYER")

        goal_handle.succeed()

        # sends the final response
        response = WeedsSprayAction.Result()
        response.total_weeds_visited.data = total_num_weeds
        return response

    def send_feedback(self, goal_handle, num_weeds_handled: int):
        feedback_msg = WeedsSprayAction.Feedback()
        feedback_msg.current_weeds_visited.data = num_weeds_handled
        goal_handle.publish_feedback(feedback_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WeedsSprayActionServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
