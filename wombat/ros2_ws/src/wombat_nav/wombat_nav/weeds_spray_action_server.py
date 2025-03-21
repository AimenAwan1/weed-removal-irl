from scipy.spatial.transform import Rotation

import rclpy.callback_groups

from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse

from wombat_msgs.action import WeedsSprayAction
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


WEEDS_SPRAY_ACTION = "weeds_spray_action"


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

    def execute_callback(self, goal_handle):
        self.get_logger().info("Starting weeds spraying actions!")

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def odom_callback(self, msg: Odometry):
        self.current_position = msg.pose.pose.position

        q = msg.pose.pose.orientation
        self.current_ang = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler(
            "xyz", degrees=False
        )[2]
