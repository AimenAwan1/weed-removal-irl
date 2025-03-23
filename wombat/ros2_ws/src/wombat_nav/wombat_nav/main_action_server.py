import rclpy.callback_groups

import rclpy.executors
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse

from wombat_msgs.action import MainAction, WaypointAction, WeedsScanAction, WeedsSprayAction
from geometry_msgs.msg import Point

MAIN_ACTION = "main_action"
WAYPOINT_ACTION = "waypoint_action"
WEEDS_SCAN_ACTION = "weeds_scan_action"
WEEDS_SPRAY_ACTION = "weeds_spray_action"

MAIN_WAYPOINTS = [
    # update this with the main waypoints we want to reach
    (1.0, 0.0)
]

class MainActionServer(Node):
    def __init__(self):
        super().__init__("main_action_server")
        self.global_count = 0
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            MainAction,
            self.execute_callback,
            callback_group=self.callback_group,
            cancel_callback=self.cancel_callback,
        )

        self.waypoint_client = ActionClient(
            self, WaypointAction, WAYPOINT_ACTION, callback_group=self.callback_group
        )
        self.weeds_scan_client = ActionClient(
            self, WeedsScanAction, WEEDS_SCAN_ACTION, callback_group=self.callback_group
        )
        self.weeds_spray_client = ActionClient(
            self, WeedsSprayAction, WEEDS_SPRAY_ACTION, callback_group=self.callback_group
        )

        self.get_logger().info("Initialized the main action server")

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Re3ceived cancel request")
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        self.get_logger().info("Starting main actions!")

        # note that there is no feedback for this action (as feedback was 
        # mainly being used for debugging purposes)

        for x, y in MAIN_WAYPOINTS:
            self.get_logger().info(f"Navigating to main waypoint: x={x},  y={y}")

            waypoint_goal = WaypointAction.Goal()
            waypoint_goal.target = Point(x=x, y=y)
            self.waypoint_client.send_goal(waypoint_goal)

            self.get_logger().info(f"Finished navigating to main waypoint, starting scan")

            scan_goal = WeedsScanAction.Goal()
            scan_result: WeedsScanAction.Result = self.weeds_scan_client.send_goal(scan_goal)
            
            self.get_logger().info(f"Finished scan at main waypoint, terminating weeds")

            spray_goal = WeedsSprayAction.Goal()
            spray_goal.weed_locations.data = scan_result.detections.data
            self.weeds_spray_client.send_goal(spray_goal)

            self.get_logger().info("Finished removing weeds around waypoint, moving to next waypoint")

        self.get_logger().info("Finished handling all main waypoints!!")

        goal_handle.success()
        result = MainAction.Result()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MainActionServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
