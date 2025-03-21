import rclpy.callback_groups
import numpy as np

from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.node import Node

from scipy.spatial.transform import Rotation

from nav_msgs.msg import Odometry

from std_msgs.msg import Float64MultiArray

from wombat_msgs.action import WeedsScanAction, TurnAction
from wombat_msgs.srv import DetectObjects

WEEDS_SCAN_ACTION = "weeds_scan_action"
TURN_ACTION = "turn_action"
DETECT_OBJECTS_SERVICE = "detect_objects"

ODOMETRY_TOPIC = "/robot_position"


class WeedsScanActionServer(Node):
    def __init__(self):
        super().__init__("weeds_scan_action_server")
        self.global_count_ = 0
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.current_ang = 0

        self._action_server = ActionServer(
            self,
            WeedsScanAction,
            WEEDS_SCAN_ACTION,
            self.execute_callback,
            callback_group=self.callback_group,
            cancel_callback=self.cancel_callback
        )

        self.odom_subscription = self.create_subscription(
            Odometry, ODOMETRY_TOPIC, self.odom_callback, 10,
            callback_group=self.callback_group
        )

        self.turn_action_client = ActionClient(
            self, TurnAction, TURN_ACTION, callback_group=self.callback_group)
        self.detect_client = self.create_client(
            DetectObjects, DETECT_OBJECTS_SERVICE, callback_group=self.callback_group)

        # while not self.detect_client.wait_for_service(timeout_sec=10):
        #     self.get_logger().warn(
        #         f"object detection service unavailable: {DETECT_OBJECTS_SERVICE}")

        self.get_logger().info("weeds scanning action server online")

    def odom_callback(self, msg: Odometry):
        self.get_logger().info("Received odom callback")
        q = msg.pose.pose.orientation
        self.current_ang = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler(
            "xyz", degrees=False
        )[2]

    def execute_callback(self, goal_handle):
        self.get_logger().info("Starting identification of weed locations")

        # first of the scans
        response = self.detect_client.call(DetectObjects.Request())
        self.get_logger().info("finished first service call !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        detections = response.detections.data.tolist()
        self.get_logger().info(f"first {detections}")

        # sends first feedback
        # feedback_msg.current_detections.data = detections
        self.get_logger().info(f"data: {detections}")
        self.get_logger().info("appended data")
        self.send_feedback(goal_handle, detections)
        self.get_logger().info("send feedback!")

        # rotate then scan
        for i in range(3):
            rotate_goal = TurnAction.Goal()
            rotate_goal.target.data = self.get_next_angle(self.current_ang)
            self.turn_action_client.send_goal(rotate_goal)

            response = self.detect_client.call(DetectObjects.Request())
            detections.extend(response.detections.data.tolist())
            print(f"More detections: {response.detections.data.tolist()}")
            self.send_feedback(goal_handle, detections)

        response = WeedsScanAction.Response()
        response.detections = detections
        return response
    
    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def send_feedback(self, goal_handle, detections):
        feedback = WeedsScanAction.Feedback()
        feedback.current_detections.data = detections
        goal_handle.publish_feedback(feedback)

    def get_next_angle(self, ang):
        next_ang = ang + np.pi/2
        if next_ang > np.pi:
            # ensures angle is on (pi,-pi)
            next_ang = -(2*np.pi - next_ang)
        return next_ang


def main(args=None):
    rclpy.init(args=args)
    node = WeedsScanActionServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
