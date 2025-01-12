import rclpy
from rclpy.node import Node

from wombat_msgs.msg import GNSSPos, GNSSPosStamped, IMUStamped
from wombat_msgs.srv import YardScan


class MeercatIFNode(Node):
    def __init__(self):
        super().__init__("meercat_if_node")

        self.imu_publisher = self.create_publisher(IMUStamped, "meercat_imu", 10)
        self.gnss_publisher = self.create_publisher(GNSSPosStamped, "meercat_gnss", 10)

        self.yard_scan_service = self.create_service(
            YardScan, "yard_scan", self.yard_scan_callback
        )

    def yard_scan_callback(
        self, request: YardScan.Request, response: YardScan.Response
    ) -> YardScan.Response:
        self.get_logger().info(f"Incoming request for yard scan id: {request.yard_id}")
        return response


def main(args=None):
    rclpy.init(args=args)

    meercat_if_node = MeercatIFNode()
    rclpy.spin(meercat_if_node)

    meercat_if_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
