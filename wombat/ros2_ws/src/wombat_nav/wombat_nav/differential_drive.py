import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from geometry_msgs.msg import Twist

CHASSIS_VEL_TOPIC = "chassis_vel"

LEFT_WHEEL_SPEED_TOPIC = "cmd_left_wheel_vel_radps"
RIGHT_WHEEL_SPEED_TOPIC = "cmd_right_wheel_vel_radps"

CHASSIS_WIDTH_M = 0.4508
WHEEL_RADIUS_M = 0.245/2


class DifferentialDrive(Node):
    def __init__(self):
        super().__init__("differential_driver_node")

        self.chassis_vel = self.create_subscription(
            Twist, CHASSIS_VEL_TOPIC, self.chassis_vel_callback, 10
        )

        self.left_wheel_speed_publisher = self.create_publisher(
            Float32, LEFT_WHEEL_SPEED_TOPIC, 10
        )
        self.right_wheel_speed_publisher = self.create_publisher(
            Float32, RIGHT_WHEEL_SPEED_TOPIC, 10
        )

    def chassis_vel_callback(self, msg: Twist):
        # self.get_logger().info(
        #     f"Chassis speed command received: v={msg.linear.x}, w={msg.angular.z}"
        # )
        chassis_speed = msg.linear.x
        chassis_ang_vel = msg.angular.z

        right_wheel_speed_radps = 1/WHEEL_RADIUS_M*(chassis_speed + CHASSIS_WIDTH_M * chassis_ang_vel / 2)
        left_wheel_speed_radps = 1/WHEEL_RADIUS_M*(chassis_speed - CHASSIS_WIDTH_M * chassis_ang_vel / 2)

        self.left_wheel_speed_publisher.publish(Float32(data=left_wheel_speed_radps))
        self.right_wheel_speed_publisher.publish(Float32(data=right_wheel_speed_radps))


def main(args=None):
    rclpy.init(args=args)

    differential_drive = DifferentialDrive()
    rclpy.spin(differential_drive)

    differential_drive.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
