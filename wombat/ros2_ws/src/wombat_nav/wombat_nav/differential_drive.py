import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

CHASSIS_SPEED_TOPIC = "chassis_speed"
CHASSIS_ANG_VEL_TOPIC = "chassis_ang_vel"

LEFT_WHEEL_SPEED_TOPIC = "left_wheel_speed_radps"
RIGHT_WHEEL_SPEED_TOPIC = "right_wheel_speed_radps"

CHASSIS_WIDTH_M = 0.4508


class DifferentialDrive(Node):
    def __init__(self):
        super().__init__("differential_driver_node")

        self.chassis_speed_subscription = self.create_subscription(
            Float32, CHASSIS_SPEED_TOPIC, self.chassis_speed_callback, 10
        )
        self.chassis_ang_vel_subscription = self.create_subscription(
            Float32, CHASSIS_ANG_VEL_TOPIC, self.chassis_ang_vel_callback, 10
        )

        self.left_wheel_speed_publisher = self.create_publisher(
            Float32, LEFT_WHEEL_SPEED_TOPIC, 10
        )
        self.right_wheel_speed_publisher = self.create_publisher(
            Float32, RIGHT_WHEEL_SPEED_TOPIC, 10
        )

        self.chassis_speed = 0
        self.chassis_ang_vel = 0

    def chassis_speed_callback(self, msg: Float32):
        self.get_logger().info(f"Speed received: {msg.data}")
        self.chassis_speed = msg.data

    def chassis_ang_vel_callback(self, msg: Float32):
        self.get_logger().info(f"Angular velocity received: {msg.data}")
        self.chassis_ang_vel = msg.data

    def publish_wheel_speeds(self):
        right_wheel_speed_radps = (
            self.chassis_speed + CHASSIS_WIDTH_M * self.chassis_ang_vel / 2
        )
        left_wheel_speed_radps = (
            self.chassis_speed - CHASSIS_WIDTH_M * self.chassis_ang_vel / 2
        )

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