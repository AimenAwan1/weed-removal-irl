import rclpy
from rclpy.node import Node

import board
from adafruit_pca9685 import PCA9685

from RPi import GPIO

from rclpy.node import Node
from std_msgs.msg import Float32

# left/right wheel driver

LEFT_WHEEL_SPEED_TOPIC = "left_wheel_speed_rpm"
LEFT_FWD_PWM_PCA_IDX = 0
LEFT_REV_PWM_PCA_IDX = 1

RIGHT_WHEEL_SPEED_TOPIC = "right_wheel_speed_rpm"
RIGHT_FWD_PWM_PCA_IDX = 2
RIGHT_REV_PWM_PCA_IDX = 3

DEFAULT_PCA_FREQ = 1000  # Hz

WHEEL_SPEED_MAX_PWM_DUTY = 0xCCCC  # 80%
WHEEL_SPEED_MAX_RADPS = 1.5
WHEEL_SPEED_DEADZONE_RADPS = 0.1


class OpenLoopMotorDriver(Node):
    def setup_pca9685(self):
        i2c = board.I2C()
        self.pca = PCA9685(i2c)

        self.pca.reset()
        self.pca.frequency = DEFAULT_PCA_FREQ

        # ensures that all motor commands are 0 at startup
        self.pca.channels[LEFT_FWD_PWM_PCA_IDX].duty_cycle = 0
        self.pca.channels[LEFT_REV_PWM_PCA_IDX].duty_cycle = 0

        self.pca.channels[RIGHT_FWD_PWM_PCA_IDX].duty_cycle = 0
        self.pca.channels[RIGHT_REV_PWM_PCA_IDX].duty_cycle = 0

    def __init__(self):
        super().__init__("open_loop_motor_driver_node")

        self.pca_freq_param = self.declare_parameter("pca_freq", DEFAULT_PCA_FREQ)
        self.reverse_left_param = self.declare_parameter("reverse_left", False)
        self.reverse_right_param = self.declare_parameter("reverse_right", False)

        self.left_wheel_speed_subscription = self.create_subscription(
            Float32, LEFT_WHEEL_SPEED_TOPIC, self.left_wheel_speed_callback, 10
        )
        self.right_wheel_speed_subscription = self.create_subscription(
            Float32, RIGHT_WHEEL_SPEED_TOPIC, self.right_wheel_speed_callback, 10
        )

        self.setup_pca9685()

    def set_wheel_speed_signals(
        self,
        speed_rpm: float,
        reverse: bool,
        fwd_pwm_idx: int,
        rev_pwm_idx: int,
    ):
        if abs(speed_rpm) <= WHEEL_SPEED_DEADZONE_RADPS:
            # disables both PWM inputs when in the desert
            self.pca.channels[fwd_pwm_idx].duty_cycle = 0x0000
            self.pca.channels[rev_pwm_idx].duty_cycle = 0x0000
        else:
            pwm_duty = WHEEL_SPEED_MAX_PWM_DUTY / WHEEL_SPEED_MAX_RADPS * abs(speed_rpm)
            clamped_pwm_duty = int(min(pwm_duty, WHEEL_SPEED_MAX_PWM_DUTY))

            if speed_rpm > 0 and not reverse:
                self.get_logger().info(f"Forward set: speed={speed_rpm}, reverse={reverse}")
                # forward wheel movement is requested
                self.pca.channels[rev_pwm_idx].duty_cycle = 0x0000
                self.pca.channels[fwd_pwm_idx].duty_cycle = clamped_pwm_duty
            else:
                self.get_logger().info(f"Reverse set: speed={speed_rpm}, reverse={reverse}")
                # reverse wheel movement is requested
                self.pca.channels[fwd_pwm_idx].duty_cycle = 0x0000
                self.pca.channels[rev_pwm_idx].duty_cycle = clamped_pwm_duty

    def left_wheel_speed_callback(self, msg: Float32):
        self.get_logger().info(f"Setting left wheel speed: {msg.data}")

        self.set_wheel_speed_signals(
            msg.data,
            self.reverse_left_param.get_parameter_value().bool_value,
            LEFT_FWD_PWM_PCA_IDX,
            LEFT_REV_PWM_PCA_IDX,
        )

    def right_wheel_speed_callback(self, msg: Float32):
        self.get_logger().info(f"Setting right wheel speed: {msg.data}")

        self.set_wheel_speed_signals(
            msg.data,
            self.reverse_right_param.get_parameter_value().bool_value,
            RIGHT_FWD_PWM_PCA_IDX,
            RIGHT_REV_PWM_PCA_IDX,
        )


def main(args=None):
    rclpy.init(args=args)

    open_loop_motor_driver = OpenLoopMotorDriver()
    rclpy.spin(open_loop_motor_driver)

    open_loop_motor_driver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
