import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO

from rclpy.node import Node
from std_msgs.msg import Float32

LEFT_WHEEL_SPEED_TOPIC = "left_wheel_speed_rpm"
# RIGHT_WHEEL_SPEED_TOPIC = "right_wheel_speed_rpm"

# NOTE: due to lack of PWM signals from the RPi only the left motor is currently used

# pwm signals for speed control
LEFT_WHEEL_FWD_PWM_PIN = 12 # pwm0
LEFT_WHEEL_REV_PWM_PIN = 13 # pwm1

# RIGHT_WHEEL_FWD_PWM_PIN = 45
# RIGHT_WHEEL_FWD_PWM_PIN = 13

WHEEL_PWM_FREQ = 20

# wheel speed parameters
WHEEL_MAX_SPEED_RADPS = 1
WHEEL_PWM_MAX_DUTY = 80  # maximum percentage


# signals for selection of direction input
LEFT_WHEEL_FWD_ENABLE_PIN = 23
LEFT_WHEEL_REV_ENABLE_PIN = 24

# RIGHT_WHEEL_FWD_ENABLE_PIN = 17
# RIGHT_WHEEL_REV_ENABLE_PIN = 18


class OpenLoopMotorDriver(Node):
    def setup_gpio(self):
        # GPIO initialization (if not already completed)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # directional control for each wheel
        GPIO.setup(LEFT_WHEEL_FWD_ENABLE_PIN, GPIO.OUT)
        GPIO.setup(LEFT_WHEEL_REV_ENABLE_PIN, GPIO.OUT)
        # GPIO.setup(RIGHT_WHEEL_FWD_ENABLE_PIN, GPIO.OUT)
        # GPIO.setup(RIGHT_WHEEL_REV_ENABLE_PIN, GPIO.OUT)

        # ensures that all speeds are disabled at startup
        GPIO.output(LEFT_WHEEL_FWD_ENABLE_PIN, GPIO.LOW)
        GPIO.output(LEFT_WHEEL_REV_ENABLE_PIN, GPIO.LOW)
        # GPIO.output(RIGHT_WHEEL_FWD_ENABLE_PIN, GPIO.LOW)
        # GPIO.output(RIGHT_WHEEL_REV_ENABLE_PIN, GPIO.OUT)

        # speed control for each wheel
        self.left_fwd_pwm = GPIO.PWM(LEFT_WHEEL_FWD_PWM_PIN, WHEEL_PWM_FREQ)
        self.left_rev_pwm = GPIO.PWM(LEFT_WHEEL_REV_PWM_PIN, WHEEL_PWM_FREQ)
        # self.right_fwd_pwm = GPIO.PWM(RIGHT_WHEEL_FWD_PWM_PIN, WHEEL_PWM_FREQ)
        # self.right_rev_pwm = GPIO.PWM(RIGHT_WHEEL_REV_ENABLE_PIN, WHEEL_PWM_FREQ)

        self.left_fwd_pwm.start(0)
        self.left_rev_pwm.start(0)
        # self.right_fwd_pwm.start(0)
        # self.right_rev_pwm.start(0)

    def __init__(self):
        super().__init__("open_loop_motor_driver_node")
        self.setup_gpio()

        # callbacks for new speed selection
        self.left_wheel_speed_subscription = self.create_subscription(
            Float32, LEFT_WHEEL_SPEED_TOPIC, self.left_wheel_speed_callback, 10
        )
        # self.right_wheel_speed_subscription = self.create_subscription(
        #     Float32, RIGHT_WHEEL_SPEED_TOPIC, self.right_wheel_speed_callback, 10
        # )

        self.reverse_left_param = self.declare_parameter("reverse_left", False)
        # self.reverse_right_param = self.declare_parameter("reverse_right", False)

    def set_wheel_speed_signals(
        self, speed: float, reverse: bool, fwd_pwm: GPIO.PWM, rev_pwm: GPIO.PWM
    ):
        pwm_duty = WHEEL_PWM_MAX_DUTY / WHEEL_MAX_SPEED_RADPS * speed
        clamped_pwm_duty = min(pwm_duty, WHEEL_PWM_MAX_DUTY)

        if not reverse:
            fwd_pwm.ChangeDutyCycle(clamped_pwm_duty)
            # rev_pwm.ChangeDutyCycle(0)
        else:
            rev_pwm.ChangeDutyCycle(clamped_pwm_duty)
            # fwd_pwm.ChangeDutyCycle(0)

    def left_wheel_speed_callback(self, msg: Float32):
        self.get_logger().info(f"Setting left wheel speed: {msg.data}")
        self.set_wheel_speed_signals(
            msg.data,
            self.reverse_left_param.get_parameter_value(),
            self.left_fwd_pwm,
            self.left_rev_pwm,
        )

    def right_wheel_speed_callback(self, msg: Float32):
        self.get_logger().info(f"Setting right wheel speed: {msg.data}")
        # self.set_wheel_speed_signals(
        #     msg.data,
        #     self.reverse_right_param.get_parameter_value(),
        #     self.right_fwd_pwm,
        #     self.right_rev_pwm,
        # )


def main(args=None):
    rclpy.init(args=args)

    open_loop_motor_driver = OpenLoopMotorDriver()
    rclpy.spin(open_loop_motor_driver)

    open_loop_motor_driver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
