import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO


PUMP_ACTIVATE_TOPIC = "pump_activate"
PUMP_PIN = 17
# PUMP_PWM_IDX = 0


class PumpDriver(Node):

    def __init__(self):
        super().__init__("pump_driver_node")

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PUMP_PIN, GPIO.OUT)

        self.pump_subscription = self.create_subscription(
            bool, PUMP_ACTIVATE_TOPIC, self.activate_callback, 10
        )
        self.pump_subscription  # prevent unused variable warning

    def activate_callback(self, pump_on):
        if pump_on:
            # turn on GPIO
            GPIO.output(PUMP_PIN, GPIO.HIGH)
            self.get_logger().info("Pump is ON!")
        else:
            # turn off GPIO
            GPIO.output(PUMP_PIN, GPIO.LOW)
            self.get_logger().info("Pump is OFF!")


def main(args=None):
    rclpy.init(args=args)

    pump_driver = PumpDriver()

    rclpy.spin(pump_driver)

    pump_driver.destroy_node()

    GPIO.cleanup()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
