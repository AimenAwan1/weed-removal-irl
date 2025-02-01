import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

import busio

from adafruit_extended_bus import ExtendedI2C
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR,
)

BNO_IMU_RATE_HZ = 10
BNO_IMU_DELAY_S = 1

BNO_I2C_BUS = 1

IMU_TOPIC = "imu"


class BNO085Driver(Node):
    def __init__(self):
        super().__init__("imu_driver_node")

        i2c = ExtendedI2C(BNO_I2C_BUS)
        self.bno = BNO08X_I2C(i2c)

        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        self.create_timer(1.0 / BNO_IMU_RATE_HZ, self.read_imu_callback)
        self.imu_pub = self.create_publisher(Imu, IMU_TOPIC, 10)

    def read_imu_callback(self):
        try:
            # only send the full imu message if all values can be read correctly

            msg = Imu()

            linear_accel = self.bno.linear_acceleration
            self.get_logger().info(f"linear accel: {linear_accel}")
            if linear_accel is not None:
                msg.linear_acceleration.x = linear_accel[0]
                msg.linear_acceleration.y = linear_accel[1]
                msg.linear_acceleration.z = linear_accel[2]
            else:
                raise

            angular_vel = self.bno.gyro
            self.get_logger().info(f"angular vel: {angular_vel}")
            if angular_vel is not None:
                msg.angular_velocity.x = angular_vel[0]
                msg.angular_velocity.y = angular_vel[1]
                msg.angular_velocity.z = angular_vel[2]
            else:
                raise

            orient = self.bno.quaternion
            self.get_logger().info(f"orientation: {orient}")
            if orient is not None:
                msg.orientation.x = orient[0]
                msg.orientation.y = orient[1]
                msg.orientation.z = orient[2]
                msg.orientation.w = orient[3]
            else:
                raise

            self.imu_pub.publish(msg)
        except:
            # sends a blank message to prevent errors
            msg = Imu()
            self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    bno085_driver_node = BNO085Driver()
    rclpy.spin(bno085_driver_node)

    bno085_driver_node.destroy_node()
    rclpy.shutdown()
