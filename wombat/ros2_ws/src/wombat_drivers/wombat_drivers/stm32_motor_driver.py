import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from wombat_msgs.msg import EstWheelsVel

from enum import Enum
from dataclasses import dataclass

from smbus2 import SMBus

import time
import struct

CMD_LEFT_WHEEL_VEL_TOPIC = "cmd_left_wheel_vel_radps"
CMD_RIGHT_WHEEL_VEL_TOPIC = "cmd_right_wheel_vel_radps"
CMD_TIMEOUT_CHECK_FREQ = 10
CMD_TIMEOUT_S = 5

CURR_EST_WHEELS_VELS_TOPIC = "curr_est_wheel_vels"
CHECK_CURR_WHEEL_VELS_FREQ = 10

STM32_I2C_BUS = 1
STM32_I2C_ADDR = 0x45


@dataclass
class WheelVelCmd:
    timestamp: float
    cmd: float
    timed_out: bool = False

    def get_cmd(self) -> float:
        if not self.timed_out:
            return self.cmd
        else:
            return 0.0

    def check_time_out(self, curr_time: float) -> bool:
        if not self.timed_out and (curr_time - self.timestamp) < CMD_TIMEOUT_S:
            self.timed_out = True
            return True  # only return True when it times out
        else:
            return False


class CmdId(Enum):
    CMD_READ_ENCODERS = 0
    CMD_SET_SPEEDS = 1


class STM32MotorDriver(Node):
    def __init__(self):
        super().__init__("stm32_motor_driver")

        self.bus = SMBus(STM32_I2C_BUS)

        self.left_wheel_vel_cmd = WheelVelCmd(0, 0)
        self.right_wheel_vel_cmd = WheelVelCmd(0, 0)

        self.create_timer(
            1.0 / CHECK_CURR_WHEEL_VELS_FREQ,
            self.check_curr_wheels_vels_callback
        )

        self.est_wheel_vels_pub = self.create_publisher(
            EstWheelsVel, CURR_EST_WHEELS_VELS_TOPIC, 10)

        self.create_timer(
            1.0 / CMD_TIMEOUT_CHECK_FREQ,
            self.cmd_timeout_check_callback)

        self.cmd_left_wheel_vel_subscription = self.create_subscription(
            Float32,
            CMD_LEFT_WHEEL_VEL_TOPIC,
            self.cmd_left_wheel_vel_callback,
            10
        )

        self.cmd_right_wheel_vel_subscription = self.create_subscription(
            Float32,
            CMD_RIGHT_WHEEL_VEL_TOPIC,
            self.cmd_right_wheel_vel_callback,
            10
        )

    def __del__(self):
        self.left_wheel_vel_cmd = WheelVelCmd(0, 0)
        self.right_wheel_vel_cmd = WheelVelCmd(0, 0)
        self.send_wheel_vel_cmd()

    def check_curr_wheels_vels_callback(self):
        data = self.bus.read_i2c_block_data(
            STM32_I2C_ADDR,
            CmdId.CMD_READ_ENCODERS.value,
            4*4) # 4 float32
        # self.get_logger().info(str(data))
        msg = EstWheelsVel()
        (msg.left_wheel_vel,
         msg.left_wheel_var,
         msg.right_wheel_vel,
         msg.right_wheel_var) = struct.unpack("ffff", bytes(data))

        self.est_wheel_vels_pub.publish(msg)

    def cmd_timeout_check_callback(self):
        curr_time = time.time()
        # if (self.left_wheel_vel_cmd.check_time_out(curr_time) or
        #         self.right_wheel_vel_cmd.check_time_out(curr_time)):
        #     self.send_wheel_vel_cmd()

    def cmd_left_wheel_vel_callback(self, msg: Float32):
        self.left_wheel_vel_cmd = WheelVelCmd(time.time(), msg.data)
        self.send_wheel_vel_cmd()

    def cmd_right_wheel_vel_callback(self, msg: Float32):
        self.right_wheel_vel_cmd = WheelVelCmd(time.time(), msg.data)
        self.send_wheel_vel_cmd()

    def send_wheel_vel_cmd(self):
        data = struct.pack(
            "ff",
            self.left_wheel_vel_cmd.get_cmd(),
            self.right_wheel_vel_cmd.get_cmd())
        self.bus.write_i2c_block_data(
            STM32_I2C_ADDR,
            CmdId.CMD_SET_SPEEDS.value,
            data)


def main(args=None):
    rclpy.init(args=args)

    stm32_motor_driver_node = STM32MotorDriver()
    rclpy.spin(stm32_motor_driver_node)

    stm32_motor_driver_node.destroy_node()
    rclpy.shutdown()
