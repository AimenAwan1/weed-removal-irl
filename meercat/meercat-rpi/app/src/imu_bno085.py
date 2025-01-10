import os
import time
import datetime

from dataclasses import dataclass
from queue import Queue

from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x.i2c import BNO08X_I2C

from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR,
)

IMU_UPDATE_RATE_HZ = 10
IMU_INIT_DELAY_S = 1

imu_data_queue = Queue()


@dataclass
class AccelData:
    accel_x: float
    accel_y: float
    accel_z: float


@dataclass
class AngVelData:
    ang_vel_x: float
    ang_vel_y: float
    ang_vel_z: float


@dataclass
class RotData:
    quat_i: float
    quat_j: float
    quat_k: float
    quat_real: float


@dataclass
class IMUData:
    timestamp: float
    accel: AccelData
    ang_vel: AngVelData
    rot: RotData


def read_imu_bno085():
    print("Initializing IMU...")

    # NOTE: a logic analyzer must NOT be connected to the lines of i2c-3 as it
    # causes failures in reading or scanning for devices - why this occurs is
    # still unknown at this time
    
    i2c = I2C(3)

    # Sets up the accelerometer and rotation vector
    bno = BNO08X_I2C(i2c)
    bno.hard_reset()

    time.sleep(2)

    bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

    time.sleep(IMU_INIT_DELAY_S)

    imu_period_s = 1.0 / IMU_UPDATE_RATE_HZ

    while True:
        try:
            # handled individually so all data ceases if one fails
            accel_x, accel_y, accel_z = bno.acceleration
            ang_vel_x, ang_vel_y, ang_vel_z = bno.gyro
            quat_i, quat_j, quat_k, quat_real = bno.quaternion

            imu_data_queue.put(
                IMUData(
                    datetime.datetime.timestamp,
                    AccelData(accel_x, accel_y, accel_z),
                    AngVelData(ang_vel_x, ang_vel_y, ang_vel_z),
                    RotData(quat_i, quat_j, quat_k, quat_real),
                )
            )

            time.sleep(imu_period_s)

        except (OSError, ValueError, KeyError) as e:
            print(f"Error reading sensor data: {e}")
            time.sleep(imu_period_s)
