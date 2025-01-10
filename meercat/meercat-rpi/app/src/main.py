import threading
import time
import pynmea2

from typing import Optional
from enum import Enum

from serial import Serial

from RPLCD.i2c import CharLCD

import RPi._GPIO as GPIO

import imu_bno085
import gnss_liv3f

import proto.sensor_data.gnss_data_pb2 as gnss_data_pb2
import proto.sensor_data.imu_data_pb2 as imu_data_pb2
import proto.wombat_interchange.sensor_update_msg_pb2 as sensor_update_msg_pb2


from typing import List

LCD_I2C_ADDR = 0x27
LCD_I2C_PORT = 1

LCD_COLUMNS = 20
LCD_ROWS = 4
LCD_DOTSIZE = 8

MAPPING_START_STOP_PIN = 4
WOMBAT_CONN_PIN = 25

SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 1


def to_time_ms(t: float) -> int:
    return int(t * 1000.0)

def encode_imu_data_to_proto(data: imu_bno085.IMUData):
    proto_data = imu_data_pb2.IMU_Data()
    proto_data.timestamp_ms = 0 # to_time_ms(data.timestamp)

    proto_data.linear_accel.accel_x = data.accel.accel_x
    proto_data.linear_accel.accel_y = data.accel.accel_y
    proto_data.linear_accel.accel_z = data.accel.accel_z

    proto_data.ang_vel.ang_vel_x = data.ang_vel.ang_vel_x
    proto_data.ang_vel.ang_vel_y = data.ang_vel.ang_vel_y
    proto_data.ang_vel.ang_vel_z = data.ang_vel.ang_vel_z

    proto_data.rot_vec.quat_i = data.rot.quat_i
    proto_data.rot_vec.quat_j = data.rot.quat_j
    proto_data.rot_vec.quat_k = data.rot.quat_k
    proto_data.rot_vec.quat_real = data.rot.quat_real

    return proto_data

def encode_gnss_data_to_proto(data: gnss_liv3f.GNSSPosData):
    proto_data = gnss_data_pb2.GNSS_Data()
    proto_data.timestamp_ms = 0 # to_time_ms(data.timestamp)

    proto_data.longitude_deg = data.longitude
    proto_data.latitude_deg = data.latitude

    return proto_data



class MeercatState(Enum):
    STATE_STARTUP = 1
    STATE_MAPPING_WAITING = 2
    STATE_MAPPING_ACTIVE = 3
    STATE_SENSOR_OUTPUT = 4


def state_mapping_waiting(state: MeercatState):
    raise NotImplementedError


def state_mapping_active(
    state: MeercatState,
    lcd: CharLCD,
    serial: Serial,
    gnss_reads: List[gnss_liv3f.GNSSPosData],
    imu_reads: List[imu_bno085.IMUData],
):
    if len(imu_reads) > 0:
        latest_imu = imu_reads[-1]
        print(f"latest imu: {latest_imu}")

    if len(gnss_reads) > 0:
        latest_gnss = gnss_reads[-1]
        print(f"latest gnss: {latest_gnss}")


def state_sensor_output_active(
    state: MeercatState,
    lcd: CharLCD,
    serial: Serial,
    gnss_reads: List[gnss_liv3f.GNSSPosData],
    imu_reads: List[imu_bno085.IMUData],
    sequence_num: int
):
    latest_imu = None
    latest_gnss = None

    if len(imu_reads) > 0:
        latest_imu = imu_reads[-1]
        # print(f"latest imu: {latest_imu}")

    if len(gnss_reads) > 0:
        latest_gnss = gnss_reads[-1]
        # print(f"latest gnss: {latest_gnss}")

    # at least one of the sensor values has to exist
    if latest_imu is not None \
            or latest_gnss is not None:
        sensor_update_msg = sensor_update_msg_pb2.Sensor_Update_Msg()
        sensor_update_msg.sequence_num = sequence_num

        t = to_time_ms(time.time())
        print(f'Le time: {t}')

        sensor_update_msg.timestamp_ms = 0 # t
        
        if latest_gnss is not None:
            sensor_update_msg.gnss_data = encode_gnss_data_to_proto(latest_gnss)

        if latest_imu is not None:
            sensor_update_msg.imu_data = encode_imu_data_to_proto(latest_imu)

        sequence_num = sequence_num + 1

        print(sensor_update_msg)


def main():
    print("Initializing...")

    # performs initialization of external connections and
    lcd = CharLCD(
        i2c_expander="PCF8574",
        address=LCD_I2C_ADDR,
        port=LCD_I2C_PORT,
        cols=LCD_COLUMNS,
        rows=LCD_ROWS,
        dotsize=LCD_DOTSIZE,
    )
    lcd.write_string("Initializing...")
    lcd.backlight_enabled = True

    time.sleep(1)

    serial = Serial(SERIAL_PORT, baudrate=SERIAL_BAUD, timeout=SERIAL_TIMEOUT)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(WOMBAT_CONN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    gnss_thread = threading.Thread(target=gnss_liv3f.read_gnss, daemon=True)
    imu_thread = threading.Thread(target=imu_bno085.read_imu_bno085, daemon=True)

    gnss_thread.start()
    imu_thread.start()

    sensor_output_sequence_num = 0

    # temporarily
    state = MeercatState.STATE_SENSOR_OUTPUT

    while True:
        # gets the most current sensor data

        gnss_reads: List[gnss_liv3f.GNSSPosData] = []
        while not gnss_liv3f.pos_queue.empty():
            gnss_reads.append(gnss_liv3f.pos_queue.get())
            gnss_liv3f.pos_queue.task_done()

        imu_reads: List[imu_bno085.IMUData] = []
        while not imu_bno085.imu_data_queue.empty():
            imu_reads.append(imu_bno085.imu_data_queue.get())
            imu_bno085.imu_data_queue.task_done()

        # state machine actions and transitions

        match state:
            case MeercatState.STATE_STARTUP:
                exit(1)
            case MeercatState.STATE_MAPPING_WAITING:
                exit(1)
            case MeercatState.STATE_MAPPING_ACTIVE:
                exit(1)
            case MeercatState.STATE_SENSOR_OUTPUT:
                state_sensor_output_active(state, lcd, serial, gnss_reads, imu_reads, sensor_output_sequence_num)

        time.sleep(0.1)  # 10 Hz update rate


if __name__ == "__main__":
    main()
