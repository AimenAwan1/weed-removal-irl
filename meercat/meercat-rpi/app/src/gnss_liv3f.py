import time
import datetime
import serial
import pynmea2
import smbus2

from dataclasses import dataclass
from queue import Queue
from enum import Enum

import RPi._GPIO as GPIO

GNSS_RESET_PIN = 18
GNSS_RESET_DELAY_S = 3

GNSS_I2C_ADDR = 0x3A
GNSS_I2C_RW_REG_ADDR = 0xFF
GNSS_I2C_DEVICE = "/dev/i2c-1"

GNSS_I2C_READ_LEN = 700
GNSS_I2C_READ_DELAY_S = 1

pos_queue = Queue()


@dataclass
class GNSSPosData:
    timestamp: float
    longitude: float
    latitude: float


class GNSSState(Enum):
    GNSS_STATE_STARTUP = 1
    GNSS_STATE_ACTIVE = 2
    GNSS_STATE_ERROR = 3


def read_gnss():
    print("Initializing GNSS...")

    GPIO.setup(GNSS_RESET_PIN, GPIO.OUT)

    while True:
        # triggers a hardware restart of the GNSS module
        GPIO.output(GNSS_RESET_PIN, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(GNSS_RESET_PIN, GPIO.HIGH)

        time.sleep(GNSS_RESET_DELAY_S)

        bus = smbus2.SMBus(GNSS_I2C_DEVICE)

        partial_msg = b""

        try:
            while True:
                msg = smbus2.i2c_msg.read(GNSS_I2C_ADDR, GNSS_I2C_READ_LEN)
                bus.i2c_rdwr(msg)

                # finds any potential match based on NMEA terminating bytes
                partial_msg += bytes(msg)
                newline_splits = partial_msg.split(b"\r\n")

                # final item is possibly the partial match without a \r\n
                partial_msg = newline_splits[-1]

                if len(newline_splits) > 1:
                    possible_matches = [
                        newline_split + b"\r\n" for newline_split in newline_splits
                    ]

                for raw_nmea_msg in possible_matches:
                    try:
                        nmea_msg = pynmea2.parse(raw_nmea_msg.decode())
                        if type(nmea_msg) is pynmea2.GLL:
                            pos_queue.put(
                                GNSSPosData(
                                    datetime.datetime.timestamp,
                                    nmea_msg.longitude,
                                    nmea_msg.latitude,
                                )
                            )
                    except:
                        # failed to decode, ignore
                        pass

                time.sleep(GNSS_I2C_READ_DELAY_S)

        except Exception as e:
            print("Read transaction from GNSS failed, resetting device...")
            bus.close()
