import rclpy
from rclpy.node import Node

import serial
import time
import pynmea2

from sensor_msgs.msg import NavSatFix

import RPi.GPIO as GPIO

TESEO_RESET_PIN = 23

TESEO_RESET_DURATION_S = 0.1
TESEO_RESET_DELAY_S = 3

TESEO_RATE_HZ = 10
TESEO_READ_LEN = 64

class TeseoLiv3fDriver(Node):
    def __init__(self):
        super().__init__("teseo_liv3f_node")

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TESEO_RESET_PIN, GPIO.OUT)

        # trigger the reset on the liv3f
        GPIO.output(TESEO_RESET_PIN, GPIO.LOW)
        time.sleep(TESEO_RESET_DURATION_S)
        GPIO.output(TESEO_RESET_PIN, GPIO.HIGH)
        time.sleep(TESEO_RESET_DELAY_S)

        self.ser = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=0)
        
        self.partial_msg = b""
        self.create_timer(1.0 / TESEO_RATE_HZ, self.read_teseo_callback)

    def read_teseo_callback(self):
        msg = self.ser.read(TESEO_READ_LEN)
    
        # finds any potential match based on NMEA terminating bytes
        self.partial_msg += msg
        newline_splits = self.partial_msg.split(b"\r\n")

        if not self.partial_msg.endswith(b"\r\n"):
            # final message from split is guaranteed not a valid message
            self.partial_msg = newline_splits[-1]
            possible_matches = [
                (newline_split + b"\r\n")
                for newline_split in newline_splits[0:-1]
            ]
        else:
            # all splits are potentially valid messages (end with \r\n)
            possible_matches = [
                (newline_split + b"\r\n")
                for newline_split in newline_splits
            ]

        # handles parsing of the sensor data
        for raw_nmea_msg in possible_matches:
            try:
                nmea_msg = pynmea2.parse(raw_nmea_msg.decode())
                if type(nmea_msg) is pynmea2.GLL:
                    self.get_logger().info(f"Position data: long={nmea_msg.longitude}, lat={nmea_msg.latitude}")
            except:
                # ignore this failue to decode properly
                pass

def main(args=None):
    rclpy.init(args=args)

    teseo_driver_node = TeseoLiv3fDriver()
    rclpy.spin(teseo_driver_node)

    teseo_driver_node.destroy_node()
    rclpy.shutdown()
