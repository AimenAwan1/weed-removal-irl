import RPi.GPIO as GPIO

import serial
import time
import pynmea2

GPS_RESET_PIN = 23
GPS_RESET_DELAY_S = 3


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPS_RESET_PIN, GPIO.OUT)

    # trigger the reset on the liv3f
    GPIO.output(GPS_RESET_PIN, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(GPS_RESET_PIN, GPIO.HIGH)

    time.sleep(GPS_RESET_DELAY_S)

    ser = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=1)

    # setup configuration to send desired messages



    while True:
        data: bytes = ser.read(32)
        print(data.decode('UTF-8'), end='')

    ser.close()


if __name__ == "__main__":
    main()
