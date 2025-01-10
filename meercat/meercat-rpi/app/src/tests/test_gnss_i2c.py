# for I2C SCL -> close J11
# for I2C SDA -> close J12

# for UART TX (D0) -> open J5
# for UART RX (D1) -> open J2

import time

import RPi.GPIO as GPIO
import smbus2

from colorama import Fore

GNSS_I2C_ADDR = 0x3A
GNSS_I2C_RW_REG_ADDR = 0xFF

GNSS_RESET_PIN = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(GNSS_RESET_PIN, GPIO.OUT)

while True:
    # performs a power cycle, occasional power cycling is necessary
    # as it seems the I2C SCL gets stuck down at one point causing
    # a communication fault

    GPIO.output(GNSS_RESET_PIN, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(GNSS_RESET_PIN, GPIO.HIGH)

    time.sleep(2) # gives enough time to start the GPS

    # exit()

    # reset pin is needed before the liv3f appears on the i2c bus
    # address is 3A

    bus = smbus2.SMBus('/dev/i2c-1')

    try:
        while True:
            msg = smbus2.i2c_msg.read(GNSS_I2C_ADDR, 512)
            bus.i2c_rdwr(msg)

            print(msg)

            time.sleep(0.5)
            
    except Exception as e:
        print(Fore.RED + f"exception: {e}" + Fore.RESET)
        print(Fore.CYAN + "Resetting i2c bus..." + Fore.RESET)
        
        bus.close()

