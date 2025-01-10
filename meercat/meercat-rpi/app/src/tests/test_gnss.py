import serial
import time
import pynmea2

import RPi.GPIO as GPIO

GNSS_RESET_PIN = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(GNSS_RESET_PIN, GPIO.OUT)

# so turns out that if ioref is not properly connected then the the UART RX line
# will not get naturally pulled up which stalls communication causing the GPS to
# not enter into communication properlyGPIO

# performs a simple reset
GPIO.output(GNSS_RESET_PIN, GPIO.LOW)
time.sleep(0.10)
GPIO.output(GNSS_RESET_PIN, GPIO.HIGH)

def wrap_nmea_cmd(cmd: str) -> bytearray:
    cmd_bytes = cmd.encode()
    checksum = cmd_bytes[0]
    
    for i in range(1, len(cmd_bytes)):
        checksum ^= cmd_bytes[i]

    wrapped_cmd = f"${cmd}*{checksum:2X}\r\n"
    return wrapped_cmd.encode()

ser = serial.Serial("/dev/serial0", 9600, timeout=1)

while True:
    try:
        line = ser.readline()
        msg = pynmea2.parse(line.decode())
        print(repr(msg))
    except serial.SerialException as e:
        print(f"Device error: {e}")
        break
    except pynmea2.ParseError as e:
        print(f"Parse error: {e}")
    except e:
        print(f"Other error: {e}")
