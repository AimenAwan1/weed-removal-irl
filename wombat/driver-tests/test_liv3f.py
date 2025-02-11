import RPi.GPIO as GPIO

import serial
import time
import pynmea2

GPS_RESET_PIN = 23
GPS_RESET_DELAY_S = 3

NMEA_GPGGA_BITMASK = 0x2
NMEA_GPGST_BITMASK = 0x8
NMEA_GPGLL_BITMASK = 0x100000

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

    # use the PSTMNMEAREQUEST command (pg. 68) to configure only the global fix
    # location and the retrieval of covariance messages (see byte content
    # description starting on pg. 190)

    # need GLL and GST

    low_byte = NMEA_GPGGA_BITMASK | NMEA_GPGST_BITMASK | NMEA_GPGLL_BITMASK
    high_byte = 0x00

    msg = pynmea2.nmea.ProprietarySentence(
        manufacturer="STM",
        data=["NMEAREQUEST", f"{low_byte:08X}", f"{high_byte:08X}"]
    )
    print(msg.render().encode())
    
    ser.write((msg.render()+"\r\n").encode())

    while True:
        data: bytes = ser.read(32)
        print(data.decode('UTF-8'), end='')

    ser.close()


if __name__ == "__main__":
    main()
