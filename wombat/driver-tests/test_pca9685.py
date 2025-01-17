import board
from adafruit_pca9685 import PCA9685

i2c = board.I2C()

pca = PCA9685(i2c)

pca.frequency = 60
pca.channels[0].duty_cycle = 0x7FFF
