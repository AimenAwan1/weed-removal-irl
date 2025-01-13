import RPi.GPIO as GPIO
import math

class MotorDriver(object):

    def __init__(self, base_pwm=50):
        self.PWMA1 = 6
        self.PWMA2 = 13
        self.PWMB1 = 20
        self.PWMB2 = 21
        self.D1 = 12
        self.D2 = 26

        self.PWM1 = 0
        self.PWM2 = 0
        self.BASE_PWM = base_pwm
        self.MAX_PWM = 100

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.PWMA1, GPIO.OUT)
        GPIO.setup(self.PWMA2, GPIO.OUT)
        GPIO.setup(self.PWMB1, GPIO.OUT)
        GPIO.setup(self.PWMB2, GPIO.OUT)
        GPIO.setup(self.D1, GPIO.OUT)
        GPIO.setup(self.D2, GPIO.OUT)

        self.p1 = GPIO.PWM(self.D1, 500)
        self.p2 = GPIO.PWM(self.D2, 500)
        self.p1.start(self.PWM1)
        self.p2.start(self.PWM2)

    def set_speed(self, left_velocity, right_velocity):
        left_rpm = left_velocity * 60 / ( 2 * math.pi)
        right_rpm = right_velocity * 60 / ( 2 * math.pi)

        self.PWM1 = min(int(left_rpm / 170 * self.BASE_PWM), self.MAX_PWM)
        self.PWM2 = min(int(right_rpm / 170 * self.BASE_PWM), self.MAX_PWM)
        
        self.p1.ChangeDutyCycle(self.PWM1)
        self.p2.ChangeDutyCycle(self.PWM2)

    def stop(self):
        self.p1.ChangeDutyCycle(0)
        self.p2.ChangeDutyCycle(0)