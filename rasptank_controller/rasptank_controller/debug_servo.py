#!/usr/bin/env/python3
'''
 SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
# sudo pip3 install adafruit-circuitpython-motor
# sudo pip3 install adafruit-circuitpython-pca9685
'''
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)
# Create a simple PCA9685 class instance.
pca = PCA9685(i2c, address=0x5f) #default 0x40

pca.frequency = 50

# servo7 = servo.Servo(pca.channels[7], min_pulse=580, max_pulse=2350)
# servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2600)
# servo7 = servo.Servo(pca.channels[7], min_pulse=400, max_pulse=2400)
# servo7 = servo.Servo(pca.channels[7], min_pulse=600, max_pulse=2500)
# servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2400)
# The pulse range is 750 - 2250 by default. This range typically gives 135 degrees of
# range, but the default is to use 180 degrees. You can specify the expected range if you wish:
# servo7 = servo.Servo(pca.channels[7], actuation_range=135)
def set_angle(ID, angle):
    servo_angle = servo.Servo(pca.channels[ID], min_pulse=500, max_pulse=2400,actuation_range=180)
    servo_angle.angle = angle

def get_angle(ID):
    servo_angle = servo.Servo(pca.channels[ID], min_pulse=500, max_pulse=2400,actuation_range=180)
    print(servo_angle.angle)


if __name__ == "__main__":
    channel = 0
    angle = 90
    get_angle(channel)
    set_angle(channel,angle)

'''

'''

#servo7.angle = 90