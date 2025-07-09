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
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import time




class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.change = 5.0
        self.servo_angle = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2400,actuation_range=180)
        self.servo_angle.angle = 0
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        self.br = CvBridge()
    
    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        face_classifier = cv2.CascadeClassifier(
        cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    )
        vid = self.br.imgmsg_to_cv2(data)
        vid = cv2.cvtColor(vid, cv2.COLOR_BGR2RGB)
        gray_image = cv2.cvtColor(vid, cv2.COLOR_BGR2GRAY)
        faces = face_classifier.detectMultiScale(gray_image, 1.1, 5, minSize=(40, 40))
        W, H = vid.shape[1],vid.shape[0]
        for (x, y, w, h) in faces:
            x0 = x + w/2
            y0 = y + h/2
            if y0> H/2 and self.servo_angle.angle<180.0 - self.change:
                self.servo_angle.angle = self.servo_angle.angle + self.change
            else :
                if self.servo_angle.angle>self.change:
                    self.servo_angle.angle = self.servo_angle.angle - self.change
            cv2.rectangle(vid, (x, y), (x + w, y + h), (0, 255, 0), 4)
        

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


