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

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
import numpy as np
import cv2
import time

# Try to import cv_bridge with error handling
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError as e:
    print(f"cv_bridge import failed: {e}")
    CV_BRIDGE_AVAILABLE = False




class CameraController(Node):
    def __init__(self):
        super().__init__('camera_controller')
        self.change = 3.0
        self.bounds = [95.0,85.0]
        self.servo_angle = servo.Servo(pca.channels[4], min_pulse=500, max_pulse=2400,actuation_range=180)
        self.servo_angle.angle = 90.0
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.start_time = time.time()
        
        # Initialize cv_bridge if available
        if CV_BRIDGE_AVAILABLE:
            self.br = CvBridge()
        else:
            self.br = None
            self.get_logger().warn("cv_bridge not available - face detection disabled")

    
    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        
        # Skip processing if cv_bridge is not available
        if not CV_BRIDGE_AVAILABLE or self.br is None:
            self.get_logger().warn("cv_bridge not available - skipping face detection")
            return
            
        try:
            face_classifier = cv2.CascadeClassifier(
                cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
            )
            vid = self.br.imgmsg_to_cv2(data)
            vid = cv2.cvtColor(vid, cv2.COLOR_BGR2RGB)
            gray_image = cv2.cvtColor(vid, cv2.COLOR_BGR2GRAY)
            faces = face_classifier.detectMultiScale(gray_image, 1.1, 5, minSize=(40, 40))
            W, H = vid.shape[1],vid.shape[0]
            
            # Initialize angle with current servo angle
            angle = self.servo_angle.angle
            
            for (x, y, w, h) in faces:
                x0 = x + w/2
                y0 = y + h/2
                if y0> H/2 and self.servo_angle.angle<self.bounds[0] - self.change:
                    angle = self.servo_angle.angle + self.change
                else :
                    if self.servo_angle.angle>self.bounds[1]+self.change:
                        angle = self.servo_angle.angle - self.change
                cv2.rectangle(vid, (x, y), (x + w, y + h), (0, 255, 0), 4)
                
            # Only update servo and publish if we have a valid angle
            if angle is not None:
                self.servo_angle.angle = angle
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                position = angle*np.pi/180.0
                msg.name = ["base_camera_joint"]
                msg.position = [position]
                msg.velocity = [1.0,1.0]
                msg.effort = [1.0,1.0]
                self.publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing video frame: {e}")
        

def main(args=None):
    rclpy.init(args=args)
    camera_controller = CameraController()
    rclpy.spin(camera_controller)
    camera_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


