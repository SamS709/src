
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
import time


model = YOLO("yolo11n.yaml")

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
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
        W, H = vid.shape[1], vid.shape[0]
        
        # Find the largest face (closest to camera)
        largest_face = None
        largest_area = 0
        
        for (x, y, w, h) in faces:
            area = w * h
            if area > largest_area:
                largest_area = area
                largest_face = (x, y, w, h)
        
        # Draw rectangle only around the nearest face
        if largest_face:
            x, y, w, h = largest_face
            cv2.rectangle(vid, (x, y), (x + w, y + h), (0, 255, 0), 4)
            
            # Calculate center point for logging
            x0 = x + w/2
            y0 = y + h/2
            self.get_logger().info(f"Tracking nearest face - Area: {largest_area}, Center: ({x0:.1f}, {y0:.1f})")
        
        cv2.imshow("Face Detection", vid)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


