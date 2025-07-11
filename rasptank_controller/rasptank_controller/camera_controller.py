#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
from time import time
import numpy as np

class SinusoidalJointPublisher(Node):
    def __init__(self):
        super().__init__('sinusoidal_joint_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.start_time = time()

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        angle = 95
        position = angle*np.pi/180.0
        msg.name = ["base_camera_joint"]
        msg.position = [position]
        msg.velocity = [1.0,1.0]
        msg.effort = [1.0,1.0]
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing joint position: {position}')

def main(args=None):
    rclpy.init(args=args)
    node = SinusoidalJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()