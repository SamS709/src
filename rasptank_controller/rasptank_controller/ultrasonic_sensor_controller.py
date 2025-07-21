#!/usr/bin/env python3

"""
Ultrasonic Sensor Controller Node for RaspTank

This node subscribes to the 'frontal_robot_distance' topic and publishes
velocity commands to the 'cmd_vel' topic to avoid obstacles.

Author: ROS2 RaspTank Controller
License: MIT
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped
import math


class UltrasonicSensorController(Node):
    """
    ROS2 node that subscribes to ultrasonic distance data and publishes
    velocity commands for obstacle avoidance.
    """

    def __init__(self):
        super().__init__('ultrasonic_sensor_controller')
        
        # Distance thresholds (in centimeters)
        self.stop_distance = 15.0      # Stop completely if obstacle is closer than 15cm
        self.slow_distance = 30.0      # Slow down if obstacle is closer than 30cm
        self.warning_distance = 50.0   # Start reducing speed if obstacle is closer than 50cm
        
        # Speed parameters
        self.max_linear_speed = 0.3    # Maximum forward speed (m/s)
        self.min_linear_speed = 0.05   # Minimum forward speed (m/s)
        self.backup_speed = -0.05       # Backup speed when too close (m/s)
        self.turn_speed = 2.0        # Angular speed for turning (rad/s)
        
        # State variables
        self.current_distance = float('inf')  # Current distance reading
        self.obstacle_detected = False
        self.backup_counter = 0        # Counter for backup duration
        self.turn_counter = 0          # Counter for turn duration
        self.state = "forward"         # States: "forward", "backup", "turn"
        
        # Create subscriber for distance data
        self.distance_subscription = self.create_subscription(
            Float32,
            'frontal_robot_distance',
            self.distance_callback,
            10)
        
        # Create publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        
        # Create timer for control loop (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_callback)
        
        # Log initialization
        self.get_logger().info('Ultrasonic Sensor Controller initialized')
        self.get_logger().info(f'Stop distance: {self.stop_distance} cm')
        self.get_logger().info(f'Slow distance: {self.slow_distance} cm')
        self.get_logger().info(f'Warning distance: {self.warning_distance} cm')

    def distance_callback(self, msg):
        """
        Callback function for distance data.
        
        Args:
            msg (Float32): Distance message in centimeters
        """
        self.current_distance = msg.data
        
        # Update obstacle detection status
        if self.current_distance < self.warning_distance:
            if not self.obstacle_detected:
                self.get_logger().info(f'Obstacle detected at {self.current_distance:.1f} cm')
            self.obstacle_detected = True
        else:
            if self.obstacle_detected:
                self.get_logger().info('Obstacle cleared, resuming normal operation')
            self.obstacle_detected = False

    def calculate_speed(self, distance):
        """
        Calculate appropriate speed based on distance to obstacle.
        
        Args:
            distance (float): Distance to obstacle in centimeters
            
        Returns:
            float: Linear speed in m/s
        """
        if distance <= self.stop_distance:
            return 0.0
        elif distance <= self.slow_distance:
            # Linear interpolation between min speed and 0
            ratio = (distance - self.stop_distance) / (self.slow_distance - self.stop_distance)
            return self.min_linear_speed * ratio
        elif distance <= self.warning_distance:
            # Linear interpolation between min and max speed
            ratio = (distance - self.slow_distance) / (self.warning_distance - self.slow_distance)
            return self.min_linear_speed + (self.max_linear_speed - self.min_linear_speed) * ratio
        else:
            return self.max_linear_speed

    def control_callback(self):
        """
        Main control loop callback function.
        """
        # Create stamped velocity message (matching teleop_twist_keyboard format)
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.header.frame_id = ''  # Empty frame_id like teleop_twist_keyboard
        
        # Initialize all twist values to zero
        cmd_vel.twist.linear.x = 0.0
        cmd_vel.twist.linear.y = 0.0
        cmd_vel.twist.linear.z = 0.0
        cmd_vel.twist.angular.x = 0.0
        cmd_vel.twist.angular.y = 0.0
        cmd_vel.twist.angular.z = 0.0
        
        # State machine for obstacle avoidance behavior
        if self.state == "forward":
            if self.current_distance <= self.stop_distance:
                # Too close, switch to backup state
                self.state = "backup"
                self.backup_counter = 20  # Backup for 1 second (20 * 0.05s)
                self.get_logger().info('Switching to backup mode')
            else:
                # Normal forward movement with speed adjustment
                linear_speed = self.calculate_speed(self.current_distance)/3
                cmd_vel.twist.linear.x = linear_speed
                
                if linear_speed < self.max_linear_speed:
                    self.get_logger().debug(f'Reduced speed: {linear_speed:.2f} m/s due to obstacle at {self.current_distance:.1f} cm')
        
        elif self.state == "backup":
            # Backup for a short duration
            cmd_vel.twist.linear.x = self.backup_speed
            self.backup_counter -= 1
            
            if self.backup_counter <= 0:
                # Switch to turn state
                self.state = "turn"
                self.turn_counter = 30  # Turn for 1.5 seconds (30 * 0.05s)
                self.get_logger().info('Switching to turn mode')
        
        elif self.state == "turn":
            # Turn in place to avoid obstacle
            cmd_vel.twist.angular.z = self.turn_speed  # Turn right (positive angular velocity)
            self.turn_counter -= 1
            
            if self.turn_counter <= 0:
                # Switch back to forward state
                self.state = "forward"
                self.get_logger().info('Resuming forward movement')
        
        # Publish stamped velocity command
        self.cmd_vel_publisher.publish(cmd_vel)
        
        # Log current state occasionally
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
            
        if self._log_counter % 40 == 0:  # Log every 2 seconds
            self.get_logger().info(f'State: {self.state}, Distance: {self.current_distance:.1f} cm, Speed: {cmd_vel.twist.linear.x:.2f} m/s, Angular: {cmd_vel.twist.angular.z:.2f} rad/s')


def main(args=None):
    rclpy.init(args=args)
    
    ultrasonic_controller = UltrasonicSensorController()
    
    try:
        rclpy.spin(ultrasonic_controller)
    except KeyboardInterrupt:
        pass
    finally:
        ultrasonic_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
