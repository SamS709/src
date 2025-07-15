#!/usr/bin/env python3

"""
Motor Controller Node for RaspTank Differential Drive Robot

This node subscribes to the /cmd_motors topic and processes motor commands.
The commands are received as Float64MultiArray messages containing velocity
commands for the left and right motors.

Author: ROS2 Control Demo
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

# Hardware control imports
try:
    from board import SCL, SDA
    import busio
    from adafruit_pca9685 import PCA9685
    from adafruit_motor import motor
    HARDWARE_AVAILABLE = True
except ImportError as e:
    HARDWARE_AVAILABLE = False
    print(f"Hardware libraries not available: {e}")
    print("Running in simulation mode")


class MotorController(Node):
    """
    ROS2 node that subscribes to /cmd_motors topic and processes motor commands.
    """

    def __init__(self):
        super().__init__('motor_controller')
        
        # Motor pin definitions (from 04_Motor.py)
        self.MOTOR_M1_IN1 = 15      # Define the positive pole of M1 (left motor)
        self.MOTOR_M1_IN2 = 14      # Define the negative pole of M1
        self.MOTOR_M2_IN1 = 12      # Define the positive pole of M2 (right motor)
        self.MOTOR_M2_IN2 = 13      # Define the negative pole of M2
        
        # Initialize hardware
        self.hardware_initialized = False
        self.pwm_motor = None
        self.motor1 = None  # Left motor (channel 1)
        self.motor2 = None  # Right motor (channel 2)
        
        self.init_hardware()
        
        # Create subscriber for motor commands
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'cmd_motors',
            self.motor_command_callback,
            10)
        
        # Initialize motor state tracking
        self.last_command_time = time.time()
        self.motor_commands = [0.0, 0.0]  # [left_motor, right_motor]
        
        # Log initialization
        self.get_logger().info('Motor Controller Node initialized')
        self.get_logger().info('Subscribing to /cmd_motors topic')
        if self.hardware_initialized:
            self.get_logger().info('Hardware interface initialized successfully')
        else:
            self.get_logger().warn('Hardware interface not available - running in simulation mode')
        
        # Create a timer for periodic status reporting
        self.timer = self.create_timer(2.0, self.status_callback)

    def init_hardware(self):
        """
        Initialize the PCA9685 PWM controller and motor objects.
        """
        if not HARDWARE_AVAILABLE:
            self.get_logger().warn("Hardware libraries not available")
            return
            
        try:
            # Initialize I2C and PCA9685
            i2c = busio.I2C(SCL, SDA)
            self.pwm_motor = PCA9685(i2c, address=0x5f)  # Use address from 04_Motor.py
            self.pwm_motor.frequency = 50
            
            # Initialize motors
            self.motor1 = motor.DCMotor(
                self.pwm_motor.channels[self.MOTOR_M1_IN1],
                self.pwm_motor.channels[self.MOTOR_M1_IN2]
            )
            self.motor1.decay_mode = motor.SLOW_DECAY
            
            self.motor2 = motor.DCMotor(
                self.pwm_motor.channels[self.MOTOR_M2_IN1],
                self.pwm_motor.channels[self.MOTOR_M2_IN2]
            )
            self.motor2.decay_mode = motor.SLOW_DECAY
            
            # Stop all motors initially
            self.motor_stop()
            
            self.hardware_initialized = True
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize hardware: {e}")
            self.hardware_initialized = False

    def map_value(self, x, in_min, in_max, out_min, out_max):
        """
        Map a value from one range to another (from 04_Motor.py).
        """
        return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min

    def motor_command_callback(self, msg):
        """
        Callback function for motor command messages.
        
        Args:
            msg (Float64MultiArray): Motor velocity commands [left, right]
        """
        self.last_command_time = time.time()
        
        if len(msg.data) >= 2:
            self.motor_commands[0] = msg.data[0]  # Left motor
            self.motor_commands[1] = msg.data[1]  # Right motor
            
            self.get_logger().info(
                f'Received motor commands: Left={self.motor_commands[0]:.3f}, '
                f'Right={self.motor_commands[1]:.3f}'
            )
            
            # Here you would add the actual motor control logic
            # For example, sending commands to motor drivers
            self.send_commands_to_motors(self.motor_commands)
        else:
            self.get_logger().warn(
                f'Invalid motor command received: expected 2 values, got {len(msg.data)}'
            )

    def send_commands_to_motors(self, commands):
        """
        Send velocity commands to the physical motors.
        
        Args:
            commands (list): [left_motor_velocity, right_motor_velocity]
        """
        if not self.hardware_initialized:
            # Simulation mode
            self.get_logger().debug(
                f'SIMULATION: Left={commands[0]:.3f}, Right={commands[1]:.3f}'
            )
            return
            
        try:
            left_vel = commands[0]   # Left motor velocity (rad/s)
            right_vel = commands[1]  # Right motor velocity (rad/s)
            
            # Convert velocities to motor speeds (0-100%)
            # Assuming max velocity is around 10 rad/s for scaling
            max_velocity = 10.0  # rad/s
            
            # Calculate speed percentages and directions
            left_speed_percent = min(100, abs(left_vel) / max_velocity * 100)
            right_speed_percent = min(100, abs(right_vel) / max_velocity * 100)
            
            left_direction = 1 if left_vel >= 0 else -1
            right_direction = 1 if right_vel >= 0 else -1
            
            # Apply commands to motors
            # For differential drive: motor1 = left motor, motor2 = right motor
            self.set_motor(1, left_direction, left_speed_percent)   # Left motor
            self.set_motor(2, right_direction, right_speed_percent) # Right motor
            
            self.get_logger().debug(
                f'Motors: L={left_speed_percent:.1f}%({left_direction:+d}), '
                f'R={right_speed_percent:.1f}%({right_direction:+d})'
            )
            
        except Exception as e:
            self.get_logger().error(f"Error controlling motors: {e}")

    def set_motor(self, channel, direction, motor_speed):
        """
        Control individual motor (adapted from 04_Motor.py).
        
        Args:
            channel (int): Motor channel (1-2)
            direction (int): Direction (1 = forward, -1 = backward)
            motor_speed (float): Speed percentage (0-100)
        """
        if not self.hardware_initialized:
            return
            
        # Clamp motor speed
        if motor_speed > 100:
            motor_speed = 100
        elif motor_speed < 0:
            motor_speed = 0
            
        # Convert to throttle value (-1.0 to 1.0)
        throttle = self.map_value(motor_speed, 0, 100, 0, 1.0)
        if direction == -1:
            throttle = -throttle
            
        try:
            if channel == 1:
                self.motor1.throttle = throttle
            elif channel == 2:
                self.motor2.throttle = throttle
        except Exception as e:
            self.get_logger().error(f"Error setting motor {channel}: {e}")

    def motor_stop(self):
        """
        Stop all motors.
        """
        if not self.hardware_initialized:
            return
            
        try:
            self.motor1.throttle = 0
            self.motor2.throttle = 0
        except Exception as e:
            self.get_logger().error(f"Error stopping motors: {e}")

    def cleanup_hardware(self):
        """
        Clean up hardware resources.
        """
        if self.hardware_initialized:
            try:
                self.motor_stop()
                if self.pwm_motor:
                    self.pwm_motor.deinit()
                self.get_logger().info("Hardware cleaned up successfully")
            except Exception as e:
                self.get_logger().error(f"Error during hardware cleanup: {e}")
            finally:
                self.hardware_initialized = False

    def status_callback(self):
        """
        Periodic status reporting callback.
        """
        time_since_last_cmd = time.time() - self.last_command_time
        
        if time_since_last_cmd > 5.0:
            self.get_logger().info(
                'Motor Controller running - No commands received in the last 5 seconds'
            )
        else:
            self.get_logger().info(
                f'Motor Controller running - Last commands: '
                f'Left={self.motor_commands[0]:.3f}, Right={self.motor_commands[1]:.3f}'
            )


def main(args=None):
    """
    Main function to initialize and run the motor controller node.
    """
    rclpy.init(args=args)
    
    motor_controller = None
    try:
        motor_controller = MotorController()
        
        # Keep the node running
        rclpy.spin(motor_controller)
        
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
    except Exception as e:
        print(f'Error in motor controller: {e}')
    finally:
        # Clean shutdown
        if motor_controller:
            try:
                motor_controller.cleanup_hardware()
                motor_controller.destroy_node()
            except:
                pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
