import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random
import math
from time import time

# Try to import gpiozero for real hardware, fallback to mock for testing
try:
    from gpiozero import DistanceSensor
    USE_REAL_SENSOR = True
except Exception:
    USE_REAL_SENSOR = False


class MockDistanceSensor:
    """Mock distance sensor for testing purposes."""
    def __init__(self, echo=None, trigger=None, max_distance=2):
        self.max_distance = max_distance
        self.start_time = time()
    
    @property
    def distance(self):
        # Simulate realistic distance readings (0.1 to 1.5 meters)
        # Add some periodic variation to simulate moving objects
        base_distance = 0.5 + 0.3 * math.sin((time() - self.start_time) * 0.5)
        noise = random.uniform(-0.05, 0.05)  # Add some noise
        return max(0.1, min(self.max_distance, base_distance + noise))


class UltrasonicSensorPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor')
        
        # GPIO pins for ultrasonic sensor
        self.trigger_pin = 23
        self.echo_pin = 24
        
        # Initialize the ultrasonic sensor (real or mock)
        try:
            if USE_REAL_SENSOR:
                self.sensor = DistanceSensor(echo=self.echo_pin, trigger=self.trigger_pin, max_distance=2)
                self.get_logger().info('Using real ultrasonic sensor')
            else:
                self.sensor = MockDistanceSensor(echo=self.echo_pin, trigger=self.trigger_pin, max_distance=2)
                self.get_logger().info('Using mock ultrasonic sensor for testing')
        except Exception as e:
            self.get_logger().warn(f'Failed to initialize real sensor, using mock: {e}')
            self.sensor = MockDistanceSensor(echo=self.echo_pin, trigger=self.trigger_pin, max_distance=2)
        
        # Create publisher for frontal robot distance
        self.publisher_ = self.create_publisher(Float32, 'frontal_robot_distance', 10)
        
        # Create timer to publish distance readings every 0.1 seconds (10 Hz)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Ultrasonic sensor publisher has been started')

    def checkdist(self):
        """Get the distance of ultrasonic detection in centimeters."""
        try:
            return (self.sensor.distance) * 100  # Unit: cm
        except Exception as e:
            self.get_logger().warn(f'Failed to read sensor: {e}')
            return -1.0  # Return -1 to indicate error

    def timer_callback(self):
        """Timer callback to publish distance readings."""
        distance = self.checkdist()
        
        if distance >= 0:  # Only publish valid readings
            msg = Float32()
            msg.data = distance
            self.publisher_.publish(msg)
            self.get_logger().debug(f'Published distance: {distance:.2f} cm')


def main(args=None):
    rclpy.init(args=args)
    
    ultrasonic_publisher = UltrasonicSensorPublisher()
    
    try:
        rclpy.spin(ultrasonic_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        ultrasonic_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()