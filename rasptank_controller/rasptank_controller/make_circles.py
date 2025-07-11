import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleMaker(Node):
    def __init__(self):
        super().__init__('make_circles')
        self.cmd_vel_pub = self.create_publisher(msg_type=Twist, topic='/cmd_vel', qos_profile=10)
        self.timer = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info('make_circles Node has been started.')
        # Here you would add the logic to make the robot move in circles.
        # This is just a placeholder for demonstration purposes.

    def send_velocity_command(self):
        self.get_logger().info('Making circles...')
        msg = Twist()
        msg.linear.x = 0.2  # Forward speed
        msg.angular.z = 0.5  # Angular speed for turning
        self.cmd_vel_pub.publish(msg)

def main(args = None):
    rclpy.init(args=args)
    node = CircleMaker()
    rclpy.spin(node)
    rclpy.shutdown()
    