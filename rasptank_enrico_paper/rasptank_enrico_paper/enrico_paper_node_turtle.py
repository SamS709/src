import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class UnicycleDynamicFeedbackTurtleNode(Node):
    """Dynamic feedback controller for turtlesim (pose -> cmd_vel)."""

    def __init__(self) -> None:
        super().__init__("unicycle_dynamic_feedback_turtle")

        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("kp1", 2.0)
        self.declare_parameter("kd1", 3.0)
        self.declare_parameter("kp2", 12.0)
        self.declare_parameter("kd2", 7.0)
        self.declare_parameter("xi0", 0.6)
        self.declare_parameter("goal_stop_radius", 0.01)
        self.declare_parameter("goal_x", 5.5)
        self.declare_parameter("goal_y", 5.5)

        self.kp1 = float(self.get_parameter("kp1").value)
        self.kd1 = float(self.get_parameter("kd1").value)
        self.kp2 = float(self.get_parameter("kp2").value)
        self.kd2 = float(self.get_parameter("kd2").value)
        self.xi = float(self.get_parameter("xi0").value)
        self.goal_stop_radius = float(self.get_parameter("goal_stop_radius").value)
        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)

        self.pose_ready = False
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_meas = 0.0
        self.goal_reached = False

        # Publisher to /turtle1/cmd_vel and subscriber to /turtle1/pose
        self.cmd_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.create_subscription(Pose, "/turtle1/pose", self._pose_cb, 10)

        rate = float(self.get_parameter("control_rate_hz").value)
        self.dt = 1.0 / rate if rate > 0.0 else 0.02
        self.timer = self.create_timer(self.dt, self._on_timer)

        self.t0 = self.get_clock().now()

        self.get_logger().info("Turtlesim controller started (/turtle1/pose -> /turtle1/cmd_vel)")

    def _pose_cb(self, msg: Pose) -> None:
        self.x = float(msg.x)
        self.y = float(msg.y)
        self.theta = float(msg.theta)
        self.v_meas = float(msg.linear_velocity)
        self.pose_ready = True

    def _desired_trajectory(self, t: float):
        del t
        return self.goal_x, self.goal_y, 0.0, 0.0, 0.0, 0.0

    def _on_timer(self) -> None:
        if not self.pose_ready:
            return

        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        xdot = self.v_meas * math.cos(self.theta)
        ydot = self.v_meas * math.sin(self.theta)

        xd, yd, xdot_d, ydot_d, xddot_d, yddot_d = self._desired_trajectory(t)

        u1 = xddot_d + self.kp1 * (xd - self.x) + self.kd1 * (xdot_d - xdot)
        u2 = yddot_d + self.kp2 * (yd - self.y) + self.kd2 * (ydot_d - ydot)

        xi_dot = u1 * math.cos(self.theta) + u2 * math.sin(self.theta)
        self.xi += xi_dot * self.dt

        v_cmd = self.xi
        omega_cmd = 0.0
        if abs(self.xi) > 1e-6:
            omega_cmd = (-u1 * math.sin(self.theta) + u2 * math.cos(self.theta)) / self.xi

        dx = self.x - self.goal_x
        dy = self.y - self.goal_y
        dist = math.hypot(dx, dy)
        if dist <= self.goal_stop_radius:
            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info(
                    "Turtle goal reached at (%.3f, %.3f), radius=%.3f" %
                    (self.goal_x, self.goal_y, self.goal_stop_radius)
                )
            v_cmd = 0.0
            omega_cmd = 0.0
        else:
            self.goal_reached = False

        msg = Twist()
        msg.linear.x = float(v_cmd)
        msg.angular.z = float(omega_cmd)
        self.cmd_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnicycleDynamicFeedbackTurtleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
