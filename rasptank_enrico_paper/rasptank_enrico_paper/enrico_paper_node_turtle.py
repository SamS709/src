import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class UnicycleDynamicFeedbackTurtleNode(Node):
    """
    Dynamic feedback linearization controller for turtlesim (pose -> cmd_vel).

    Implements the controller from:
      "Stabilization of the Unicycle via Dynamic Feedback Linearization"
      De Luca, Oriolo, Vendittelli - IFAC 2000

    Key equations:
      ξdot = u1*cos(theta) + u2*sin(theta)          (eq. 4)
      v   = ξ                               (eq. 5)
      ω   = (-u1*sin(theta) + u2*cos(theta)) / ξ   (eq. 6)

    PD control laws (set-point regulation, eqs. 13-14):
      u1 = -kp1*x  - kd1*xdot
      u2 = -kp2*y  - kd2*ydot

    Stability conditions (Assumption A1, eqs. 15-16):
      kd1**2 - 4*kp1 = kd2**2 - 4*kp2  > 0
      kd2 - kd1    > 2*sqrt(kd2**2 - 4*kp2)

    Assumption A2 (sign of ξ_0):
      ξ_0 < 0  if  x_0 ≥ 0  (robot starts in Q_r -> backward motion toward origin)
      ξ_0 > 0  if  x_0 < 0  (robot starts in Q_l -> forward  motion toward origin)

    Forbidden initialisation (eq. 17):
      ξ_0 != 2*(kp1*x_0*sin(theta_0) - kp2*y_0*cos(theta_0)) / (kd2 - kd1)
    """

    #  Tuneable thresholds
    _XI_SINGULARITY_THRESH = 1e-4  # singularity guard
    _XI_RESET_BUMP = 0.05  # magnitude added when resetting ξ away from 0
    _FORBIDDEN_INIT_EPS = 0.05  # min distance from the forbidden ξ_0 value
    _FLOAT_EQ_EPS = 1e-9  # tolerance for floating-point equality checks

    # turtlesim offset
    _TS_OFFSET = [5.5, 5.5]

    def __init__(self) -> None:
        super().__init__("unicycle_dynamic_feedback_turtle")

        # ROS params
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("kp1", 2.0)
        self.declare_parameter("kd1", 3.0)
        self.declare_parameter("kp2", 12.0)
        self.declare_parameter("kd2", 7.0)
        self.declare_parameter("xi0_magnitude", 0.1)  # |ξ_0|; sign is set by A2
        self.declare_parameter("goal_stop_radius", 0.01)
        self.declare_parameter("goal_x", 5.5)
        self.declare_parameter("goal_y", 5.5)

        self.kp1 = float(self.get_parameter("kp1").value)
        self.kd1 = float(self.get_parameter("kd1").value)
        self.kp2 = float(self.get_parameter("kp2").value)
        self.kd2 = float(self.get_parameter("kd2").value)
        self._xi0_magnitude = abs(float(self.get_parameter("xi0_magnitude").value))
        self.goal_stop_radius = float(self.get_parameter("goal_stop_radius").value)
        self.goal_x = float(self.get_parameter("goal_x").value) + self._TS_OFFSET[0]
        self.goal_y = float(self.get_parameter("goal_y").value) + self._TS_OFFSET[1]

        # ξ is uninitialised until the first pose callback sets the sign (A2)
        self.xi: float | None = None
        self._xi_initialised = False

        # State
        self.pose_ready = False
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.goal_reached = False

        # Fix gains
        self._fix_parameters()

        # Ros pub / sub / timer
        self.cmd_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.create_subscription(Pose, "/turtle1/pose", self._pose_cb, 10)

        rate = float(self.get_parameter("control_rate_hz").value)
        self.dt = 1.0 / rate if rate > 0.0 else 0.02
        self.timer = self.create_timer(self.dt, self._on_timer)

        self.t0 = self.get_clock().now()
        self.get_logger().info(
            "Turtlesim controller started (/turtle1/pose -> /turtle1/cmd_vel)"
        )

    def _fix_parameters(self) -> None:
        """
        Enforce Assumption A1 (eqs. 15-16).  If the current gains violate
        either condition the smallest valid (kp2, kd2) pair is computed.
        """
        discriminant = self.kd1**2 - 4.0 * self.kp1

        if discriminant <= 0.0:
            self.get_logger().error(
                "kd1**2-4*kp1 must be strictly positive for real eigenvalues. "
                f"Got {discriminant}. Increase kd1 or decrease kp1."
            )
            raise ValueError("Invalid gains: discriminant ≤ 0")

        if not self._check_params():
            # Smallest kd2 satisfying eq. 16 with a small safety margin
            self.kd2 = self.kd1 + 2.0 * math.sqrt(discriminant) + 0.1
            # kp2 derived from eq. 15  ->  kp2 = (kd2**2-kd1**2+4*kp1)/4
            self.kp2 = (self.kd2**2 - self.kd1**2 + 4.0 * self.kp1) / 4.0
            self.get_logger().warning(
                "Gain conditions violated; auto-corrected to "
                f"kp2={self.kp2}, kd2={self.kd2}"
            )

        self.get_logger().info(
            f"Active gains - kp1={self.kp1}  kd1={self.kd1}  "
            f"kp2={self.kp2}  kd2={self.kd2}"
        )

    def _check_params(self) -> bool:
        """Return True iff Assumption A1 (eqs. 15-16) is satisfied."""
        lhs15 = self.kd1**2 - 4.0 * self.kp1
        rhs15 = self.kd2**2 - 4.0 * self.kp2

        # Condition 15: kd1**2-4kp1 = kd2**2-4kp2  (use float-safe comparison)
        if abs(lhs15 - rhs15) > self._FLOAT_EQ_EPS:
            self.get_logger().warning(
                f"Condition 15 violated: lhs={lhs15:.6f} != rhs={rhs15:.6f}"
            )
            return False

        # Condition 16: kd2-kd1 > 2*sqrt(kd2**2-4kp2)
        lhs16 = self.kd2 - self.kd1
        rhs16 = 2.0 * math.sqrt(max(rhs15, 0.0))
        if lhs16 <= rhs16:
            self.get_logger().warning(
                f"Condition 16 violated: kd2-kd1={lhs16} ≤ 2*sqrt(kd2**2-4kp2)={rhs16}"
            )
            return False

        return True

    def _init_xi(self) -> None:
        """
        Set the sign of ξ_0 according to Assumption A2, then check that the
        chosen value is not the forbidden initialisation (eq. 17).

        Called once, on the first pose callback after goal coordinates are
        expressed in a frame centred on the goal (i.e. using error coords).
        """
        # Translate to goal-centred coordinates (paper's origin = goal)
        ex = self.x - self.goal_x
        ey = self.y - self.goal_y
        th = self.theta

        # A2: sign rule
        #   Q_r  ->  ex ≥ 0  ->  ξ_0 < 0  (backward motion)
        #   Q_l  ->  ex < 0  ->  ξ_0 > 0  (forward  motion)
        sign = -1.0 if ex >= 0.0 else 1.0
        xi_candidate = sign * self._xi0_magnitude

        # Forbidden initialisation (eq. 17) - only defined when kd2 != kd1
        dkd = self.kd2 - self.kd1
        if abs(dkd) > self._FLOAT_EQ_EPS:
            xi_forbidden = (
                2.0
                * (self.kp1 * ex * math.sin(th) - self.kp2 * ey * math.cos(th))
                / dkd
            )
            if abs(xi_candidate - xi_forbidden) < self._FORBIDDEN_INIT_EPS:
                # Change away from the forbidden value
                xi_candidate += sign * self._FORBIDDEN_INIT_EPS * 2.0
                self.get_logger().warning(
                    f"ξ_0 too close to forbidden value ({xi_forbidden}); "
                    f"changed to {xi_candidate}"
                )
        else:
            xi_forbidden = None

        self.xi = xi_candidate
        self._xi_initialised = True
        self.get_logger().info(
            f"ξ_0 initialised to {self.xi}  "
            f"(forbidden={xi_forbidden if xi_forbidden is not None else 'N/A'})"
        )

    #  Pose callback
    def _pose_cb(self, msg: Pose) -> None:
        self.x = float(msg.x)
        self.y = float(msg.y)
        self.theta = float(msg.theta)
        self.pose_ready = True

        # Initialise ξ on the very first pose (requires knowing start pose)
        if not self._xi_initialised:
            self._init_xi()

    #  Desired trajectory (static goal -> set-point regulation)            #
    def _desired_trajectory(self, t: float):
        del t
        return self.goal_x, self.goal_y, 0.0, 0.0, 0.0, 0.0

    #  Control timer                                                       #
    def _on_timer(self) -> None:
        if not self.pose_ready or not self._xi_initialised:
            return

        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        # Cartesian velocities from the compensator state ξ
        # FIX: use ξ (internal model state) rather than v_meas from odometry
        # (eqs. 9-10).
        xdot = self.xi * math.cos(self.theta)
        ydot = self.xi * math.sin(self.theta)

        xd, yd, xdot_d, ydot_d, xddot_d, yddot_d = self._desired_trajectory(t)

        # PD control laws (eqs. 13-14)
        u1 = xddot_d + self.kp1 * (xd - self.x) + self.kd1 * (xdot_d - xdot)
        u2 = yddot_d + self.kp2 * (yd - self.y) + self.kd2 * (ydot_d - ydot)

        # Dynamic compensator update (eq. 4)
        xi_dot = u1 * math.cos(self.theta) + u2 * math.sin(self.theta)
        self.xi += xi_dot * self.dt

        # Singularity handling
        # If |ξ| falls below the threshold the ω expression (eq. 6) is undefined
        # Following the paper's recommendation (Section 2.2),
        # reset ξ to a small value with the correct sign so that rolling is re-established and the controller can continue.
        if abs(self.xi) < self._XI_SINGULARITY_THRESH:
            sign_before = math.copysign(1.0, self.xi) if self.xi != 0.0 else -1.0
            self.xi = sign_before * self._XI_RESET_BUMP
            self.get_logger().warning(
                f"ξ singularity detected; reset to {self.xi}  "
                "(isolated discontinuity in v - see paper Section 2.2)"
            )

        # Velocity commands (eqs. 5-6)
        v_cmd = self.xi
        omega_cmd = (-u1 * math.sin(self.theta) + u2 * math.cos(self.theta)) / self.xi

        # Goal-stop
        dist = math.hypot(self.x - self.goal_x, self.y - self.goal_y)
        if dist <= self.goal_stop_radius:
            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info(
                    "Goal reached at (%.3f, %.3f) - distance=%.4f"
                    % (self.x - self._TS_OFFSET[0], self.y - self._TS_OFFSET[1], dist)
                )
            v_cmd = 0.0
            omega_cmd = 0.0
        else:
            self.goal_reached = False

        # Publish
        msg = Twist()
        msg.linear.x = float(v_cmd)
        msg.angular.z = float(omega_cmd)
        self.cmd_pub.publish(msg)


#  Entry point
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
