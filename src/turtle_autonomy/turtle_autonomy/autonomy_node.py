import math
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.msg import Pose

def euclidean(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def predict_multistep(x, y, theta, v, omega, dt, steps):
    for _ in range(steps):
        x     = x     + v * math.cos(theta) * dt
        y     = y     + v * math.sin(theta) * dt
        theta = theta + omega * dt
    return x, y

class TurtleAutonomyNode(Node):

    def __init__(self):
        super().__init__('turtle_autonomy')

        self.declare_parameter('v',               1.5)
        self.declare_parameter('dt',              0.1)
        self.declare_parameter('lookahead_steps', 20)
        self.declare_parameter('omega_max',       4.0)
        self.declare_parameter('omega_samples',   61)
        self.declare_parameter('goal_tolerance',  0.3)
        self.declare_parameter('control_rate',    20.0)

        self._read_params()

        self._pose    = None
        self._goal    = None
        self._reached = False

        self._cmd_pub    = self.create_publisher(Twist,  '/turtle1/cmd_vel',         10)
        self._status_pub = self.create_publisher(String, '/turtle1/autonomy_status', 10)

        self.create_subscription(Pose,        '/turtle1/pose', self._pose_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose',    self._goal_cb, 10)

        period = 1.0 / self._control_rate
        self._timer = self.create_timer(period, self._control_loop)

        horizon = self._v * self._dt * self._lookahead_steps
        self.get_logger().info(
            '\nTurtleAutonomyNode ready  [multi-step lookahead]\n'
            '  v               = %.2f m/s\n'
            '  dt              = %.2f s\n'
            '  lookahead_steps = %d  (horizon=%.2f m)\n'
            '  omega_max       = %.2f rad/s\n'
            '  omega_samples   = %d\n'
            '  goal_tolerance  = %.2f m\n'
            '  control_rate    = %.1f Hz\n'
            '\nSend a goal:\n'
            '  ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped '
            '"{pose: {position: {x: 8.0, y: 8.0}}}" --once'
            % (self._v, self._dt, self._lookahead_steps, horizon,
               self._omega_max, self._omega_samples, self._goal_tol,
               self._control_rate)
        )

    def _read_params(self):
        self._v               = float(self.get_parameter('v').value)
        self._dt              = float(self.get_parameter('dt').value)
        self._lookahead_steps = int(self.get_parameter('lookahead_steps').value)
        self._omega_max       = float(self.get_parameter('omega_max').value)
        self._omega_samples   = int(self.get_parameter('omega_samples').value)
        self._goal_tol        = float(self.get_parameter('goal_tolerance').value)
        self._control_rate    = float(self.get_parameter('control_rate').value)

    def _pose_cb(self, msg):
        self._pose = msg

    def _goal_cb(self, msg):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self._goal    = (gx, gy)
        self._reached = False
        self.get_logger().info('New goal: (%.2f, %.2f)' % (gx, gy))

    def _select_best_omega(self, x, y, theta, gx, gy):
        omegas = np.linspace(-self._omega_max, self._omega_max,
                             self._omega_samples)

        best_omega = 0.0
        best_dist  = float('inf')

        for omega in omegas:
            xp, yp = predict_multistep(
                x, y, theta,
                self._v, float(omega), self._dt,
                self._lookahead_steps,
            )
            d = euclidean(xp, yp, gx, gy)
            if d < best_dist:
                best_dist  = d
                best_omega = float(omega)

        return best_omega

    def _control_loop(self):
        if self._pose is None or self._goal is None:
            return

        x, y, theta = self._pose.x, self._pose.y, self._pose.theta
        gx, gy      = self._goal
        dist        = euclidean(x, y, gx, gy)

        # Goal reached
        if dist < self._goal_tol:
            if not self._reached:
                self._reached = True
                self._stop()
                self.get_logger().info(
                    'Goal reached! dist=%.3f m  tol=%.3f m' % (dist, self._goal_tol)
                )
                status_msg = String()
                status_msg.data = 'GOAL_REACHED'
                self._status_pub.publish(status_msg)
            return

        omega_best = self._select_best_omega(x, y, theta, gx, gy)

        cmd = Twist()
        cmd.linear.x  = self._v
        cmd.angular.z = omega_best
        self._cmd_pub.publish(cmd)

        self.get_logger().debug(
            'pos=(%.2f,%.2f) theta=%.1fdeg  dist=%.2f  omega=%.3f'
            % (x, y, math.degrees(theta), dist, omega_best)
        )

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def destroy_node(self):
        self._stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleAutonomyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
