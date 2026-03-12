import math
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.msg import Pose
from turtle_interfaces.srv import GetRobotPoses

def dist(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def simulate_trajectory(x, y, theta, v, omega, dt, steps):
    """Roll out unicycle model; return list of (x,y) at every step."""
    pts = []
    for _ in range(steps):
        x     = x + v * math.cos(theta) * dt
        y     = y + v * math.sin(theta) * dt
        theta = theta + omega * dt
        pts.append((x, y))
    return pts

def trajectory_cost(pts, gx, gy, obstacles, repulsion_weight):
    goal_d = dist(pts[-1][0], pts[-1][1], gx, gy)
    if not obstacles or repulsion_weight == 0.0:
        return goal_d
    min_clr = min(
        dist(px, py, ox, oy)
        for (px, py) in pts
        for (ox, oy) in obstacles
    )
    return goal_d + repulsion_weight / max(min_clr, 1e-3)

def project_safe_goal(gx, gy, obstacles, safe_r):
    blocked = False
    for (ox, oy) in obstacles:
        dx, dy   = gx - ox, gy - oy
        d        = math.sqrt(dx * dx + dy * dy)
        if d < safe_r:
            blocked = True
            if d < 1e-6:                      
                gx, gy = ox + safe_r, oy
            else:
                nx, ny = dx / d, dy / d
                gx, gy = ox + nx * safe_r, oy + ny * safe_r
    return gx, gy, blocked

class TurtleAutonomyNode(Node):

    def __init__(self):
        super().__init__('turtle_autonomy')

        self.declare_parameter('robot_name',        'turtle1')
        self.declare_parameter('v',                 1.5)
        self.declare_parameter('dt',                0.1)
        self.declare_parameter('lookahead_steps',   20)
        self.declare_parameter('omega_max',         4.0)
        self.declare_parameter('omega_samples',     61)
        self.declare_parameter('goal_tolerance',    0.3)
        self.declare_parameter('obstacle_radius',   1.2)
        self.declare_parameter('repulsion_weight',  3.0)
        self.declare_parameter('control_rate',      20.0)
        self.declare_parameter('obstacle_refresh',  10.0)
        self._p()  

        self._pose        = None
        self._goal_raw    = None  
        self._goal_active = None  
        self._blocked     = False  
        self._reached     = False
        self._obstacles   = []
        self._pending_obs = False

        self._cmd    = self.create_publisher(Twist,  '/%s/cmd_vel'         % self._name, 10)
        self._status = self.create_publisher(String, '/%s/autonomy_status' % self._name, 10)

        self.create_subscription(Pose,        '/%s/pose' % self._name, self._on_pose, 10)
        self.create_subscription(PoseStamped, '/goal_pose',             self._on_goal, 10)

        self._cli = self.create_client(GetRobotPoses, '/get_robot_poses')

        self.create_timer(1.0 / self._ctrl_hz,  self._control_loop)
        self.create_timer(1.0 / self._obs_hz,   self._query_obstacles)

        self.get_logger().info(
            '\nTurtleAutonomyNode ready\n'
            '  robot            = %s\n'
            '  v / dt           = %.2f m/s  / %.2f s\n'
            '  lookahead        = %d steps  (%.2f m)\n'
            '  omega            = ±%.1f rad/s  (%d samples)\n'
            '  goal_tolerance   = %.2f m\n'
            '  obstacle_radius  = %.2f m\n'
            '  repulsion_weight = %.2f\n'
            '  Modes: NORMAL (repulsion on) | BLOCKED (repulsion off, stop at edge)'
            % (self._name, self._v, self._dt,
               self._lookahead, self._v * self._dt * self._lookahead,
               self._omega_max, self._n_omega,
               self._tol, self._obs_r, self._rep_w)
        )

    def _p(self):
        g = self.get_parameter
        self._name     = g('robot_name').value
        self._v        = float(g('v').value)
        self._dt       = float(g('dt').value)
        self._lookahead= int(g('lookahead_steps').value)
        self._omega_max= float(g('omega_max').value)
        self._n_omega  = int(g('omega_samples').value)
        self._tol      = float(g('goal_tolerance').value)
        self._obs_r    = float(g('obstacle_radius').value)
        self._rep_w    = float(g('repulsion_weight').value)
        self._ctrl_hz  = float(g('control_rate').value)
        self._obs_hz   = float(g('obstacle_refresh').value)

    def _on_pose(self, msg):
        self._pose = msg

    def _on_goal(self, msg):
        self._goal_raw  = (msg.pose.position.x, msg.pose.position.y)
        self._reached   = False
        self.get_logger().info('Goal set: (%.2f, %.2f)' % self._goal_raw)

    def _query_obstacles(self):
        if self._pending_obs or not self._cli.service_is_ready():
            return
        req = GetRobotPoses.Request()
        req.requester_name = self._name
        self._pending_obs = True
        self._cli.call_async(req).add_done_callback(self._obs_done)

    def _obs_done(self, future):
        self._pending_obs = False
        try:
            r = future.result()
            self._obstacles = list(zip(r.xs, r.ys))
        except Exception as e:
            self.get_logger().warn('Obstacle query failed: %s' % e)

    def _resolve_goal(self):
        gx, gy, blocked = project_safe_goal(
            self._goal_raw[0], self._goal_raw[1],
            self._obstacles, self._obs_r,
        )

        if blocked and not self._blocked:
            self._blocked = True
            self._reached = False
            self.get_logger().warn(
                'BLOCKED: goal (%.2f,%.2f) is inside obstacle zone.\n'
                '         Navigating to safe edge point (%.2f,%.2f).\n'
                '         Repulsion disabled for final approach.'
                % (self._goal_raw[0], self._goal_raw[1], gx, gy)
            )

        elif not blocked and self._blocked:
            self._blocked = False
            self._reached = False
            self.get_logger().info(
                'UNBLOCKED: obstacle cleared, resuming to (%.2f,%.2f)'
                % self._goal_raw
            )

        self._goal_active = (gx, gy)

    def _best_omega(self, x, y, theta, gx, gy):
        rep_w = 0.0 if self._blocked else self._rep_w

        omegas     = np.linspace(-self._omega_max, self._omega_max, self._n_omega)
        best_omega = 0.0
        best_cost  = float('inf')

        for omega in omegas:
            pts  = simulate_trajectory(x, y, theta, self._v,
                                       float(omega), self._dt, self._lookahead)
            cost = trajectory_cost(pts, gx, gy, self._obstacles, rep_w)
            if cost < best_cost:
                best_cost, best_omega = cost, float(omega)

        return best_omega

    def _control_loop(self):
        if self._pose is None or self._goal_raw is None:
            return

        self._resolve_goal()

        x, y, theta = self._pose.x, self._pose.y, self._pose.theta
        gx, gy      = self._goal_active
        d           = dist(x, y, gx, gy)

        if d < self._tol:
            if not self._reached:
                self._reached = True
                self._stop()
                if self._blocked:
                    d_raw = dist(x, y, self._goal_raw[0], self._goal_raw[1])
                    self.get_logger().warn(
                        'GOAL_BLOCKED: stopped at safe point (%.2f,%.2f)  '
                        '[%.2f m from requested goal (%.2f,%.2f)]'
                        % (x, y, d_raw, self._goal_raw[0], self._goal_raw[1])
                    )
                    self._pub_status('GOAL_BLOCKED')
                else:
                    self.get_logger().info(
                        'GOAL_REACHED: (%.2f,%.2f)  dist=%.3f m' % (gx, gy, d)
                    )
                    self._pub_status('GOAL_REACHED')
            return

        self._reached = False
        omega = self._best_omega(x, y, theta, gx, gy)
        cmd   = Twist()
        cmd.linear.x  = self._v
        cmd.angular.z = omega
        self._cmd.publish(cmd)

        self.get_logger().debug(
            '[%s] pos=(%.2f,%.2f) d=%.2f blocked=%s omega=%.3f'
            % (self._name, x, y, d, self._blocked, omega)
        )

    def _stop(self):
        self._cmd.publish(Twist())

    def _pub_status(self, s):
        msg = String(); msg.data = s
        self._status.publish(msg)

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
