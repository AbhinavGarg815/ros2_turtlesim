# Turtlesim Assignment

## 1. Build and Run

### Prerequisites

ROS 2 Humble (or later) with turtlesim and tf2

### Build

```bash
cp -r turtle_interfaces turtle_odometry turtle_autonomy ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select turtle_interfaces
colcon build --packages-select turtle_odometry_pkg turtle_autonomy_pkg
source install/setup.bash
```

### Run odometry only (Section 1 & 2)

```bash
ros2 launch turtle_odometry turtle_odom.launch.py
```

### Send a navigation goal

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{pose: {position: {x: 8.0, y: 8.0}}}" --once

# From RViz2, use the "2D Nav Goal" button (Fixed Frame = map)
rviz2
```

### Run Autonomous (Section 3)

```bash
ros2 launch turtle_autonomy turtle_autonomy.launch.py
```
### Send a navigation goal

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{pose: {position: {x: 2.0, y: 2.0}}}" --once
```

---

## 2. Approach

### Section 1 & 2

In this part i simply created a node that publishes to /turtle1/odom (type: nav_msgs/msg/Odometry) and subscribes to /turtle1/pose (type: turtlesim/msg/Pose).

In section-2, broadcasts a dynamic TF `odom → base_link` on every pose update and a one-time static TF `map → odom` with a translation of
  `(x=5.5, y=5.5)` placing the odom frame origin at the centre of the
  turtlesim world `[0,11] × [0,11]`

### Section 3

**Approach:**

1. Read current pose `(x, y, θ)` from `/turtle1/pose`
2. Read goal `(gx, gy)` from `/goal_pose`
3. Sweep 61 candidate yaw rates `ω ∈ [−4.0, +4.0]` rad/s
4. For each `ω`, simulate 20 steps of the unicycle model propagating `θ`
   at every step so different candidates genuinely diverge
5. Score each trajectory and select the `ω` with the lowest cost
6. Publish `Twist(linear.x=v, angular.z=ω_best)` to `/turtle1/cmd_vel`
7. Stop when `distance(pose, goal) < goal_tolerance` (default 0.3 m)
