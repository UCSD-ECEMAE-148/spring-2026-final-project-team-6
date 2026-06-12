
#!/usr/bin/env python3

"""
simple_patrol_route_node.py
---------------------------

BEHAVIOR:
  1. Start driving immediately using AckermannDriveStamped via mux
  2. PID controller equalizes left and right wall distances
  3. When odometry reaches TARGET_DISTANCE → stop
  4. Stay stopped — reset manually (Ctrl+C and relaunch)
  5. Joystick can override at any time (higher mux priority)

TOPICS:
  Subscribes:
    /odom   (nav_msgs/Odometry)     — tracks distance traveled
    /scan   (sensor_msgs/LaserScan) — wall centering

  Publishes:
    /drive (ackermann_msgs/AckermannDriveStamped) — via ackermann_mux

TUNING:
  TARGET_DISTANCE — how far to drive in meters
  CRUISE_SPEED    — forward speed in m/s
  KP, KI, KD     — PID gains for wall centering
    KP: main correction strength
    KI: corrects accumulated drift over time
    KD: dampens oscillation

LIDAR ANGLES (lidar is flipped):
  front = 180°  right = 90°  left = 270°
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from collections import deque

import math


# ─────────────────────────────────────────────
#  CONFIG — tune these for your hallway
# ─────────────────────────────────────────────

TARGET_DISTANCE  = 22.5   # meters — how far to drive before stopping
CRUISE_SPEED     = 0.5    # m/s — forward speed

# PID gains — tune these
KP               = 0.21   # proportional — main correction
KI               = 0.0   # integral — corrects slow drift
KD               = 0.16   # derivative — dampens oscillation

# Limits
MAX_STEERING     = 0.4    # radians — max steering angle
INTEGRAL_LIMIT   = 1.0    # clamp integral to prevent windup

# Lidar config
SIDE_CONE_DEG    = 60     # degrees cone to sample side walls
FRONT_CONE_DEG   = 20     # degrees cone to check front
OBSTACLE_DIST    = 0.8    # meters — stop if obstacle ahead closer than this
BUFFER_SIZE      = 10     # rolling average window for wall distances


# ─────────────────────────────────────────────
#  HELPER
# ─────────────────────────────────────────────

def get_range_at_angle(ranges, angle_min, angle_inc, target_deg, cone_deg=10):
    """Get minimum range reading within a cone around target angle."""
    target_rad = math.radians(target_deg)
    cone_rad   = math.radians(cone_deg)
    min_range  = float('inf')
    for i, r in enumerate(ranges):
        if r == 0.0 or math.isinf(r) or math.isnan(r):
            continue
        angle = angle_min + i * angle_inc
        if abs(angle - target_rad) <= cone_rad:
            min_range = min(min_range, r)
    return min_range


# ─────────────────────────────────────────────
#  NODE
# ─────────────────────────────────────────────

class SimplePatrolRouteNode(Node):

    def __init__(self):
        super().__init__('simple_patrol_route_node')
        self.get_logger().info('Simple Patrol Route Node started!')
        self.get_logger().info(f'Target distance: {TARGET_DISTANCE}m')
        self.get_logger().info(f'Cruise speed: {CRUISE_SPEED} m/s')
        self.get_logger().info(f'PID gains — KP: {KP}  KI: {KI}  KD: {KD}')
        self.get_logger().info('Driving immediately — will stop at target distance.')
        self.get_logger().info('Joystick can override at any time.')

        # ── State ──
        self.start_x           = None
        self.start_y           = None
        self.distance_traveled = 0.0
        self.stopped           = False
        self.avoiding          = False
        # ── Lidar data ──
        self.left_dist  = float('inf')
        self.right_dist = float('inf')
        self.front_dist = float('inf')

        # ── Rolling average buffers ──
        self.left_buffer  = deque(maxlen=BUFFER_SIZE)
        self.right_buffer = deque(maxlen=BUFFER_SIZE)

        # ── PID state ──
        self.prev_error  = 0.0
        self.integral    = 0.0
        self.prev_time   = self.get_clock().now()

        # ── Subscribers ──
        self.odom_sub  = self.create_subscription(Odometry,  '/odom',  self.odom_callback,  10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan',  self.lidar_callback, 10)

        # ── Publisher ──
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # ── Control loop at 20Hz ──
        self.timer = self.create_timer(0.05, self.control_loop)


    # ──────────────────────────────────────────
    #  CALLBACKS
    # ──────────────────────────────────────────

    def odom_callback(self, msg):
        """Track distance traveled from start position."""
        self.get_logger().info('Odom received!', throttle_duration_sec=5.0)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.get_logger().info(f'Start position: ({x:.2f}, {y:.2f})')
            return

        self.distance_traveled = math.sqrt(
            (x - self.start_x) ** 2 +
            (y - self.start_y) ** 2
        )


    def lidar_callback(self, msg):
        """Update wall distances using rolling average."""
        front = get_range_at_angle(
            msg.ranges, msg.angle_min, msg.angle_increment, 180, FRONT_CONE_DEG)
        left  = get_range_at_angle(
            msg.ranges, msg.angle_min, msg.angle_increment, 270, SIDE_CONE_DEG)
        right = get_range_at_angle(
            msg.ranges, msg.angle_min, msg.angle_increment, 90, SIDE_CONE_DEG)

        self.front_dist = front

        if not math.isinf(left):
            self.left_buffer.append(left)
        if not math.isinf(right):
            self.right_buffer.append(right)

        self.left_dist  = sum(self.left_buffer)  / len(self.left_buffer)  if self.left_buffer  else float('inf')
        self.right_dist = sum(self.right_buffer) / len(self.right_buffer) if self.right_buffer else float('inf')


    # ──────────────────────────────────────────
    #  PID WALL CENTERING
    # ──────────────────────────────────────────

    def pid_steering(self):
        """
        PID controller to equalize left and right wall distances.

        Error = right_dist - left_dist
          positive error → right wall is farther → steer right (negative steering)
          negative error → left wall is farther  → steer left  (positive steering)

        Returns steering angle in radians.
        """
        left_valid  = not math.isinf(self.left_dist)
        right_valid = not math.isinf(self.right_dist)

        # Calculate error based on available walls
        if left_valid and right_valid:
            # Both walls — equalize distances
            error = self.right_dist - self.left_dist
        elif left_valid:
            # Only left wall — maintain safe distance from it
            error = -(self.left_dist - 0.5)
        elif right_valid:
            # Only right wall — maintain safe distance from it
            error = self.right_dist - 0.5
        else:
            # No walls — go straight, reset PID
            self.integral   = 0.0
            self.prev_error = 0.0
            return 0.0

        # Time delta
        now      = self.get_clock().now()
        dt       = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        if dt <= 0.0:
            dt = 0.05

        # PID terms
        proportional = KP * error
        self.integral = max(-INTEGRAL_LIMIT,
                        min( INTEGRAL_LIMIT,
                             self.integral + error * dt))
        integral     = KI * self.integral
        derivative   = KD * (error - self.prev_error) / dt

        self.prev_error = error

        steering = proportional + integral + derivative
        return max(-MAX_STEERING, min(MAX_STEERING, steering))


    # ──────────────────────────────────────────
    #  CONTROL LOOP
    # ──────────────────────────────────────────

    def control_loop(self):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        # ── Already stopped ──
        if self.stopped:
            self.cmd_pub.publish(cmd)
            return

        # ── Target reached ──
        if self.start_x is not None and self.distance_traveled >= TARGET_DISTANCE:
            self.stopped = True
            self.get_logger().info('=' * 50)
            self.get_logger().info(
                f'TARGET REACHED — {self.distance_traveled:.2f}m traveled'
            )
            self.get_logger().info('Stopped. Reset manually to run again.')
            self.get_logger().info('=' * 50)
            self.cmd_pub.publish(cmd)
            return

        # ── Obstacle ahead — emergency stop ──
        if self.front_dist < OBSTACLE_DIST:
            self.avoiding = True

        if self.avoiding:
            if self.front_dist >= OBSTACLE_DIST:
                self.avoiding = False
                self.integral = 0.0        # reset PID so no jerk when resuming
                self.prev_error = 0.0
            else:
                if self.left_dist > self.right_dist:
                    cmd.drive.steering_angle = MAX_STEERING    # steer left
                else:
                    cmd.drive.steering_angle = -MAX_STEERING   # steer right (default)
                cmd.drive.speed = CRUISE_SPEED * 0.5
                self.cmd_pub.publish(cmd)
                return


        # ── PID wall centering ──
        steering = self.pid_steering()

        cmd.drive.speed          = CRUISE_SPEED
        cmd.drive.steering_angle = steering

        if self.start_x is None:
            self.get_logger().info(
                f'Driving — waiting for first odom... | '
                f'L: {self.left_dist:.2f}m | R: {self.right_dist:.2f}m | '
                f'Error: {(self.right_dist - self.left_dist):.2f}m | '
                f'Steer: {math.degrees(steering):.1f}°',
                throttle_duration_sec=1.0
            )
        else:
            self.get_logger().info(
                f'Dist: {self.distance_traveled:.2f}m / {TARGET_DISTANCE}m | '
                f'L: {self.left_dist:.2f}m | R: {self.right_dist:.2f}m | '
                f'Error: {(self.right_dist - self.left_dist):.2f}m | '
                f'Steer: {math.degrees(steering):.1f}°',
                throttle_duration_sec=0.5
            )

        self.cmd_pub.publish(cmd)


# ─────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SimplePatrolRouteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = AckermannDriveStamped()
        try:
            node.cmd_pub.publish(stop)
        except Exception:
            pass
        node.get_logger().info('Simple patrol route node stopped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
