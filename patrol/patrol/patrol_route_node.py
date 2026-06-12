#!/usr/bin/env python3
"""
patrol_route_node.py
--------------------
Loads checkpoints.csv and follows the path in a continuous loop.

Checkpoint types:
  nav     — drive through at cruise speed, just steer toward it
  correct — slow down, read lidar L/R, apply correction if drift > threshold,
             then continue — never fully stops

Loop behavior:
  Drives checkpoint 0 → 1 → 2 → ... → N → back to 0 → repeat forever
  The U-turn is part of the recorded path (drove it during recording)

STATE MACHINE:
  LOAD_FILE      → read CSV, fail loudly if missing
  WAIT_FOR_ODOM  → wait for first odom fix
  DRIVE          → steering toward next checkpoint at cruise speed
  OBSTACLE_WAIT  → obstacle in front, hold until clear
  CORRECT        → at a 'correct' checkpoint: slow + read lidar + steer fix
  DONE           → CSV missing or empty

TOPICS:
  Subscribes:  /odom (Odometry)  /scan (LaserScan)
  Publishes:   /drive (AckermannDriveStamped)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

import csv
import math
import os
import time
from enum import Enum, auto
from ament_index_python.packages import get_package_share_directory


# ─────────────────────────────────────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────────────────────────────────────

# ── Checkpoint file ──
CHECKPOINT_FILE = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', '..', '..', 'src', 'patrol', 'maps', 'checkpoints.csv'
))

# ── Driving ──
CRUISE_SPEED      = 0.5    # m/s — normal driving speed
CORRECT_SPEED     = 0.25   # m/s — speed during correction checkpoint
CHECKPOINT_RADIUS = 0.4    # metres — distance to count as "reached"

# ── Steering toward waypoint (pure pursuit style) ──
LOOKAHEAD_GAIN    = 1.0    # how aggressively to steer toward waypoint
MAX_STEERING      = 0.4    # radians — clamp

# ── Lidar convention: 180°=FRONT  270°=LEFT  90°=RIGHT ──
FRONT_DEG         = 180
FRONT_CONE_DEG    = 20
LEFT_MIN_DEG      = 240
LEFT_MAX_DEG      = 300
RIGHT_MIN_DEG     = 60
RIGHT_MAX_DEG     = 120

# ── Drift correction (at 'correct' checkpoints) ──
DRIFT_THRESHOLD   = 0.08   # metres — ignore drift smaller than this
CORRECT_STEERING  = 0.20   # radians — correction steering angle
CORRECT_DURATION  = 0.5    # seconds — how long to hold correction
STRAIGHTEN_DUR    = 0.2    # seconds — straighten after correction

# ── Obstacle ──
OBSTACLE_DIST     = 0.4    # metres — stop if closer
OBSTACLE_CLEAR    = 0.6    # metres — resume when clears to this

# ─────────────────────────────────────────────────────────────────────────────


class State(Enum):
    LOAD_FILE     = auto()
    WAIT_FOR_ODOM = auto()
    DRIVE         = auto()
    OBSTACLE_WAIT = auto()
    CORRECT       = auto()   # slowing + reading lidar
    CORRECTING    = auto()   # applying steering pulse
    STRAIGHTEN    = auto()   # wheels back to center
    DONE          = auto()


def range_at_angle(ranges, angle_min, angle_inc, target_deg, cone_deg):
    target_rad = math.radians(target_deg)
    cone_rad   = math.radians(cone_deg)
    valid = [
        r for i, r in enumerate(ranges)
        if not (r == 0.0 or math.isinf(r) or math.isnan(r))
        and abs((angle_min + i * angle_inc) - target_rad) <= cone_rad
    ]
    return min(valid) if valid else float('inf')


def mean_range(ranges, angle_min, angle_inc, deg_min, deg_max):
    rad_min = math.radians(deg_min)
    rad_max = math.radians(deg_max)
    valid = [
        r for i, r in enumerate(ranges)
        if not (r == 0.0 or math.isinf(r) or math.isnan(r))
        and rad_min <= (angle_min + i * angle_inc) <= rad_max
    ]
    return sum(valid) / len(valid) if valid else float('inf')


class PatrolRouteNode(Node):

    def __init__(self):
        super().__init__('patrol_route_node')

        # ── Checkpoints — list of dicts with x, y, type ───────────────────
        self.checkpoints    = []
        self.checkpoint_idx = 0
        self.lap_count      = 0

        # ── Odometry ─────────────────────────────────────────────────────
        self.current_x   = None
        self.current_y   = None
        self.current_yaw = 0.0

        # ── Lidar ─────────────────────────────────────────────────────────
        self.front_dist = float('inf')
        self.left_dist  = float('inf')
        self.right_dist = float('inf')

        # ── Correction state ──────────────────────────────────────────────
        self.correction_sign  = 0.0
        self.state_entry_time = time.time()

        # ── State ─────────────────────────────────────────────────────────
        self.state = State.LOAD_FILE

        # ── ROS ───────────────────────────────────────────────────────────
        self.create_subscription(Odometry,  '/odom', self.odom_cb,  10)
        self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_timer(0.05, self.loop)   # 20 Hz

        self.get_logger().info('Patrol Route Node starting...')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def lidar_cb(self, msg):
        self.front_dist = range_at_angle(
            msg.ranges, msg.angle_min, msg.angle_increment,
            FRONT_DEG, FRONT_CONE_DEG)
        self.left_dist = mean_range(
            msg.ranges, msg.angle_min, msg.angle_increment,
            LEFT_MIN_DEG, LEFT_MAX_DEG)
        self.right_dist = mean_range(
            msg.ranges, msg.angle_min, msg.angle_increment,
            RIGHT_MIN_DEG, RIGHT_MAX_DEG)

    # ── Main loop ─────────────────────────────────────────────────────────

    def loop(self):
        now = time.time()
        s   = self.state

        if   s == State.LOAD_FILE:     self._load_file()
        elif s == State.WAIT_FOR_ODOM: self._wait_for_odom()
        elif s == State.DRIVE:         self._drive()
        elif s == State.OBSTACLE_WAIT: self._obstacle_wait()
        elif s == State.CORRECT:       self._correct_read(now)
        elif s == State.CORRECTING:    self._correct_apply(now)
        elif s == State.STRAIGHTEN:    self._straighten(now)
        elif s == State.DONE:          self.publish_stop()

    # ── State handlers ────────────────────────────────────────────────────

    def _load_file(self):
        self.publish_stop()

        # Try src path first, then installed share path
        candidates = [
            CHECKPOINT_FILE,
            os.path.join(
                get_package_share_directory('patrol'),
                'maps', 'checkpoints.csv'
            ),
        ]

        found = None
        for p in candidates:
            if os.path.exists(os.path.normpath(p)):
                found = os.path.normpath(p)
                break

        if found is None:
            self.get_logger().error(
                'checkpoints.csv not found!\n'
                'Record a path first:\n'
                '  ros2 run patrol record_checkpoints_node'
            )
            self._enter(State.DONE)
            return

        try:
            with open(found, 'r') as f:
                reader = csv.DictReader(f)
                self.checkpoints = [
                    {
                        'x':    float(row['x']),
                        'y':    float(row['y']),
                        'type': row['type'].strip(),   # 'nav' or 'correct'
                    }
                    for row in reader
                ]
        except Exception as e:
            self.get_logger().error(f'Failed to read CSV: {e}')
            self._enter(State.DONE)
            return

        if not self.checkpoints:
            self.get_logger().error('checkpoints.csv is empty.')
            self._enter(State.DONE)
            return

        nav_count     = sum(1 for c in self.checkpoints if c['type'] == 'nav')
        correct_count = sum(1 for c in self.checkpoints if c['type'] == 'correct')

        self.get_logger().info('=' * 54)
        self.get_logger().info(
            f'  Loaded {len(self.checkpoints)} checkpoints '
            f'({nav_count} nav, {correct_count} correct)'
        )
        self.get_logger().info(f'  From: {found}')
        self.get_logger().info('=' * 54)

        self._enter(State.WAIT_FOR_ODOM)

    def _wait_for_odom(self):
        self.publish_stop()
        if self.current_x is not None:
            self.get_logger().info(
                f'Odom ready ({self.current_x:.2f}, {self.current_y:.2f}) — starting.'
            )
            self._enter(State.DRIVE)

    def _drive(self):
        # ── Obstacle check ────────────────────────────────────────────────
        if self.front_dist < OBSTACLE_DIST:
            self.get_logger().warn(
                f'Obstacle at {self.front_dist:.2f}m — waiting.'
            )
            self._enter(State.OBSTACLE_WAIT)
            self.publish_stop()
            return

        cp = self.checkpoints[self.checkpoint_idx]
        tx, ty = cp['x'], cp['y']

        dist = math.sqrt(
            (self.current_x - tx) ** 2 +
            (self.current_y - ty) ** 2
        )

        # ── Checkpoint reached ────────────────────────────────────────────
        if dist < CHECKPOINT_RADIUS:
            cp_type = cp['type']
            self.get_logger().info(
                f'[{self.checkpoint_idx}] {cp_type.upper()} reached '
                f'(dist={dist:.2f}m)'
            )

            if cp_type == 'correct':
                self._enter(State.CORRECT)
            else:
                # nav — advance immediately, no stop
                self._advance()
            return

        # ── Steer toward checkpoint ───────────────────────────────────────
        angle_to_target = math.atan2(
            ty - self.current_y,
            tx - self.current_x
        )
        heading_error = angle_to_target - self.current_yaw

        # Normalize to [-pi, pi]
        while heading_error >  math.pi: heading_error -= 2 * math.pi
        while heading_error < -math.pi: heading_error += 2 * math.pi

        steering = float(
            max(-MAX_STEERING,
                min(MAX_STEERING, LOOKAHEAD_GAIN * heading_error))
        )

        self.get_logger().info(
            f'DRIVE [{self.checkpoint_idx}/{len(self.checkpoints)-1}] '
            f'{cp["type"].upper()} | dist={dist:.2f}m | '
            f'steer={math.degrees(steering):.1f}° | front={self.front_dist:.2f}m',
            throttle_duration_sec=0.4
        )
        self.publish_drive(CRUISE_SPEED, steering)

    def _obstacle_wait(self):
        self.publish_stop()
        if self.front_dist >= OBSTACLE_CLEAR:
            self.get_logger().info('Obstacle cleared — resuming.')
            self._enter(State.DRIVE)
        else:
            self.get_logger().info(
                f'Waiting... front={self.front_dist:.2f}m',
                throttle_duration_sec=1.0
            )

    def _correct_read(self, now):
        """
        Slow down and read lidar for CORRECT_DURATION seconds
        to get a stable average, then decide correction direction.
        """
        elapsed = now - self.state_entry_time
        self.publish_drive(CORRECT_SPEED, 0.0)   # keep moving slowly

        self.get_logger().info(
            f'CORRECT read | {elapsed:.1f}/{CORRECT_DURATION}s | '
            f'L={self.left_dist:.2f}m  R={self.right_dist:.2f}m',
            throttle_duration_sec=0.2
        )

        if elapsed >= CORRECT_DURATION:
            self._decide_correction()

    def _decide_correction(self):
        left_valid  = not math.isinf(self.left_dist)
        right_valid = not math.isinf(self.right_dist)

        if left_valid and right_valid:
            drift = self.left_dist - self.right_dist
            self.get_logger().info(
                f'Drift: L={self.left_dist:.2f}m  R={self.right_dist:.2f}m  '
                f'drift={drift:+.3f}m'
            )
            if abs(drift) < DRIFT_THRESHOLD:
                self.get_logger().info('Drift within threshold — continuing.')
                self._advance()
                return

            self.correction_sign = 1.0 if drift > 0 else -1.0
            direction = 'RIGHT' if self.correction_sign > 0 else 'LEFT'
            self.get_logger().info(
                f'Applying correction {direction} (drift={drift:+.3f}m)'
            )
            self._enter(State.CORRECTING)

        else:
            self.get_logger().warn('Wall readings invalid — skipping correction.')
            self._advance()

    def _correct_apply(self, now):
        """Apply steering correction pulse while still moving forward."""
        elapsed = now - self.state_entry_time
        if elapsed < CORRECT_DURATION:
            self.publish_drive(
                CORRECT_SPEED,
                self.correction_sign * CORRECT_STEERING
            )
        else:
            self._enter(State.STRAIGHTEN)

    def _straighten(self, now):
        """Brief straight run to finish correction arc."""
        elapsed = now - self.state_entry_time
        if elapsed < STRAIGHTEN_DUR:
            self.publish_drive(CORRECT_SPEED, 0.0)
        else:
            self.get_logger().info('Correction done — resuming cruise.')
            self._advance()

    # ── Helpers ───────────────────────────────────────────────────────────

    def _advance(self):
        """Move to next checkpoint, wrapping back to 0 at the end."""
        next_idx = self.checkpoint_idx + 1

        if next_idx >= len(self.checkpoints):
            # Completed one full loop
            self.lap_count      += 1
            self.checkpoint_idx  = 0
            self.get_logger().info('=' * 54)
            self.get_logger().info(
                f'  LAP {self.lap_count} complete — returning to checkpoint 0'
            )
            self.get_logger().info('=' * 54)
        else:
            self.checkpoint_idx = next_idx
            cp = self.checkpoints[self.checkpoint_idx]
            self.get_logger().info(
                f'Next → [{self.checkpoint_idx}] {cp["type"].upper()} '
                f'x={cp["x"]:.3f}  y={cp["y"]:.3f}'
            )

        self._enter(State.DRIVE)

    def _enter(self, new_state):
        self.get_logger().info(f'→ {new_state.name}')
        self.state            = new_state
        self.state_entry_time = time.time()

    def publish_drive(self, speed, steering):
        cmd = AckermannDriveStamped()
        cmd.header.stamp         = self.get_clock().now().to_msg()
        cmd.header.frame_id      = 'base_link'
        cmd.drive.speed          = float(speed)
        cmd.drive.steering_angle = float(steering)
        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        self.cmd_pub.publish(AckermannDriveStamped())


def main(args=None):
    rclpy.init(args=args)
    node = PatrolRouteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.get_logger().info('Patrol route node stopped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
