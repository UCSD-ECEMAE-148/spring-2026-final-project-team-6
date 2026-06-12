#!/usr/bin/env python3

"""
patrol_node.py
--------------
BEHAVIOR:
  1. Drive straight down hallway using PID wall centering
  2. Person detected → stop, sound alarm, wait for alarm to finish
  3. If person still present after alarm → replay alarm
  4. Person clears → wait RESUME_DELAY_SEC → resume driving
  5. Reached TARGET_DISTANCE → stop permanently, no more alarms
  6. Joystick can override at any time

REQUIREMENTS:
  - mqtt_bridge node running (publishes /thermal/detection)
  - USB speaker plugged in (Jabra SPEAK 510, card 2)
  - VESC + lidar running

TOPICS:
  Subscribes:
    /odom               (nav_msgs/Odometry)
    /scan               (sensor_msgs/LaserScan)
    /thermal/detection  (std_msgs/String)  "present:0.95" or "empty:0.80"

  Publishes:
    /drive              (ackermann_msgs/AckermannDriveStamped)
    /patrol/alarm       (std_msgs/String)

TUNING:
  TARGET_DISTANCE — hallway length in meters
  CRUISE_SPEED    — forward speed m/s
  KP, KI, KD     — PID gains for wall centering
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from collections import deque

import math
import threading
import subprocess
import time
import os


# ─────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────

# ── Patrol ──
TARGET_DISTANCE      = 22.0   # meters
CRUISE_SPEED         = 0.5    # m/s

# ── PID ──
KP                   = 0.18
KI                   = 0.0
KD                   = 0.25
MAX_STEERING         = 0.4    # radians
INTEGRAL_LIMIT       = 1.0

# ── Lidar ──
SIDE_CONE_DEG        = 60
FRONT_CONE_DEG       = 20
OBSTACLE_DIST        = 0.4    # meters
BUFFER_SIZE          = 10

# ── Thermal alarm ──
CONFIDENCE_THRESHOLD = 0.5
ALARM_COOLDOWN_SEC   = 0.5    # short — allows replay after alarm finishes
RESUME_DELAY_SEC     = 2.0    # seconds after person clears before resuming

# ── Sound ──
ALSA_DEVICE          = 'plughw:2,0'
SOUND_DIR            = os.path.join(get_package_share_directory('patrol'), 'patrol')
ALARM_SOUND          = os.path.join(SOUND_DIR, 'attentionWarning.wav')
RESUME_SOUND         = os.path.join(SOUND_DIR, 'clear.wav')


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

class PatrolNode(Node):

    def __init__(self):
        super().__init__('patrol_node')
        self.get_logger().info('Patrol Node started!')
        self.get_logger().info(f'Target distance: {TARGET_DISTANCE}m')
        self.get_logger().info(f'Cruise speed: {CRUISE_SPEED} m/s')
        self.get_logger().info(f'PID — KP:{KP} KI:{KI} KD:{KD}')
        self.get_logger().info(f'Alarm sound: {ALARM_SOUND}')
        self.get_logger().info(f'Resume sound: {RESUME_SOUND}')

        # ── Patrol state ──
        self.start_x           = None
        self.start_y           = None
        self.distance_traveled = 0.0
        self.stopped           = False

        # ── Thermal state ──
        self.person_detected   = False
        self.alarm_playing     = False
        self.last_alarm_time   = 0.0
        self.person_clear_time = None

        # ── Lidar data ──
        self.left_dist         = float('inf')
        self.right_dist        = float('inf')
        self.front_dist        = float('inf')
        self.left_buffer       = deque(maxlen=BUFFER_SIZE)
        self.right_buffer      = deque(maxlen=BUFFER_SIZE)

        # ── PID state ──
        self.prev_error        = 0.0
        self.integral          = 0.0
        self.prev_time         = self.get_clock().now()

        # ── Subscribers ──
        self.odom_sub    = self.create_subscription(Odometry,  '/odom',              self.odom_callback,    10)
        self.lidar_sub   = self.create_subscription(LaserScan, '/scan',              self.lidar_callback,   10)
        self.thermal_sub = self.create_subscription(String,    '/thermal/detection', self.thermal_callback, 10)

        # ── Publishers ──
        self.cmd_pub   = self.create_publisher(AckermannDriveStamped, '/drive',       10)
        self.alarm_pub = self.create_publisher(String,                '/patrol/alarm', 10)

        # ── Control loop at 20Hz ──
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Ready — driving immediately.')


    # ──────────────────────────────────────────
    #  CALLBACKS
    # ──────────────────────────────────────────

    def odom_callback(self, msg):
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


    def thermal_callback(self, msg):
        """Parse thermal detection and trigger alarm if person detected."""
        if self.stopped:
            return

        try:
            parts      = msg.data.split(':')
            label      = parts[0]
            confidence = float(parts[1])
        except Exception:
            self.get_logger().warn(f'Could not parse thermal message: {msg.data}')
            return

        if label == 'present' and confidence >= CONFIDENCE_THRESHOLD:
            if not self.person_detected:
                self.get_logger().warn(f'PERSON DETECTED | Confidence: {confidence:.2%}')
                self.person_detected   = True
                self.person_clear_time = None
            # replay after previous finishes
            self._trigger_alarm()

        else:
            if self.person_detected:
                self.get_logger().info('Person cleared — resuming after delay...')
                self.person_detected   = False
                self.person_clear_time = time.time()


    # ──────────────────────────────────────────
    #  ALARM
    # ──────────────────────────────────────────

    def _trigger_alarm(self):
        """
        Play alarm sound in background thread.
        - Waits for current alarm to finish before replaying
        - Short cooldown prevents rapid re-triggering
        """
        now = time.time()

        # Wait for current alarm to finish before replaying
        if self.alarm_playing:
            return

        # Short cooldown between alarms
        if now - self.last_alarm_time < ALARM_COOLDOWN_SEC:
            return

        self.last_alarm_time = now

        self.get_logger().warn('=' * 50)
        self.get_logger().warn('  ⚠  ALARM: PERSON DETECTED  ⚠')
        self.get_logger().warn('=' * 50)

        alarm_msg      = String()
        alarm_msg.data = 'PERSON_DETECTED'
        self.alarm_pub.publish(alarm_msg)

        def play():
            self.alarm_playing = True
            try:
                subprocess.run(
                    ['aplay', '-D', ALSA_DEVICE, ALARM_SOUND],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
            except Exception as e:
                self.get_logger().error(f'Failed to play alarm: {e}')
            finally:
                self.alarm_playing = False

        threading.Thread(target=play, daemon=True).start()


    def _play_resume_sound(self):
        """Play resume sound in background thread."""
        def play():
            try:
                subprocess.run(
                    ['aplay', '-D', ALSA_DEVICE, RESUME_SOUND],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
            except Exception as e:
                self.get_logger().error(f'Failed to play resume sound: {e}')

        threading.Thread(target=play, daemon=True).start()


    # ──────────────────────────────────────────
    #  PID WALL CENTERING
    # ──────────────────────────────────────────

    def pid_steering(self):
        left_valid  = not math.isinf(self.left_dist)
        right_valid = not math.isinf(self.right_dist)

        if left_valid and right_valid:
            error = self.right_dist - self.left_dist
        elif left_valid:
            error = -(self.left_dist - 0.5)
        elif right_valid:
            error = self.right_dist - 0.5
        else:
            self.integral   = 0.0
            self.prev_error = 0.0
            return 0.0

        now  = self.get_clock().now()
        dt   = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now
        if dt <= 0.0:
            dt = 0.05

        proportional  = KP * error
        self.integral = max(-INTEGRAL_LIMIT,
                        min( INTEGRAL_LIMIT,
                             self.integral + error * dt))
        integral      = KI * self.integral
        derivative    = KD * (error - self.prev_error) / dt

        self.prev_error = error

        steering = proportional + integral + derivative
        return max(-MAX_STEERING, min(MAX_STEERING, steering))


    # ──────────────────────────────────────────
    #  CONTROL LOOP
    # ──────────────────────────────────────────

    def control_loop(self):
        cmd = AckermannDriveStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        # ── Permanently stopped at target ──
        if self.stopped:
            self.cmd_pub.publish(cmd)
            return

        # ── Target distance reached ──
        if self.start_x is not None and self.distance_traveled >= TARGET_DISTANCE:
            self.stopped = True
            self.get_logger().info('=' * 50)
            self.get_logger().info(f'TARGET REACHED — {self.distance_traveled:.2f}m')
            self.get_logger().info('Stopped. No more alarms. Reset manually to run again.')
            self.get_logger().info('=' * 50)
            self.cmd_pub.publish(cmd)
            return

        # ── Person detected OR alarm still playing — stay stopped ──
        if self.person_detected or self.alarm_playing:
            self.get_logger().warn(
                'Stopped — person detected!' if self.person_detected else 'Stopped — alarm playing!',
                throttle_duration_sec=1.0
            )
            self.cmd_pub.publish(cmd)
            return

        # ── Person just cleared — wait resume delay ──
        if self.person_clear_time is not None:
            if time.time() - self.person_clear_time < RESUME_DELAY_SEC:
                self.get_logger().info(
                    'Waiting before resuming...',
                    throttle_duration_sec=0.5
                )
                self.cmd_pub.publish(cmd)
                return
            else:
                self.get_logger().info('Resuming patrol!')
                self._play_resume_sound()
                self.person_clear_time = None
                # Reset PID to avoid integral windup from standing still
                self.integral   = 0.0
                self.prev_error = 0.0

        # ── Obstacle ahead ──
        if self.front_dist < OBSTACLE_DIST:
            self.get_logger().warn(
                f'Obstacle ahead ({self.front_dist:.2f}m) — stopping!',
                throttle_duration_sec=1.0
            )
            self.cmd_pub.publish(cmd)
            return

        # ── Normal driving ──
        if self.start_x is None:
            self.get_logger().info(
                'Driving — waiting for first odom...',
                throttle_duration_sec=1.0
            )

        steering = self.pid_steering()
        cmd.drive.speed          = CRUISE_SPEED
        cmd.drive.steering_angle = steering

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
    node = PatrolNode()
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
        node.get_logger().info('Patrol node stopped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
