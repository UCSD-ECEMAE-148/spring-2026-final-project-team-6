#!/usr/bin/env python3
"""
record_checkpoints_node.py
--------------------------
Drive the car manually and press buttons to record checkpoints.

CONTROLS:
  X button (2) — drop a NAV waypoint      (drive-through, no stop)
  B button (1) — drop a CORRECT waypoint  (slow down + lidar drift check)
  Y button (3) — save CSV and exit

OUTPUT:
  src/patrol/maps/checkpoints.csv
  Columns: index, x, y, heading_deg, type
  type: 'nav' or 'correct'

TIPS:
  - Start at position 0 and drop a CORRECT checkpoint (B) there
  - Drop NAV checkpoints (X) every 1-2m along the straight sections
  - Drop CORRECT checkpoints (B) every 5m or wherever you want drift correction
  - Drive the U-turn and drop NAV checkpoints through the arc
  - End back near position 0 and drop a final NAV checkpoint (X)
  - Press Y to save

TOPICS:
  Subscribes:
    /odom  (nav_msgs/Odometry)
    /joy   (sensor_msgs/Joy)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

import csv
import math
import os
import time


# ── Config ────────────────────────────────────────────────────────────────────
NAV_BUTTON     = 2   # X — nav waypoint
CORRECT_BUTTON = 1   # B — correction checkpoint
SAVE_BUTTON    = 3   # Y — save and exit

OUTPUT_DIR  = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', '..', '..', 'src', 'patrol', 'maps'
))
OUTPUT_FILE = os.path.join(OUTPUT_DIR, 'checkpoints.csv')
# ─────────────────────────────────────────────────────────────────────────────


def quaternion_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


class RecordCheckpointsNode(Node):

    def __init__(self):
        super().__init__('record_checkpoints_node')

        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0
        self.checkpoints = []

        # Button debounce
        self.prev_nav     = False
        self.prev_correct = False
        self.prev_save    = False

        os.makedirs(OUTPUT_DIR, exist_ok=True)

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Joy,      '/joy',  self.joy_cb,  10)

        self.get_logger().info('=' * 54)
        self.get_logger().info('  CHECKPOINT RECORDER')
        self.get_logger().info('  X → nav waypoint      (drive-through)')
        self.get_logger().info('  B → correction point  (lidar drift check)')
        self.get_logger().info('  Y → save and exit')
        self.get_logger().info(f'  Output → {OUTPUT_FILE}')
        self.get_logger().info('=' * 54)
        self.get_logger().info('TIP: Start with B at position 0, end near start with X')

    def odom_cb(self, msg):
        self.current_x   = msg.pose.pose.position.x
        self.current_y   = msg.pose.pose.position.y
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def joy_cb(self, msg):
        if len(msg.buttons) <= max(NAV_BUTTON, CORRECT_BUTTON, SAVE_BUTTON):
            return

        nav     = bool(msg.buttons[NAV_BUTTON])
        correct = bool(msg.buttons[CORRECT_BUTTON])
        save    = bool(msg.buttons[SAVE_BUTTON])

        # Rising edge only
        if nav and not self.prev_nav:
            self._drop('nav')
        if correct and not self.prev_correct:
            self._drop('correct')
        if save and not self.prev_save:
            self._save_and_exit()

        self.prev_nav     = nav
        self.prev_correct = correct
        self.prev_save    = save

    def _drop(self, checkpoint_type):
        idx = len(self.checkpoints)
        cp  = {
            'index':       idx,
            'x':           round(self.current_x,   4),
            'y':           round(self.current_y,   4),
            'heading_deg': round(self.current_yaw, 2),
            'type':        checkpoint_type,
        }
        self.checkpoints.append(cp)

        tag = 'NAV    ' if checkpoint_type == 'nav' else 'CORRECT'
        self.get_logger().info(
            f'  [{idx:03d}] {tag} | '
            f'x={cp["x"]:7.3f}  y={cp["y"]:7.3f}  '
            f'heading={cp["heading_deg"]:6.1f}°'
        )

    def _save_and_exit(self):
        if not self.checkpoints:
            self.get_logger().warn('No checkpoints recorded — nothing to save.')
            rclpy.shutdown()
            return

        nav_count     = sum(1 for c in self.checkpoints if c['type'] == 'nav')
        correct_count = sum(1 for c in self.checkpoints if c['type'] == 'correct')

        try:
            with open(OUTPUT_FILE, 'w', newline='') as f:
                writer = csv.DictWriter(
                    f, fieldnames=['index', 'x', 'y', 'heading_deg', 'type']
                )
                writer.writeheader()
                writer.writerows(self.checkpoints)

            self.get_logger().info('=' * 54)
            self.get_logger().info(
                f'  SAVED {len(self.checkpoints)} checkpoints '
                f'({nav_count} nav, {correct_count} correct)'
            )
            self.get_logger().info(f'  → {OUTPUT_FILE}')
            self.get_logger().info('=' * 54)

        except Exception as e:
            self.get_logger().error(f'Failed to save: {e}')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RecordCheckpointsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
