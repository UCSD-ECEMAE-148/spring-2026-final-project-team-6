#!/usr/bin/env python3
 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
 
import threading
import subprocess
import os
from ament_index_python.packages import get_package_share_directory
 
 
# ─────────────────────────────────────────────
#  CONFIG — change ALARM_MODE to switch sounds
#  Options: 'FAHHH', 'WARNING', 'RUN'
# ─────────────────────────────────────────────
 
ALARM_MODE = 'RUN'
 
ALARM_SOUNDS = {
    'FAHHH'  : os.path.join(get_package_share_directory('patrol'), 'patrol', 'fahhhhh.wav'),
    'WARNING': os.path.join(get_package_share_directory('patrol'), 'patrol', 'attentionWarning.wav'),
    'RUN'    : os.path.join(get_package_share_directory('patrol'), 'patrol', 'awolnation-run.wav'),
}
 
ALARM_SOUND = ALARM_SOUNDS[ALARM_MODE]
 
ALARM_COOLDOWN_SEC = 3.0
 
 
# ─────────────────────────────────────────────
#  NODE
# ─────────────────────────────────────────────
 
class ThermalAlarmTestNode(Node):
 
    def __init__(self):
        super().__init__('thermal_alarm_test_node')
        self.get_logger().info('Thermal Alarm Test Node started!')
        self.get_logger().info(f'Alarm mode : {ALARM_MODE}')
        self.get_logger().info(f'Alarm sound: {ALARM_SOUND}')
        self.get_logger().info(f'Listening on /thermal/detection...')
 
        # Check if alarm sound file exists
        if not os.path.exists(ALARM_SOUND):
            self.get_logger().warn(
                f'Alarm sound file not found: {ALARM_SOUND}\n'
                f'Terminal warning will still work without the sound file.'
            )
 
        # Check if aplay is available
        try:
            subprocess.run(['aplay', '--version'], capture_output=True, check=True)
            self.speaker_available = True
            self.get_logger().info('Speaker ready — using aplay on Jabra SPEAK 510!')
        except Exception:
            self.speaker_available = False
            self.get_logger().warn('aplay not available — terminal warning only.')
 
        # State
        self.last_alarm_time = 0.0
        self.alarm_playing   = False
 
        # ── Subscriber ──
        self.detection_sub = self.create_subscription(
            String,
            '/thermal/detection',
            self.detection_callback,
            10
        )
 
 
    def detection_callback(self, msg):
        """
        Called every time /thermal/detection publishes.
        Format: "present:0.9532" or "empty:0.8000" or "uncertain:0.4500"
        """
        try:
            parts      = msg.data.split(':')
            label      = parts[0]
            confidence = float(parts[1])
        except Exception:
            self.get_logger().warn(f'Could not parse message: {msg.data}')
            return
 
        # ── Person detected ──
        if label == 'present':
            self.get_logger().warn(
                f'PERSON DETECTED | Confidence: {confidence:.2%}'
            )
            self.trigger_alarm()
 
        # ── Uncertain ──
        elif label == 'uncertain':
            self.get_logger().info(
                f'Uncertain detection ({confidence:.2%}) — no alarm',
                throttle_duration_sec=2.0
            )
 
        # ── Clear ──
        else:
            self.get_logger().info(
                f'Clear | Confidence: {confidence:.2%}',
                throttle_duration_sec=2.0
            )
 
 
    def trigger_alarm(self):
        """
        Play alarm sound in a background thread using aplay.
        Respects cooldown to prevent spam.
        """
        import time
        now = time.time()
 
        # Check cooldown
        if now - self.last_alarm_time < ALARM_COOLDOWN_SEC:
            return
 
        if self.alarm_playing:
            return
 
        self.last_alarm_time = now
 
        # Terminal warning
        self.get_logger().warn('=' * 50)
        self.get_logger().warn(f'  ⚠  ALARM: PERSON DETECTED [{ALARM_MODE}]  ⚠')
        self.get_logger().warn('=' * 50)
 
        # Play sound in background thread using aplay
        if self.speaker_available and os.path.exists(ALARM_SOUND):
            def play():
                self.alarm_playing = True
                try:
                    subprocess.run(
                        ['aplay', '-D', 'plughw:2,0', ALARM_SOUND],
                        capture_output=True
                    )
                except Exception as e:
                    self.get_logger().error(f'Failed to play sound: {e}')
                finally:
                    self.alarm_playing = False
 
            threading.Thread(target=play, daemon=True).start()
 
 
# ─────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────
 
def main(args=None):
    rclpy.init(args=args)
    node = ThermalAlarmTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
