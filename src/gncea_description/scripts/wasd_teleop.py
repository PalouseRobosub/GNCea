#!/usr/bin/env python3
import sys, termios, tty, select, threading, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

HELP = """\
Controls (body frame, timed pulses):
  Translation  : W/S (+/-X), A/D (+/-Y), E/Q (+/-Z)
  Rotation     : I/K (+/-Pitch), J/L (+/-Yaw), U/O (+/-Roll)
  Space        : zero all (force & torque)

Parameters (ros2 run ... --ros-args -p name:=value):
  force_topic      (string)  default: /auve1/force_body
  torque_topic     (string)  default: /auve1/torque_body
  force            (double)  default: 100.0
  torque           (double)  default: 50.0
  rate_hz          (double)  default: 60.0
  pulse_ms         (int)     default: 200     # how long a force pulse lasts
  torque_pulse_ms  (int)     default: 200     # how long a torque pulse lasts
  pulse_decay      (double)  default: 1.0     # 1.0=no decay during pulse
  torque_decay     (double)  default: 1.0
  deadman_ms       (int)     default: 500     # if no pub in this time, auto-zero
"""

def _getch(timeout=0.0):
    """Non-blocking single-char read with timeout (seconds). Returns None if no key."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

class WasdTeleop(Node):
    """
    Timed-pulse teleop: each keypress applies a pulse of force/torque for N ms,
    then auto-zeros that axis. Publishes Vector3 on force_topic and torque_topic.
    """
    def __init__(self):
        super().__init__('wasd_teleop')

        # ---- Parameters ----
        self.declare_parameter('force_topic', '/auve1/force_body')
        self.declare_parameter('torque_topic', '/auve1/torque_body')
        self.declare_parameter('force', 100.0)
        self.declare_parameter('torque', 50.0)
        self.declare_parameter('rate_hz', 60.0)
        self.declare_parameter('pulse_ms', 200)
        self.declare_parameter('torque_pulse_ms', 200)
        self.declare_parameter('pulse_decay', 1.0)      # decay during pulse
        self.declare_parameter('torque_decay', 1.0)
        self.declare_parameter('deadman_ms', 500)

        self.force_topic     = self.get_parameter('force_topic').get_parameter_value().string_value
        self.torque_topic    = self.get_parameter('torque_topic').get_parameter_value().string_value
        self.force_mag       = float(self.get_parameter('force').value)
        self.torque_mag      = float(self.get_parameter('torque').value)
        self.rate_hz         = max(1.0, float(self.get_parameter('rate_hz').value))
        self.pulse_ms        = int(self.get_parameter('pulse_ms').value)
        self.torque_pulse_ms = int(self.get_parameter('torque_pulse_ms').value)
        self.pulse_decay     = float(self.get_parameter('pulse_decay').value)
        self.torque_decay    = float(self.get_parameter('torque_decay').value)
        self.deadman_ms      = int(self.get_parameter('deadman_ms').value)

        qos = 10
        self.pub_force  = self.create_publisher(Vector3, self.force_topic, qos)
        self.pub_torque = self.create_publisher(Vector3, self.torque_topic, qos)

        self.get_logger().info(f'Publishing force  Vector3 on "{self.pub_force.topic_name}"')
        self.get_logger().info(f'Publishing torque Vector3 on "{self.pub_torque.topic_name}"')
        self.get_logger().info(HELP)

        # Current commanded vectors
        self.f = Vector3()
        self.t = Vector3()

        # Pulse expiry times (nanoseconds since epoch)
        self.expiry_ns = {
            'fx': 0, 'fy': 0, 'fz': 0,
            'tx': 0, 'ty': 0, 'tz': 0,
        }

        # Deadman / last-publish timestamp
        self.last_pub_ns = 0

        # Timer for periodic publish (and pulse logic)
        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self._on_timer)

        self._stop_event = threading.Event()

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    # ---- Periodic tick ----
    def _on_timer(self):
        now = self._now_ns()

        # Auto-zero axes whose pulses have expired
        if now > self.expiry_ns['fx']: self.f.x = 0.0
        if now > self.expiry_ns['fy']: self.f.y = 0.0
        if now > self.expiry_ns['fz']: self.f.z = 0.0
        if now > self.expiry_ns['tx']: self.t.x = 0.0
        if now > self.expiry_ns['ty']: self.t.y = 0.0
        if now > self.expiry_ns['tz']: self.t.z = 0.0

        # Optional in-pulse decay toward zero
        self.f.x *= self.pulse_decay
        self.f.y *= self.pulse_decay
        self.f.z *= self.pulse_decay
        self.t.x *= self.torque_decay
        self.t.y *= self.torque_decay
        self.t.z *= self.torque_decay

        # Deadman: if we haven't published in a while, zero everything
        if self.last_pub_ns and (now - self.last_pub_ns) > self.deadman_ms * 1_000_000:
            if any((self.f.x, self.f.y, self.f.z, self.t.x, self.t.y, self.t.z)):
                self.get_logger().warn("Deadman timeout → zeroing wrench")
            self.f.x = self.f.y = self.f.z = 0.0
            self.t.x = self.t.y = self.t.z = 0.0

        # Publish
        self.pub_force.publish(self.f)
        self.pub_torque.publish(self.t)
        self.last_pub_ns = now

    # ---- Key handler (creates pulses) ----
    def handle_key(self, ch: str):
        ch = ch.lower()
        updated = False
        now = self._now_ns()
        p_end = now + self.pulse_ms * 1_000_000
        tp_end = now + self.torque_pulse_ms * 1_000_000

        # Translation (mutually exclusive X/Y like your original; Z independent)
        if ch == 'w':   self.f.x =  self.force_mag; self.f.y = 0.0; self.expiry_ns['fx'] = p_end; self.expiry_ns['fy'] = now; updated = True
        elif ch == 's': self.f.x = -self.force_mag; self.f.y = 0.0; self.expiry_ns['fx'] = p_end; self.expiry_ns['fy'] = now; updated = True
        elif ch == 'a': self.f.y =  self.force_mag; self.f.x = 0.0; self.expiry_ns['fy'] = p_end; self.expiry_ns['fx'] = now; updated = True
        elif ch == 'd': self.f.y = -self.force_mag; self.f.x = 0.0; self.expiry_ns['fy'] = p_end; self.expiry_ns['fx'] = now; updated = True
        elif ch == 'e': self.f.z =  self.force_mag; self.expiry_ns['fz'] = p_end; updated = True
        elif ch == 'q': self.f.z = -self.force_mag; self.expiry_ns['fz'] = p_end; updated = True

        # Rotation (independent)
        elif ch == 'i': self.t.y =  self.torque_mag; self.expiry_ns['ty'] = tp_end; updated = True  # +Pitch
        elif ch == 'k': self.t.y = -self.torque_mag; self.expiry_ns['ty'] = tp_end; updated = True  # -Pitch
        elif ch == 'j': self.t.z =  self.torque_mag; self.expiry_ns['tz'] = tp_end; updated = True  # +Yaw
        elif ch == 'l': self.t.z = -self.torque_mag; self.expiry_ns['tz'] = tp_end; updated = True  # -Yaw
        elif ch == 'u': self.t.x =  self.torque_mag; self.expiry_ns['tx'] = tp_end; updated = True  # +Roll
        elif ch == 'o': self.t.x = -self.torque_mag; self.expiry_ns['tx'] = tp_end; updated = True  # -Roll

        # Zero all immediately
        elif ch == ' ':
            self.f.x = self.f.y = self.f.z = 0.0
            self.t.x = self.t.y = self.t.z = 0.0
            for k in self.expiry_ns: self.expiry_ns[k] = now
            updated = True

        if not updated:
            return

        # Publish immediately for crisp response
        self.pub_force.publish(self.f)
        self.pub_torque.publish(self.t)
        self.last_pub_ns = now

        self.get_logger().info(
            f"Key '{ch}' → F=({self.f.x:.1f},{self.f.y:.1f},{self.f.z:.1f}) "
            f"T=({self.t.x:.1f},{self.t.y:.1f},{self.t.z:.1f}) "
            f"[pulse {self.pulse_ms}ms/{self.torque_pulse_ms}ms]"
        )

    def stop(self):
        self._stop_event.set()

def main():
    rclpy.init()
    node = WasdTeleop()

    stdin_is_tty = sys.stdin.isatty()
    if stdin_is_tty:
        print(HELP)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if stdin_is_tty:
                ch = _getch(0.0)
                if ch:
                    if ch == '\x03':  # Ctrl-C
                        break
                    node.handle_key(ch)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.get_logger().info("Shutting down wasd_teleop.")
        except Exception:
            pass
        node.stop()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
