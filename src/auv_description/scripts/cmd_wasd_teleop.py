#!/usr/bin/env python3
import sys, termios, tty, select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

HELP = """\
AUV keyboard teleop (force/torque only, continuous set & hold)

Keys:
  W/S: ±X   A/D: ±Y   E/Q: ±Z
  I/K: ±Pitch (±Y)   J/L: ±Yaw (±Z)   U/O: ±Roll (±X)
  Space: zero all
"""

def _getch_nonblocking():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], 0.0)
        return sys.stdin.read(1) if r else None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

class KeyboardAuvTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_auv_teleop")

        # Params FIRST
        self.declare_parameter("force_topic", "/auve1/force_body")
        self.declare_parameter("torque_topic", "/auve1/torque_body")
        self.declare_parameter("linear_speed", 1.0)
        self.declare_parameter("angular_speed", 1.0)
        self.declare_parameter("rate_hz", 5000.0)
        self.declare_parameter("k_linear", 100.0)
        self.declare_parameter("k_angular", 50.0)

        self.force_topic   = self.get_parameter("force_topic").get_parameter_value().string_value
        self.torque_topic  = self.get_parameter("torque_topic").get_parameter_value().string_value
        self.linear_speed  = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.rate_hz       = max(1.0, float(self.get_parameter("rate_hz").value))
        self.k_linear      = float(self.get_parameter("k_linear").value)
        self.k_angular     = float(self.get_parameter("k_angular").value)

        # Publishers (force/torque only)
        self.pub_force = self.create_publisher(Vector3, self.force_topic, 10)
        self.pub_tau   = self.create_publisher(Vector3, self.torque_topic, 10)
        
        # NOW wait for subscribers (after publishers exist!)
        self.get_logger().info("Waiting for subscribers...")
        while self.pub_force.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"Connected! {self.pub_force.get_subscription_count()} subscriber(s)")
        
        self.get_logger().info(f'Publishing Vector3 force on "{self.pub_force.topic_name}"')
        self.get_logger().info(f'Publishing Vector3 torque on "{self.pub_tau.topic_name}"')
        self.get_logger().info(HELP)

        # Rest of initialization...
        self.twist = Twist()
        self.timer_pub = self.create_timer(1.0 / self.rate_hz, self._on_publish_timer)
        self.stdin_is_tty = sys.stdin.isatty()
        self.timer_key = self.create_timer(0.001, self._on_key_poll)
        self._pub_count = 0

    def _on_publish_timer(self):
        # Map Twist → Vector3 wrench and publish
        f = Vector3(
            x=self.k_linear * self.twist.linear.x,
            y=self.k_linear * self.twist.linear.y,
            z=self.k_linear * self.twist.linear.z,
        )
        tau = Vector3(
            x=self.k_angular * self.twist.angular.x,
            y=self.k_angular * self.twist.angular.y,
            z=self.k_angular * self.twist.angular.z,
        )
        self.pub_force.publish(f)
        self.pub_tau.publish(tau)

        self._pub_count += 1
        if self._pub_count % int(self.rate_hz * 1.0) == 0:
            self.get_logger().info("Publishing force/torque...")

    def _on_key_poll(self):
        if not self.stdin_is_tty:
            return
        ch = _getch_nonblocking()
        if not ch:
            return
        if ch == "\x03":  # Ctrl-C
            rclpy.shutdown()
            return
        self._handle_key(ch)

    def _zero(self):
        self.twist.linear.x = self.twist.linear.y = self.twist.linear.z = 0.0
        self.twist.angular.x = self.twist.angular.y = self.twist.angular.z = 0.0

    def _handle_key(self, ch: str):
        ch = ch.lower()
        updated = True

        # Translation
        if ch == "w":
            self.twist.linear.x = self.linear_speed;  self.twist.linear.y = 0.0
        elif ch == "s":
            self.twist.linear.x = -self.linear_speed; self.twist.linear.y = 0.0
        elif ch == "a":
            self.twist.linear.y = self.linear_speed;  self.twist.linear.x = 0.0
        elif ch == "d":
            self.twist.linear.y = -self.linear_speed; self.twist.linear.x = 0.0
        elif ch == "e":
            self.twist.linear.z = self.linear_speed
        elif ch == "q":
            self.twist.linear.z = -self.linear_speed

        # Rotation (roll=x, pitch=y, yaw=z)
        elif ch == "i":
            self.twist.angular.y = self.angular_speed
        elif ch == "k":
            self.twist.angular.y = -self.angular_speed
        elif ch == "j":
            self.twist.angular.z = self.angular_speed
        elif ch == "l":
            self.twist.angular.z = -self.angular_speed
        elif ch == "u":
            self.twist.angular.x = self.angular_speed
        elif ch == "o":
            self.twist.angular.x = -self.angular_speed

        elif ch == " ":
            self._zero()
        else:
            updated = False

        if updated:
            self.get_logger().info(
                f"Key '{ch}' → lin=({self.twist.linear.x:.2f},"
                f"{self.twist.linear.y:.2f},{self.twist.linear.z:.2f}) "
                f"ang=({self.twist.angular.x:.2f},{self.twist.angular.y:.2f},"
                f"{self.twist.angular.z:.2f})"
            )

def main():
    rclpy.init()
    node = KeyboardAuvTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down keyboard_auv_teleop.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
