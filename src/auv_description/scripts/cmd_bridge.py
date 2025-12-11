#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

HELP = """\
cmd_bridge: Twist cmd_vel → body-frame wrench (continuous)

Subscribes:
  cmd_vel_topic  (geometry_msgs/Twist)   default: /cmd_vel

Publishes:
  force_topic    (geometry_msgs/Vector3) default: /auve1/force_body
  torque_topic   (geometry_msgs/Vector3) default: /auve1/torque_body

Behavior:
  - Stores latest Twist from cmd_vel_topic.
  - Publishes corresponding force & torque at fixed rate (rate_hz).
  - No deadman: if cmd_vel stops, last command is held.

Parameters (ros2 run ... --ros-args -p name:=value):
  cmd_vel_topic  (string)  default: /cmd_vel
  force_topic    (string)  default: /auve1/force_body
  torque_topic   (string)  default: /auve1/torque_body
  k_linear       (double)  default: 100.0  # N per (m/s)
  k_angular      (double)  default: 50.0   # N·m per (rad/s)
  rate_hz        (double)  default: 20.0   # publish rate [Hz]
"""


class CmdVelToWrench(Node):
    """
    Bridge node: latest Twist command is mapped to body-frame force & torque
    and published continuously at rate_hz.
    """
    def __init__(self):
        super().__init__("cmd_vel_to_wrench")

        # Parameters
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("force_topic", "/auve1/force_body")
        self.declare_parameter("torque_topic", "/auve1/torque_body")
        self.declare_parameter("k_linear", 100.0)
        self.declare_parameter("k_angular", 50.0)
        self.declare_parameter("rate_hz", 20.0)

        self.cmd_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.force_topic = self.get_parameter("force_topic").get_parameter_value().string_value
        self.torque_topic = self.get_parameter("torque_topic").get_parameter_value().string_value
        self.k_lin = float(self.get_parameter("k_linear").value)
        self.k_ang = float(self.get_parameter("k_angular").value)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))

        self.pub_force = self.create_publisher(Vector3, self.force_topic, 10)
        self.pub_torque = self.create_publisher(Vector3, self.torque_topic, 10)

        self.sub_cmd = self.create_subscription(
            Twist,
            self.cmd_topic,
            self.cb_cmd,
            10,
        )

        self.last_twist = Twist()

        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(HELP)
        self.get_logger().info(
            f"Bridging {self.cmd_topic} (Twist) → "
            f"{self.force_topic} / {self.torque_topic} (Vector3)\n"
            f"k_linear={self.k_lin}, k_angular={self.k_ang}, rate_hz={self.rate_hz}"
        )

    def cb_cmd(self, msg: Twist):
        self.last_twist = msg

    def on_timer(self):
        t = self.last_twist

        f = Vector3()
        tau = Vector3()

        # Simple proportional mapping: v → F, ω → τ
        f.x = self.k_lin * t.linear.x
        f.y = self.k_lin * t.linear.y
        f.z = self.k_lin * t.linear.z

        tau.x = self.k_ang * t.angular.x  # roll
        tau.y = self.k_ang * t.angular.y  # pitch
        tau.z = self.k_ang * t.angular.z  # yaw

        self.pub_force.publish(f)
        self.pub_torque.publish(tau)


def main():
    rclpy.init()
    node = CmdVelToWrench()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.get_logger().info("Shutting down cmd_vel_to_wrench.")
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
