"""
Control system for AUV gate navigation
"""

import numpy as np
from geometry_msgs.msg import Twist
from enum import Enum

class NavigationState(Enum):
    SEARCHING = 1
    APPROACHING = 2
    TRAVERSING = 3
    PASSED = 4

class GateController:
    def __init__(self, kp_yaw=0.003, kp_forward=0.001, kp_depth=0.002,
                 target_depth_px=240, forward_speed=0.3,
                 far_threshold=200, close_threshold=400):
        self.kp_yaw = kp_yaw
        self.kp_forward = kp_forward
        self.kp_depth = kp_depth
        self.target_depth_px = target_depth_px
        self.forward_speed = forward_speed
        self.far_threshold = far_threshold
        self.close_threshold = close_threshold

        self.state = NavigationState.SEARCHING
        self.last_gate_x = None
        self.frames_without_detection = 0

    def compute_control(self, gate_info, frame_shape):
        if gate_info is None:
            self.frames_without_detection += 1
            return self._search_pattern()

        self.frames_without_detection = 0
        gate_x, gate_y, pole_distance = gate_info
        self.last_gate_x = gate_x

        height, width = frame_shape[:2]
        frame_center_x = width // 2

        error_x = gate_x - frame_center_x
        error_y = gate_y - self.target_depth_px

        cmd = Twist()
        cmd.angular.z = -self.kp_yaw * error_x
        cmd.linear.z = -self.kp_depth * error_y
        cmd.linear.x = self._compute_forward_speed(pole_distance)

        cmd.linear.x = np.clip(cmd.linear.x, 0, 0.5)
        cmd.angular.z = np.clip(cmd.angular.z, -0.5, 0.5)
        cmd.linear.z = np.clip(cmd.linear.z, -0.3, 0.3)

        return cmd

    def _compute_forward_speed(self, pole_distance):
        if pole_distance < self.far_threshold:
            return self.forward_speed
        elif pole_distance > self.close_threshold:
            return self.forward_speed * 0.3
        else:
            return self.forward_speed * 0.6

    def _search_pattern(self):
        cmd = Twist()
        if self.frames_without_detection < 30:
            cmd.linear.x = 0.1
        else:
            cmd.angular.z = 0.2
        return cmd
