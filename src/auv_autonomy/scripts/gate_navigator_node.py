#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu
from ros_gz_interfaces.msg import Altimeter

import cv2
import numpy as np
from cv_bridge import CvBridge
import math


def quat_to_euler_xyz(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class GateNavigator(Node):
    """
    UPDATED: lane-following between red(left boundary) and white(right boundary) poles.

    Keeps your existing:
      - depth PID
      - roll/pitch stabilization
      - SEARCH/APPROACH/PASS state machine

    Changes:
      - replaces "detect_gate_and_debug" with lane fit:
          fit x = a*y + b lines for red boundary and white boundary
          lane centerline = average of those two
          yaw = kp*(pixel center error) + feed-forward from lane heading + yaw-rate damping
    """

    def __init__(self):
        super().__init__('gate_navigator')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('enable_debug', True),
                ('show_cv_windows', True),

                # depth
                ('target_depth', 1.0),
                ('depth_increases_down', True),
                ('kp_depth', 4.0),
                ('ki_depth', 0.1),
                ('kd_depth', 0.5),
                ('max_heave_cmd', 2.0),

                # roll/pitch stabilize
                ('kp_roll', 3.0),
                ('kd_roll', 0.8),
                ('kp_pitch', 3.0),
                ('kd_pitch', 0.8),
                ('max_att_cmd', 2.0),

                # yaw / lane tracking
                ('kp_yaw_gate', 0.003),     # pixel -> yaw
                ('kd_yaw', 0.05),           # yaw-rate damping
                ('k_lane_theta', 0.6),      # feed-forward from lane heading
                ('max_yaw', 1.2),
                ('search_yaw', 0.25),
                ('center_tol_px', 25),

                # lane specifics
                ('lane_lookahead_frac', 0.65),   # y lookahead as fraction of image height
                ('lane_found_timeout', 0.5),     # seconds to "coast" after losing lane
                ('min_poles_each_color', 2),     # need at least N points each side
                ('max_poles_each_color', 6),     # use up to N strongest / lowest poles

                # forward
                ('forward_speed', 0.30),
                ('forward_speed_search', 0.05),

                # pass logic
                ('pass_width_px', 260),
                ('pass_time_s', 1.0),

                # HSV
                ('hsv_red1_low',  [0,   100, 100]),
                ('hsv_red1_high', [10,  255, 255]),
                ('hsv_red2_low',  [170, 100, 100]),
                ('hsv_red2_high', [180, 255, 255]),
                ('hsv_white_low',  [0,   0,   55]),
                ('hsv_white_high', [179, 45, 255]),

                # contour / filtering
                ('min_contour_area', 300),
                ('min_aspect', 2.0),
                ('blur_ksize', 5),
                ('morph_ksize', 5),

                # anti-"white background" (optional but helps a lot)
                ('roi_top_frac', 0.35),          # ignore top % of image
                ('max_component_frac', 0.20),    # drop connected components bigger than this % of pixels
            ]
        )

        self.debug = bool(self.get_parameter('enable_debug').value)
        self.show_cv = bool(self.get_parameter('show_cv_windows').value)

        self.target_depth = float(self.get_parameter('target_depth').value)
        self.depth_increases_down = bool(self.get_parameter('depth_increases_down').value)
        self.kp_depth = float(self.get_parameter('kp_depth').value)
        self.ki_depth = float(self.get_parameter('ki_depth').value)
        self.kd_depth = float(self.get_parameter('kd_depth').value)
        self.max_heave_cmd = float(self.get_parameter('max_heave_cmd').value)

        self.kp_roll = float(self.get_parameter('kp_roll').value)
        self.kd_roll = float(self.get_parameter('kd_roll').value)
        self.kp_pitch = float(self.get_parameter('kp_pitch').value)
        self.kd_pitch = float(self.get_parameter('kd_pitch').value)
        self.max_att_cmd = float(self.get_parameter('max_att_cmd').value)

        self.kp_yaw_gate = float(self.get_parameter('kp_yaw_gate').value)
        self.kd_yaw = float(self.get_parameter('kd_yaw').value)
        self.k_lane_theta = float(self.get_parameter('k_lane_theta').value)
        self.max_yaw = float(self.get_parameter('max_yaw').value)
        self.search_yaw = float(self.get_parameter('search_yaw').value)
        self.center_tol_px = int(self.get_parameter('center_tol_px').value)

        self.lane_lookahead_frac = float(self.get_parameter('lane_lookahead_frac').value)
        self.lane_found_timeout = float(self.get_parameter('lane_found_timeout').value)
        self.min_poles_each_color = int(self.get_parameter('min_poles_each_color').value)
        self.max_poles_each_color = int(self.get_parameter('max_poles_each_color').value)

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.forward_speed_search = float(self.get_parameter('forward_speed_search').value)

        self.pass_width_px = int(self.get_parameter('pass_width_px').value)
        self.pass_time_s = float(self.get_parameter('pass_time_s').value)

        self.red1_low  = np.array(self.get_parameter('hsv_red1_low').value,  dtype=np.uint8)
        self.red1_high = np.array(self.get_parameter('hsv_red1_high').value, dtype=np.uint8)
        self.red2_low  = np.array(self.get_parameter('hsv_red2_low').value,  dtype=np.uint8)
        self.red2_high = np.array(self.get_parameter('hsv_red2_high').value, dtype=np.uint8)

        self.white_low  = np.array(self.get_parameter('hsv_white_low').value,  dtype=np.uint8)
        self.white_high = np.array(self.get_parameter('hsv_white_high').value, dtype=np.uint8)

        self.min_area = int(self.get_parameter('min_contour_area').value)
        self.min_aspect = float(self.get_parameter('min_aspect').value)

        self.blur_ksize = int(self.get_parameter('blur_ksize').value)
        self.morph_ksize = int(self.get_parameter('morph_ksize').value)

        self.roi_top_frac = float(self.get_parameter('roi_top_frac').value)
        self.max_component_frac = float(self.get_parameter('max_component_frac').value)

        # ROS / CV state
        self.bridge = CvBridge()
        self.image = None

        self.depth = 0.0
        self.have_depth = False

        self.roll = self.pitch = self.yaw = 0.0
        self.roll_rate = self.pitch_rate = self.yaw_rate = 0.0
        self.have_imu = False

        # depth PID memory
        self.integral_depth = 0.0
        self.prev_depth_error = 0.0
        self.prev_time = self.get_clock().now()

        # autonomy state
        self.state = "SEARCH"
        self.pass_start_time = None
        self.last_lane_seen_time = None

        # ROS I/O
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/cube/image_raw', self.image_callback, 10)
        self.altimeter_sub = self.create_subscription(Altimeter, '/altimeter', self.altimeter_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        if self.show_cv:
            cv2.namedWindow("rgb", cv2.WINDOW_NORMAL)
            cv2.namedWindow("red_mask", cv2.WINDOW_NORMAL)
            cv2.namedWindow("white_mask", cv2.WINDOW_NORMAL)
            cv2.namedWindow("combined", cv2.WINDOW_NORMAL)
            cv2.namedWindow("overlay", cv2.WINDOW_NORMAL)

        self.get_logger().info("GateNavigator (LANE MODE) running with OpenCV imshow debug.")

    def altimeter_callback(self, msg: Altimeter):
        z = float(msg.vertical_position)
        self.depth = z if self.depth_increases_down else -z
        self.have_depth = True

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.roll, self.pitch, self.yaw = quat_to_euler_xyz(q.x, q.y, q.z, q.w)

        w = msg.angular_velocity
        self.roll_rate = float(w.x)
        self.pitch_rate = float(w.y)
        self.yaw_rate = float(w.z)

        self.have_imu = True

    def image_callback(self, msg: Image):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")

    # ---------------- lane vision helpers ----------------
    def _pole_centroids(self, mask: np.ndarray):
        """Return list of (cx, cy, area, (x,y,w,h)) for pole-like contours."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        poles = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w <= 0:
                continue
            aspect = h / float(w)
            if aspect < self.min_aspect:
                continue
            M = cv2.moments(c)
            if M["m00"] <= 1e-6:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            poles.append((cx, cy, area, (x, y, w, h)))

        # prefer nearer poles (lower in image), then bigger
        poles.sort(key=lambda p: (p[1], p[2]), reverse=True)
        return poles

    def _remove_big_components(self, mask: np.ndarray):
        """Drop huge connected components (often 'background white') so poles survive."""
        H, W = mask.shape
        max_big = int(self.max_component_frac * H * W)
        num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        out = np.zeros_like(mask)
        for i in range(1, num):
            area = stats[i, cv2.CC_STAT_AREA]
            if 0 < area < max_big:
                out[labels == i] = 255
        return out

    def detect_lane_and_debug(self, bgr: np.ndarray):
        """
        Fit boundaries x=a*y+b for red(left) and white(right).
        Returns:
          x_center_lookahead, lane_theta, lane_width_px, dbg
          OR (None, None, None, dbg)
        """
        img = bgr
        if self.blur_ksize and self.blur_ksize > 1:
            k = self.blur_ksize if self.blur_ksize % 2 == 1 else self.blur_ksize + 1
            img = cv2.GaussianBlur(img, (k, k), 0)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        red_mask1 = cv2.inRange(hsv, self.red1_low, self.red1_high)
        red_mask2 = cv2.inRange(hsv, self.red2_low, self.red2_high)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        white_mask = cv2.inRange(hsv, self.white_low, self.white_high)

        mk = self.morph_ksize if self.morph_ksize % 2 == 1 else self.morph_ksize + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (mk, mk))

        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # ROI: ignore top of image (often skybox / walls)
        H, W = bgr.shape[:2]
        roi_y0 = int(np.clip(self.roi_top_frac, 0.0, 0.95) * H)
        if roi_y0 > 0:
            red_mask[:roi_y0, :] = 0
            white_mask[:roi_y0, :] = 0

        # remove giant background-like components from white
        white_mask = self._remove_big_components(white_mask)

        combined = cv2.bitwise_or(red_mask, white_mask)

        # overlay with transparency so you always see camera
        overlay = bgr.copy()
        layer = np.zeros_like(bgr)
        layer[red_mask > 0] = (0, 0, 255)
        layer[white_mask > 0] = (255, 255, 255)
        overlay = cv2.addWeighted(overlay, 1.0, layer, 0.35, 0.0)

        red_poles = self._pole_centroids(red_mask)[:self.max_poles_each_color]
        white_poles = self._pole_centroids(white_mask)[:self.max_poles_each_color]

        # draw boxes/centroids
        for (cx, cy, area, (x, y, w, h)) in red_poles:
            cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.circle(overlay, (cx, cy), 3, (0, 0, 255), -1)
        for (cx, cy, area, (x, y, w, h)) in white_poles:
            cv2.rectangle(overlay, (x, y), (x + w, y + h), (220, 220, 220), 2)
            cv2.circle(overlay, (cx, cy), 3, (220, 220, 220), -1)

        dbg = {
            "rgb": bgr,
            "red_mask": red_mask,
            "white_mask": white_mask,
            "combined": combined,
            "overlay": overlay,
        }

        if len(red_poles) < self.min_poles_each_color or len(white_poles) < self.min_poles_each_color:
            return None, None, None, dbg

        # lookahead row
        y_look = int(np.clip(self.lane_lookahead_frac, 0.1, 0.95) * H)

        # fit x = a*y + b for each boundary
        ry = np.array([p[1] for p in red_poles], dtype=np.float32)
        rx = np.array([p[0] for p in red_poles], dtype=np.float32)
        wy = np.array([p[1] for p in white_poles], dtype=np.float32)
        wx = np.array([p[0] for p in white_poles], dtype=np.float32)

        ar, br = np.polyfit(ry, rx, 1)
        aw, bw = np.polyfit(wy, wx, 1)

        x_left = float(ar * y_look + br)
        x_right = float(aw * y_look + bw)

        # enforce red-left / white-right at lookahead
        if x_left >= x_right:
            return None, None, None, dbg

        x_center = 0.5 * (x_left + x_right)
        x_center = float(np.clip(x_center, 0.0, float(W - 1)))
        lane_width = float(x_right - x_left)

        # centerline slope
        a_center = 0.5 * (ar + aw)
        lane_theta = float(np.arctan(a_center))  # radians

        # draw fitted lines + center lookahead point
        y0, y1 = 0, H - 1
        xl0, xl1 = int(ar * y0 + br), int(ar * y1 + br)
        xr0, xr1 = int(aw * y0 + bw), int(aw * y1 + bw)
        cv2.line(overlay, (xl0, y0), (xl1, y1), (0, 0, 255), 2)
        cv2.line(overlay, (xr0, y0), (xr1, y1), (220, 220, 220), 2)
        cv2.circle(overlay, (int(x_center), int(y_look)), 6, (0, 255, 0), -1)
        cv2.line(overlay, (int(W / 2), 0), (int(W / 2), H - 1), (0, 255, 255), 1)

        return x_center, lane_theta, lane_width, dbg

    # ---------------- control loop ----------------
    def control_loop(self):
        if self.image is None or not self.have_depth or not self.have_imu:
            return

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 1e-6:
            return
        self.prev_time = now

        # Depth PID
        depth_error = self.target_depth - self.depth
        self.integral_depth += depth_error * dt
        derivative_depth = (depth_error - self.prev_depth_error) / dt
        self.prev_depth_error = depth_error

        heave = (self.kp_depth * depth_error +
                 self.ki_depth * self.integral_depth +
                 self.kd_depth * derivative_depth)
        heave = float(np.clip(heave, -self.max_heave_cmd, self.max_heave_cmd))

        # Roll/pitch stabilization (PD)
        roll_cmd = self.kp_roll * (0.0 - self.roll) + self.kd_roll * (0.0 - self.roll_rate)
        pitch_cmd = self.kp_pitch * (0.0 - self.pitch) + self.kd_pitch * (0.0 - self.pitch_rate)
        roll_cmd = float(np.clip(roll_cmd, -self.max_att_cmd, self.max_att_cmd))
        pitch_cmd = float(np.clip(pitch_cmd, -self.max_att_cmd, self.max_att_cmd))

        # Lane detection
        x_center, lane_theta, lane_width, dbg = self.detect_lane_and_debug(self.image)

        # show debug windows
        if self.show_cv:
            cv2.imshow("rgb", dbg["rgb"])
            cv2.imshow("red_mask", dbg["red_mask"])
            cv2.imshow("white_mask", dbg["white_mask"])
            cv2.imshow("combined", dbg["combined"])
            cv2.imshow("overlay", dbg["overlay"])
            cv2.waitKey(1)

        H, W = self.image.shape[:2]
        img_cx = 0.5 * W

        cmd = Twist()
        cmd.linear.z = heave
        cmd.angular.x = roll_cmd
        cmd.angular.y = pitch_cmd

        # ---- Lost lane handling ----
        lane_now = (x_center is not None)

        if lane_now:
            self.last_lane_seen_time = now
        else:
            # if lane recently seen: coast
            if self.last_lane_seen_time is not None:
                lost_s = (now - self.last_lane_seen_time).nanoseconds / 1e9
            else:
                lost_s = 1e9

            if lost_s <= self.lane_found_timeout:
                cmd.linear.x = 0.5 * self.forward_speed
                cmd.angular.z = float(np.clip(-self.kd_yaw * self.yaw_rate, -self.max_yaw, self.max_yaw))
                self.cmd_pub.publish(cmd)
                if self.debug:
                    self.get_logger().info(
                        f"[COAST] x={cmd.linear.x:.2f} z={cmd.linear.z:+.2f} "
                        f"r={cmd.angular.x:+.2f} p={cmd.angular.y:+.2f} yaw={cmd.angular.z:+.2f} "
                        f"depth={self.depth:.2f}"
                    )
                return

            # SEARCH spin
            self.state = "SEARCH"
            self.pass_start_time = None
            cmd.linear.x = self.forward_speed_search
            cmd.angular.z = float(np.clip(self.search_yaw + (-self.kd_yaw * self.yaw_rate),
                                          -self.max_yaw, self.max_yaw))
            self.cmd_pub.publish(cmd)
            if self.debug:
                self.get_logger().info(
                    f"[SEARCH] x={cmd.linear.x:.2f} z={cmd.linear.z:+.2f} "
                    f"r={cmd.angular.x:+.2f} p={cmd.angular.y:+.2f} yaw={cmd.angular.z:+.2f} "
                    f"depth={self.depth:.2f}"
                )
            return

        # ---- Lane-follow yaw ----
        error_px = img_cx - x_center  # + => lane center is left => yaw left
        yaw_track = self.kp_yaw_gate * error_px
        yaw_ff = -self.k_lane_theta * lane_theta
        yaw_damp = -self.kd_yaw * self.yaw_rate
        yaw_cmd = float(np.clip(yaw_track + yaw_ff + yaw_damp, -self.max_yaw, self.max_yaw))

        centered = abs(error_px) < self.center_tol_px

        # ---- State machine (reuse your PASS logic, use lane width instead of gate width) ----
        if self.state == "SEARCH":
            self.state = "APPROACH"

        if self.state == "APPROACH":
            cmd.angular.z = yaw_cmd
            cmd.linear.x = self.forward_speed if centered else 0.15 * self.forward_speed

            # when lane looks wide enough (close), do PASS
            if lane_width is not None and lane_width >= float(self.pass_width_px):
                self.state = "PASS"
                self.pass_start_time = now

        elif self.state == "PASS":
            cmd.angular.z = 0.0
            cmd.linear.x = self.forward_speed

            elapsed = 0.0
            if self.pass_start_time is not None:
                elapsed = (now - self.pass_start_time).nanoseconds / 1e9
            if elapsed >= self.pass_time_s:
                self.state = "SEARCH"
                self.pass_start_time = None

        self.cmd_pub.publish(cmd)

        if self.debug:
            self.get_logger().info(
                f"[{self.state}] x={cmd.linear.x:.2f} z={cmd.linear.z:+.2f} "
                f"r={cmd.angular.x:+.2f} p={cmd.angular.y:+.2f} yaw={cmd.angular.z:+.2f} "
                f"err_px={error_px:+.1f} theta={lane_theta:+.3f} lane_w={lane_width:.1f} depth={self.depth:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = GateNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
