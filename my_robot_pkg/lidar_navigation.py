"""
LiDAR tabanli serit tarama (boustrophedon) ve koridor ortalama modulu.
"""

import math
from typing import Optional, Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class LidarNavigation:
    """Saf serit tarama durum makinesi ve kararlı engel bypass kontrolcusu."""

    def __init__(self,
                 fwd_speed: float = 0.20,
                 cross_speed: float = 0.18,
                 turn_speed: float = 0.45,
                 lane_dist: float = 1.20,
                 max_lanes: int = 2,
                 max_w: float = 0.45,
                 yaw_tol: float = 0.05,
                 min_range: float = 0.12):
        self.fwd_speed = fwd_speed
        self.cross_speed = cross_speed
        self.turn_speed = turn_speed
        self.lane_dist = lane_dist
        self.max_lanes = max_lanes
        self.max_w = max_w
        self.yaw_tol = yaw_tol
        self.min_range = min_range

        self.state = 'AISLE'
        self.lanes_completed = 0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        self.lane_y_list = [0.0, -self.lane_dist]
        self.lane_heading = 0.0
        self.base_lane_y = 0.0

        # bumper/recovery yanal ofset
        self.external_offset_y = 0.0
        self.offset_remaining_m = 0.0

        self.turn_target = 0.0
        self.target_y = 0.0
        self.shift_start_y = 0.0
        self.shift_start_time = 0.0
        self._now = 0.0

        self.avoiding_obstacle = False
        self.active_avoid_y_shift = 0.0
        self.avoid_start_x = 0.0

        # LiDAR sektorleri
        self.front = 6.0
        self.left = 0.6
        self.right = 0.6
        self._r: np.ndarray = np.zeros(0, dtype=float)
        self._a: np.ndarray = np.zeros(0, dtype=float)

    def apply_offset(self, offset_m: float, length_m: float):
        """(bumper/recovery) fiziksel carpma tespit ettiginde cagirilir."""
        self.external_offset_y = clamp(offset_m, -0.22, 0.22)
        self.offset_remaining_m = max(0.0, length_m)

    def set_pose(self, x: float, y: float, yaw: float):
        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw
        self.odom_received = True

    def update_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.set_pose(p.x, p.y, yaw)

    def _sector(self, a0: float, a1: float) -> float:
        m = (self._a >= a0) & (self._a <= a1)
        v = self._r[m]
        v = v[np.isfinite(v) & (v >= self.min_range)]
        return float(np.min(v)) if v.size else 6.0

    def _corridor_obstacle_info(self, look: float = 0.95) -> Tuple[float, float]:
        """
        Koridor icindeki engelin mesafesini ve ortalama Y konumunu hesaplar.
        """
        if self._r.size == 0 or not self.odom_received:
            return 6.0, 0.0

        ok = np.isfinite(self._r) & (self._r >= self.min_range)
        x_pts = self.current_x + self._r * np.cos(self.current_yaw + self._a)
        y_pts = self.current_y + self._r * np.sin(self.current_yaw + self._a)

        is_east = (self.lanes_completed % 2 == 0)
        curr_idx = min(self.lanes_completed, len(self.lane_y_list) - 1)
        base_y = self.lane_y_list[curr_idx]

        in_corridor_y = (y_pts > base_y - 0.30) & (y_pts < base_y + 0.30)
        dx = (x_pts - self.current_x) if is_east else (self.current_x - x_pts)
        in_lane = ok & in_corridor_y & (dx > 0.08) & (dx < look)

        if not in_lane.any():
            return 6.0, 0.0

        min_d = float(np.min(dx[in_lane]))
        mean_y = float(np.mean(y_pts[in_lane]))
        return min_d, mean_y

    def update(self,
               scan: LaserScan,
               pose: Optional[Tuple[float, float, float]] = None,
               now: float = 0.0) -> Twist:
        self._now = now
        if pose is not None:
            self.set_pose(pose[0], pose[1], pose[2])

        cmd = Twist()
        if not self.odom_received:
            return cmd

        self._r = np.asarray(scan.ranges, dtype=float)
        self._a = scan.angle_min + np.arange(self._r.size) * scan.angle_increment
        rad = math.radians

        self.front = self._sector(-rad(10), rad(10))
        self.left = self._sector(rad(70), rad(110))
        self.right = self._sector(-rad(110), -rad(70))

        if self.state in ('AISLE', 'LANE'):
            return self._handle_aisle()
        elif self.state == 'TURN1':
            return self._handle_turn1()
        elif self.state == 'SHIFT':
            return self._handle_shift()
        elif self.state == 'TURN2':
            return self._handle_turn2()
        elif self.state == 'DONE':
            return cmd
        return cmd

    def _handle_aisle(self) -> Twist:
        cmd = Twist()
        is_east = (self.lanes_completed % 2 == 0)

        corridor_done = False
        if is_east:
            if (self.current_x >= 8.80 and self.front < 1.05) or self.current_x >= 9.80:
                corridor_done = True
        else:
            if (self.current_x <= 0.20 and self.front < 1.05) or self.current_x <= -1.20:
                corridor_done = True

        if corridor_done:
            self.avoiding_obstacle = False
            self.active_avoid_y_shift = 0.0
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if self.lanes_completed < self.max_lanes - 1:
                next_idx = self.lanes_completed + 1
                self.target_y = self.lane_y_list[next_idx]
                self.turn_target = (
                    -math.pi / 2.0 if self.target_y < self.current_y else math.pi / 2.0
                )
                self.state = 'TURN1'
            else:
                self.state = 'DONE'
            return cmd

        curr_idx = min(self.lanes_completed, len(self.lane_y_list) - 1)
        self.base_lane_y = self.lane_y_list[curr_idx]

        is_lane1 = (self.lanes_completed == 0 or self.current_y > -0.60)
        on_ramp = is_lane1 and (1.80 <= self.current_x <= 4.60)

        obs_dist, obs_y = self._corridor_obstacle_info(look=0.95)
        obs_ahead = (self.current_x > 1.20) and (obs_dist < 0.85) and (not on_ramp)

        if obs_ahead and not self.avoiding_obstacle:
            self.avoiding_obstacle = True
            self.avoid_start_x = self.current_x
            if not is_east:
                self.active_avoid_y_shift = -0.17 if obs_y >= self.base_lane_y else 0.17
            else:
                self.active_avoid_y_shift = -0.17 if obs_y >= self.base_lane_y else 0.17

        if self.avoiding_obstacle:
            dist_in_avoid = abs(self.current_x - self.avoid_start_x)
            if obs_dist < 0.50:
                self.avoid_start_x = self.current_x
            elif dist_in_avoid >= 0.70 and not obs_ahead:
                self.avoiding_obstacle = False
                self.active_avoid_y_shift = 0.0

        if self.avoiding_obstacle:
            target_y = self.base_lane_y + self.active_avoid_y_shift
        else:
            target_y = self.base_lane_y + self.external_offset_y

        if on_ramp:
            v = self.fwd_speed * 1.15
        elif self.avoiding_obstacle and abs(self.current_y - target_y) > 0.04:
            v = 0.11  # Kacinma seridine oturana kadar guvenli yavaslama
        elif self.avoiding_obstacle:
            v = 0.14  # Kacinma seridinde ilerleme hizi
        else:
            v = self.fwd_speed

        y_err = target_y - self.current_y

        if is_east:
            max_angle = math.radians(24.0) if self.avoiding_obstacle else math.radians(20.0)
            target_yaw = clamp(2.0 * y_err, -max_angle, max_angle)
            heading_err = normalize_angle(target_yaw - self.current_yaw)
        else:
            max_angle = math.radians(24.0) if self.avoiding_obstacle else math.radians(20.0)
            delta_yaw = clamp(2.0 * y_err, -max_angle, max_angle)
            target_yaw = normalize_angle(math.pi - delta_yaw)
            heading_err = normalize_angle(target_yaw - self.current_yaw)

        w = clamp(1.8 * heading_err, -self.max_w, self.max_w)

        if (not self.avoiding_obstacle) and (not on_ramp) and \
                self.left < 0.75 and self.right < 0.75:
            diff = self.left - self.right
            if abs(diff) > 0.06:
                w += clamp(0.5 * (diff / 2.0), -0.08, 0.08)

        if self.left < 0.11:
            w = min(w, -0.15)
        elif self.right < 0.11:
            w = max(w, 0.15)

        w = clamp(w, -self.max_w, self.max_w)

        step = abs(v * 0.1)
        if self.offset_remaining_m > 0.0:
            self.offset_remaining_m -= step
            if self.offset_remaining_m <= 0.0:
                self.external_offset_y = 0.0

        cmd.linear.x = v
        cmd.angular.z = w
        return cmd

    def _handle_shift(self) -> Twist:
        cmd = Twist()
        shift_dir = 1.0 if self.target_y > self.shift_start_y else -1.0
        past_target = (self.current_y <= self.target_y if shift_dir < 0.0
                       else self.current_y >= self.target_y)
        reached = past_target or abs(self.current_y - self.target_y) <= 0.08

        timeout = (self.shift_start_time > 0.0 and (self._now - self.shift_start_time) > 15.0)
        if reached or timeout:
            self.state = 'TURN2'
            return cmd

        y_dist = abs(self.current_y - self.target_y)
        speed = 0.08 if y_dist < 0.25 else self.cross_speed

        err = normalize_angle(self.turn_target - self.current_yaw)
        cmd.linear.x = speed
        cmd.angular.z = clamp(1.8 * err, -0.30, 0.30)
        return cmd

    def _rotate_to(self, target: float) -> Optional[float]:
        err = normalize_angle(target - self.current_yaw)
        if abs(abs(self.current_yaw) - math.pi) < self.yaw_tol and \
                abs(abs(target) - math.pi) < self.yaw_tol:
            return None
        if abs(err) < self.yaw_tol:
            return None
        w = clamp(1.8 * err, -self.turn_speed, self.turn_speed)
        if abs(w) < 0.18:
            w = math.copysign(0.18, err)
        return w

    def _handle_turn1(self) -> Twist:
        cmd = Twist()
        w = self._rotate_to(self.turn_target)
        if w is None:
            self.state = 'SHIFT'
            self.shift_start_y = self.current_y
            self.shift_start_time = self._now
        else:
            cmd.angular.z = w
        return cmd

    def _handle_turn2(self) -> Twist:
        cmd = Twist()
        target = math.pi if (self.lanes_completed % 2 == 0) else 0.0
        w = self._rotate_to(target)
        if w is None:
            self.lanes_completed += 1
            self.lane_heading = target
            self.avoiding_obstacle = False
            self.active_avoid_y_shift = 0.0
            self.avoid_start_x = 0.0
            self.external_offset_y = 0.0
            self.state = 'AISLE'
        else:
            cmd.angular.z = w
        return cmd


class LidarNavigationNode(Node):
    """Tek basina test calistirici dugumu."""

    def __init__(self):
        super().__init__('lidar_navigation_test')
        self.nav = LidarNavigation()
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)

    def on_odom(self, msg: Odometry):
        self.nav.update_odom(msg)

    def on_scan(self, scan: LaserScan):
        now = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        pose = (self.nav.current_x, self.nav.current_y, self.nav.current_yaw)
        cmd = self.nav.update(scan, pose=pose, now=now)
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = LidarNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()