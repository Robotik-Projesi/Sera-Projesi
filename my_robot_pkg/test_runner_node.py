"""
GECICI TEST DUGUMU (GitHub'a yuklenmeyecek).
Uye C gelene kadar IMU (rampa sabitleme) ve Bumper (carpisma/ofset cagrisi)
gorevlerini simule ederek LidarNavigation sinifini tek basina test eder.
"""

import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from my_robot_pkg.lidar_navigation import LidarNavigation, normalize_angle, clamp


class LidarStandaloneTester(Node):

    def __init__(self):
        super().__init__('lidar_standalone_tester')
        self.nav = LidarNavigation()

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)

    def on_odom(self, msg: Odometry):
        self.nav.update_odom(msg)

    def on_scan(self, scan: LaserScan):
        now = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        pose = (self.nav.current_x, self.nav.current_y, self.nav.current_yaw)

        # 1. Asıl Uye A LiDAR algoritmasindan cikan komutu al
        cmd = self.nav.update(scan, pose=pose, now=now)

        # ---------------- IMU VE BUMPER MOCK KATMANI ----------------
        # A) IMU GOREVI: Rampada (1.8 <= x <= 4.6) yalpalama onleme ve tork artirma
        on_ramp = (self.nav.lanes_completed == 0 and 1.8 <= self.nav.current_x <= 4.6)
        if on_ramp and self.nav.state == 'AISLE':
            heading_err = normalize_angle(self.nav.lane_heading - self.nav.current_yaw)
            cmd.angular.z = clamp(3.5 * heading_err, -0.4, 0.4)
            cmd.linear.x = self.nav.fwd_speed * 1.3

        if self.nav.front < 0.10 and not on_ramp and self.nav.state == 'AISLE':
            cmd.linear.x = 0.0
            # Uye C sozlesmesini test et: 0.18 metre saga kay ve 0.6 metre boyunca ofseti koru
            if self.nav.offset_remaining_m <= 0.0:
                self.get_logger().warning("MOCK BUMPER TETIKLENDI: apply_offset cagriliyor!")
                self.nav.apply_offset(offset_m=-0.18, length_m=0.60)

        # C) SHIFT GUVENLIGI: Koridor gecisinde duvara toslama kilidi
        if self.nav.state == 'SHIFT' and self.nav.front < 0.20:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        # -------------------------------------------------------------

        self.pub.publish(cmd)

        n = self.nav
        self.get_logger().info(
            f'[{n.state} | Serit {n.lanes_completed + 1}/{n.max_lanes}] '
            f'on={n.front:.2f} sol={n.left:.2f} sag={n.right:.2f} '
            f'x={n.current_x:.2f} y={n.current_y:.2f} '
            f'yaw={math.degrees(n.current_yaw):.0f}°',
            throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = LidarStandaloneTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()