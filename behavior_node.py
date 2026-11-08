"""
behavior_node: /cmd_vel'e yazan TEK node. Davranislari oncelik sirasiyla cagirir.

Oncelik:
  1. bumper_recovery  (carptiysa geri cekil, sonra serit ofseti)
  2. tilt_guard       (egim cok dikse dur; egimde yavasla)
  3. lidar_navigation (serit takibi, engelden kacis)  -> Uye A
  4. varsayilan       (ileri git; odom/scan yoksa)

Dinler:   /scan, /odom, /imu, /bumper
Yayinlar: /cmd_vel (Twist), /behavior_state (String)
"""
import copy
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

from my_robot_pkg.bumper_recovery import BumperRecovery
from my_robot_pkg.lidar_navigation import LidarNavigation
from my_robot_pkg.tilt_guard import TiltGuard

BUMP_HOLD_S = 0.3   # son temas mesaji bu sureden eskiyse temas yok sayilir


class BehaviorNode(Node):
    def __init__(self):
        super().__init__('behavior_node')

        d = self.declare_parameter
        d('default_speed', 0.2)
        d('rate_hz', 20.0)
        d('sensor_timeout', 1.0)
        d('tilt.slow_deg', 8.0)
        d('tilt.stop_deg', 18.0)
        d('tilt.min_scale', 0.5)
        d('tilt.lpf_alpha', 0.2)
        d('tilt.hysteresis_deg', 2.0)
        d('bumper.reverse_speed', -0.12)
        d('bumper.reverse_time', 1.0)
        d('bumper.offset_m', 0.18)
        d('bumper.offset_length_m', 0.80)
        d('bumper.cooldown', 1.0)

        p = self.get_parameter
        self.default_speed = p('default_speed').value
        self.timeout = p('sensor_timeout').value

        self.tilt = TiltGuard(
            slow_deg=p('tilt.slow_deg').value,
            stop_deg=p('tilt.stop_deg').value,
            min_scale=p('tilt.min_scale').value,
            lpf_alpha=p('tilt.lpf_alpha').value,
            hysteresis_deg=p('tilt.hysteresis_deg').value)
        self.bumper = BumperRecovery(
            reverse_speed=p('bumper.reverse_speed').value,
            reverse_time=p('bumper.reverse_time').value,
            offset_m=p('bumper.offset_m').value,
            offset_length_m=p('bumper.offset_length_m').value,
            cooldown=p('bumper.cooldown').value)
        self.nav = LidarNavigation()

        self.scan: Optional[LaserScan] = None
        self.imu: Optional[Imu] = None
        self.contacts: Optional[Contacts] = None
        self.t_imu = -1e9
        self.t_bump = -1e9

        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Contacts, '/bumper', self._on_bumper, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/behavior_state', 10)

        self._last_state = None
        self.create_timer(1.0 / p('rate_hz').value, self._tick)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_scan(self, msg: LaserScan):
        self.scan = msg

    def _on_odom(self, msg: Odometry):
        self.nav.update_odom(msg)

    def _on_imu(self, msg: Imu):
        self.imu = msg
        self.t_imu = self._now()

    def _on_bumper(self, msg: Contacts):
        if len(msg.contacts) > 0:
            self.contacts = msg
            self.t_bump = self._now()

    def _tick(self):
        now = self._now()
        nav = self.nav

        contacts = self.contacts if (now - self.t_bump) < BUMP_HOLD_S else None
        imu = self.imu if (now - self.t_imu) < self.timeout else None
        robot_xy = (nav.current_x, nav.current_y) if nav.odom_received else None

        state = ''
        cmd: Optional[Twist] = None

        # 1) Bumper
        cmd = self.bumper.update(contacts, now, robot_xy)
        if cmd is not None:
            state = 'BUMPER_RECOVERY'
        req = self.bumper.take_offset_request()
        if req is not None:
            nav.apply_offset(offset_m=req[0], length_m=req[1])
            self.get_logger().info(
                f'bumper: serit ofseti {req[0]:+.2f} m, {req[1]:.2f} m boyunca')

        # 2) Egim korumasi (dur)
        if cmd is None:
            cmd = self.tilt.update(imu, now)
            if cmd is not None:
                state = 'TILT_STOP'

        # 3) LiDAR navigasyonu (A)
        if cmd is None and self.scan is not None and nav.odom_received:
            cmd = nav.update(self.scan, pose=(nav.current_x, nav.current_y, nav.current_yaw),
                             now=now)
            state = 'LIDAR_DONE' if nav.state == 'DONE' else 'LIDAR_NAV'

        # 4) Varsayilan
        if cmd is None:
            cmd = Twist()
            cmd.linear.x = self.default_speed
            state = 'DEFAULT'

        # Egimde yavaslama: sadece lidar/varsayilan komutlara uygulanir.
        if state in ('LIDAR_NAV', 'DEFAULT') and self.tilt.speed_scale < 1.0:
            cmd = copy.deepcopy(cmd)
            cmd.linear.x *= self.tilt.speed_scale
            state += '+TILT_SLOW'

        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=state))
        if state != self._last_state:
            self.get_logger().info(f'davranis: {state}')
            self._last_state = state


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd_pub.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
