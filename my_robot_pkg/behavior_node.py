import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from ros_gz_interfaces.msg import Contacts

from my_robot_pkg.bumper_recovery import BumperRecovery
from my_robot_pkg.tilt_guard import TiltGuard
from my_robot_pkg.lidar_navigation import LidarNavigation

class BehaviorNode(Node):
    def __init__(self):
        super().__init__('behavior_node')

        self.declare_parameter('max_tilt_angle_rad', 0.35)
        self.declare_parameter('warning_tilt_angle_rad', 0.15)
        self.declare_parameter('recovery_duration_sec', 1.5)
        self.declare_parameter('recovery_speed', -0.2)
        self.declare_parameter('default_forward_speed', 0.3)

        self.bumper_recovery = BumperRecovery(self)
        self.tilt_guard = TiltGuard(self)
        self.lidar_nav = LidarNavigation(self)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/behavior_state', 10)

        self.create_subscription(Contacts, '/bumper', self.bumper_recovery.update_contacts, 10)
        self.create_subscription(Imu, '/imu', self.tilt_guard.update_imu, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_nav.update_scan, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

    def control_loop(self):
        cmd = None
        state = "DEFAULT_FORWARD"

        # 1. Bumper Recovery
        cmd = self.bumper_recovery.get_cmd_vel()
        if cmd is not None:
            state = "BUMPER_RECOVERY"

        # 2. Tilt Guard
        if cmd is None:
            nav_cmd = self.lidar_nav.get_cmd_vel()
            tilt_cmd, tilt_state = self.tilt_guard.get_cmd_vel(nav_cmd)
            if tilt_cmd is not None:
                cmd = tilt_cmd
                state = tilt_state
            else:
                cmd = nav_cmd
                if cmd is not None:
                    state = "LIDAR_NAVIGATION"

        # 3. İleri Git
        if cmd is None:
            cmd = Twist()
            cmd.linear.x = self.get_parameter('default_forward_speed').value
            state = "DEFAULT_FORWARD"

        self.cmd_pub.publish(cmd)
        
        state_msg = String()
        state_msg.data = state
        self.state_pub.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
