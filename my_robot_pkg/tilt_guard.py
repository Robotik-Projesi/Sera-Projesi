import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class TiltGuard:
    def __init__(self, node):
        self.node = node
        self.pitch = 0.0
        self.roll = 0.0
        self.max_tilt = self.node.get_parameter('max_tilt_angle_rad').value
        self.warn_tilt = self.node.get_parameter('warning_tilt_angle_rad').value

    def update_imu(self, msg: Imu):
        q = msg.orientation
        sinr_coord = 2 * (q.w * q.x + q.y * q.z)
        cosr_coord = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.roll = math.atan2(sinr_coord, cosr_coord)

        sinp_coord = math.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
        cosp_coord = math.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
        self.pitch = 2 * math.atan2(sinp_coord, cosp_coord) - math.pi / 2

    def get_cmd_vel(self, current_cmd: Twist):
        total_tilt = math.sqrt(self.roll**2 + self.pitch**2)
        
        if total_tilt > self.max_tilt:
            return Twist(), "CRITICAL_TILT_STOP"
        elif total_tilt > self.warn_tilt and current_cmd is not None:
            cmd = Twist()
            cmd.linear.x = current_cmd.linear.x * 0.5
            cmd.angular.z = current_cmd.angular.z * 0.5
            return cmd, "TILT_SLOWDOWN"
            
        return None, None
