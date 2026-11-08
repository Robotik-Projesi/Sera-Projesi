import time
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import Contacts

class BumperRecovery:
    def __init__(self, node):
        self.node = node
        self.is_collided = False
        self.recovery_start_time = None
        self.recovery_duration = self.node.get_parameter('recovery_duration_sec').value
        self.recovery_speed = self.node.get_parameter('recovery_speed').value

    def update_contacts(self, msg: Contacts):
        if len(msg.contacts) > 0:
            self.is_collided = True
            self.recovery_start_time = time.time()

    def get_cmd_vel(self):
        if self.recovery_start_time is not None:
            elapsed = time.time() - self.recovery_start_time
            if elapsed < self.recovery_duration:
                cmd = Twist()
                cmd.linear.x = self.recovery_speed
                cmd.angular.z = 0.0
                return cmd
            else:
                self.recovery_start_time = None
                self.is_collided = False
        return None
