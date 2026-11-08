"""
bumper_recovery: carpma olunca geri cekil, sonra A'nin serit takibine yanal ofset ver.

Arayuz (sozlesme): update(contacts, now, robot_xy) -> Twist | None
  - contacts: ros_gz_interfaces/Contacts veya None
  - now: saniye (sim zamani)
  - robot_xy: (x, y) odom konumu; carpma noktasinin hangi tarafta oldugunu bulur
Durum makinesi: IDLE -> REVERSE -> IDLE. Geri cekilme bitince
take_offset_request() ile (offset_m, length_m) verilir; behavior_node bunu
LidarNavigation.apply_offset() fonksiyonuna iletir. Boylece robot engele
tekrar tekrar carpmaz, serit icinde engelin yanindan kayarak gecer.
"""
from typing import Optional, Tuple

from geometry_msgs.msg import Twist

IDLE, REVERSE = 'IDLE', 'REVERSE'


class BumperRecovery:
    def __init__(self, reverse_speed: float = -0.12, reverse_time: float = 1.0,
                 offset_m: float = 0.18, offset_length_m: float = 0.80,
                 cooldown: float = 1.0):
        self.reverse_speed = reverse_speed
        self.reverse_time = reverse_time
        self.offset_m = offset_m
        self.offset_length_m = offset_length_m
        self.cooldown = cooldown
        self.state = IDLE
        self._t0 = 0.0
        self._last_end = -1e9
        self._pending: Optional[Tuple[float, float]] = None
        self._pending_side = -1.0
        self.hit_count = 0

    @staticmethod
    def _hit(contacts) -> bool:
        return contacts is not None and len(contacts.contacts) > 0

    @staticmethod
    def _contact_y(contacts) -> Optional[float]:
        """Temas noktalarinin ortalama dunya y degeri (yoksa None)."""
        ys = []
        for c in contacts.contacts:
            for p in getattr(c, 'positions', []) or getattr(c, 'position', []):
                ys.append(p.y)
        return sum(ys) / len(ys) if ys else None

    def update(self, contacts, now: float,
               robot_xy: Optional[Tuple[float, float]] = None) -> Optional[Twist]:
        if self.state == IDLE:
            if self._hit(contacts) and (now - self._last_end) > self.cooldown:
                self.state = REVERSE
                self._t0 = now
                self.hit_count += 1
                # Temas robotun neresindeyse ters yone kay (A'nin offset y ekseni dunya y).
                side = -1.0
                cy = self._contact_y(contacts)
                if cy is not None and robot_xy is not None and abs(cy - robot_xy[1]) > 0.02:
                    side = -1.0 if cy > robot_xy[1] else 1.0
                self._pending_side = side
            else:
                return None

        if self.state == REVERSE:
            if now - self._t0 < self.reverse_time:
                cmd = Twist()
                cmd.linear.x = self.reverse_speed
                return cmd
            self.state = IDLE
            self._last_end = now
            self._pending = (self._pending_side * self.offset_m, self.offset_length_m)
        return None

    def take_offset_request(self) -> Optional[Tuple[float, float]]:
        """Bekleyen (offset_m, length_m) istegini bir kez verir."""
        req, self._pending = self._pending, None
        return req
