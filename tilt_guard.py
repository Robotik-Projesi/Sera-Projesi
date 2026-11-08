"""
tilt_guard: IMU'dan govde egimini hesaplar; egimde yavaslatir, cok dikse durdurur.

Arayuz (sozlesme): update(imu, now) -> Twist | None
  - Egim durdurma esigini asarsa sifir hizli Twist doner (dur).
  - Aksi halde None doner ("bu davranis devrede degil").
  - Yavaslama icin speed_scale (0..1) guncellenir. behavior_node bunu alt
    oncelikli komutun (lidar/varsayilan) dogrusal hizina uygular.

Egim, govdenin z ekseninin dunya z ekseninden sapmasidir (yaw'dan bagimsiz).
Esikler config/behavior_params.yaml icinden gelir.
"""
import math
from typing import Optional

from geometry_msgs.msg import Twist


def tilt_deg_from_quat(x: float, y: float, z: float, w: float) -> Optional[float]:
    """Kuaterniyondan toplam egim (derece). Gecersiz kuaterniyonda None."""
    n2 = x * x + y * y + z * z + w * w
    if n2 < 1e-9:
        return None
    # Govde z ekseninin dunya z bileseni: R33 = 1 - 2(x^2 + y^2) (birim kuaterniyon)
    c = 1.0 - 2.0 * (x * x + y * y) / n2
    c = max(-1.0, min(1.0, c))
    return math.degrees(math.acos(c))


class TiltGuard:
    def __init__(self, slow_deg: float = 8.0, stop_deg: float = 18.0,
                 min_scale: float = 0.5, lpf_alpha: float = 0.2,
                 hysteresis_deg: float = 2.0):
        self.slow_deg = slow_deg
        self.stop_deg = stop_deg
        self.min_scale = min_scale
        self.alpha = lpf_alpha
        self.hyst = hysteresis_deg
        self.tilt_deg = 0.0       # filtrelenmis egim
        self.speed_scale = 1.0    # 1.0 = tam hiz
        self.stopped = False
        self._have_data = False

    def update(self, imu, now: float = 0.0) -> Optional[Twist]:
        """imu: sensor_msgs/Imu. None ise (veri yok) davranis devre disi."""
        if imu is None:
            self.speed_scale = 1.0
            self.stopped = False
            return None
        q = imu.orientation
        raw = tilt_deg_from_quat(q.x, q.y, q.z, q.w)
        if raw is None:
            return None
        if not self._have_data:
            self.tilt_deg = raw
            self._have_data = True
        else:
            self.tilt_deg += self.alpha * (raw - self.tilt_deg)

        # Durma: esikte dur, histerezisle birak (titresimi onler).
        if self.stopped:
            if self.tilt_deg < self.stop_deg - self.hyst:
                self.stopped = False
        elif self.tilt_deg >= self.stop_deg:
            self.stopped = True

        # Yavaslama: slow..stop arasinda dogrusal 1.0 -> min_scale.
        if self.tilt_deg <= self.slow_deg:
            self.speed_scale = 1.0
        else:
            span = max(self.stop_deg - self.slow_deg, 1e-6)
            frac = min((self.tilt_deg - self.slow_deg) / span, 1.0)
            self.speed_scale = 1.0 - frac * (1.0 - self.min_scale)

        if self.stopped:
            self.speed_scale = 0.0
            return Twist()
        return None
