"""
Utilidades de marcos de referencia para el Bebop 2 real.

La odometría de ``ros2_bebop_driver`` integra la velocidad que reporta el
dron (NED magnético) y la publica con convención:
    x = Norte, y = Oeste, z = Arriba, yaw = rumbo CCW desde el Norte.

El resto de ``control_bebop`` trabaja en un "marco de despegue":
    origen = punto donde despegó el dron
    x      = hacia donde apuntaba el frente del dron al despegar
    y      = a la izquierda del dron al despegar
    z      = arriba (0 = piso)

Este módulo no depende de ROS para poder probarse con pytest.
"""

import math


def normalize_angle(angle):
    """Normaliza un ángulo a [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x, y, z, w):
    """Extrae el yaw de un cuaternión."""
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


def quaternion_from_yaw(yaw):
    """Cuaternión (x, y, z, w) a partir de un yaw."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def rotate2d(x, y, angle):
    """Rota el vector (x, y) un ángulo ``angle`` (CCW)."""
    c = math.cos(angle)
    s = math.sin(angle)
    return c * x - s * y, s * x + c * y


class TakeoffFrame:
    """Transforma odometría del driver al marco de despegue.

    Además aplica correcciones de calibración opcionales:
        - ``scale``: factor por eje (x, y, z) en el marco de despegue.
        - ``vel_bias``: sesgo de velocidad (m/s) por eje (x, y) en el marco
          de despegue que se resta integrado en el tiempo (error de la
          odometría medido con cinta, NO deriva real del dron).
    """

    def __init__(self, scale=(1.0, 1.0, 1.0), vel_bias=(0.0, 0.0)):
        self.scale = tuple(float(s) for s in scale)
        self.vel_bias = tuple(float(b) for b in vel_bias)
        self.origin = None      # (x0, y0, z0, yaw0) en marco odom
        self.t0 = None          # segundos

    @property
    def is_set(self):
        return self.origin is not None

    def set_origin(self, x, y, z, yaw, t=0.0):
        """Fija el origen del marco (llamar al iniciar el despegue)."""
        self.origin = (float(x), float(y), float(z), float(yaw))
        self.t0 = float(t)

    def pose_to_local(self, x, y, z, yaw, t=None):
        """Pose en marco odom → (x, y, z, yaw) en marco de despegue."""
        if self.origin is None:
            self.set_origin(x, y, z, yaw, 0.0 if t is None else t)
        x0, y0, z0, yaw0 = self.origin
        lx, ly = rotate2d(x - x0, y - y0, -yaw0)
        lz = z - z0
        lx *= self.scale[0]
        ly *= self.scale[1]
        lz *= self.scale[2]
        if t is not None and self.t0 is not None:
            dt = max(0.0, t - self.t0)
            lx -= self.vel_bias[0] * dt
            ly -= self.vel_bias[1] * dt
        return lx, ly, lz, normalize_angle(yaw - yaw0)

    def body_vel_to_local(self, vx_b, vy_b, vz, yaw_local):
        """Velocidad en marco del cuerpo → marco de despegue (corregida)."""
        wx, wy = rotate2d(vx_b, vy_b, yaw_local)
        wx = wx * self.scale[0] - self.vel_bias[0]
        wy = wy * self.scale[1] - self.vel_bias[1]
        return wx, wy, vz * self.scale[2]
