"""
Lógica de seguridad pura (sin ROS) para el Bebop 2 real.

Comandos ``cmd`` = (pitch, roll, gaz, yaw_rate) normalizados en [-1, 1],
con la misma semántica que ``geometry_msgs/Twist`` del driver:
    linear.x  → pitch    (+ adelante)
    linear.y  → roll     (+ izquierda)
    linear.z  → gaz      (+ subir)
    angular.z → yaw rate (+ CCW)
"""

import math
from dataclasses import dataclass

from control_bebop.frames import rotate2d


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


@dataclass
class GeofenceParams:
    """Parámetros de la geocerca (marco de despegue, metros)."""

    enabled: bool = True
    half_size: float = 2.0       # Área de 4x4 m → ±2.0 m
    soft_margin: float = 0.5     # A partir de |p| >= half-soft: bloquea salir
    hard_margin: float = 0.3     # A partir de |p| >= half-hard: empuja dentro
    push_cmd: float = 0.08       # Comando de empuje hacia el centro
    max_altitude: float = 2.0    # Techo absoluto
    alt_soft_margin: float = 0.3  # z >= max-soft: bloquea subir
    alt_hard_margin: float = 0.15  # z >= max-hard: fuerza bajar
    push_cmd_z: float = 0.25
    min_altitude: float = 0.4    # z <= min: bloquea bajar (usar 'land')


def apply_geofence(cmd, pose, p):
    """Filtra un comando para que el dron no salga del área permitida.

    Args:
        cmd:  (pitch, roll, gaz, yaw_rate) en marco del cuerpo.
        pose: (x, y, z, yaw) en marco de despegue.
        p:    GeofenceParams.

    Returns:
        (cmd_filtrado, lista_de_eventos)
    """
    pitch, roll, gaz, yaw_rate = cmd
    if not p.enabled:
        return (pitch, roll, gaz, yaw_rate), []

    x, y, z, yaw = pose
    events = []

    # Comando horizontal en marco de despegue
    wx, wy = rotate2d(pitch, roll, yaw)
    soft = p.half_size - p.soft_margin
    hard = p.half_size - p.hard_margin

    def _limit(w, pos, name):
        if abs(pos) >= hard:
            events.append(f'empuje_{name}')
            return -math.copysign(p.push_cmd, pos)
        if abs(pos) >= soft and w * pos > 0.0:
            events.append(f'bloqueo_{name}')
            return 0.0
        return w

    wx = _limit(wx, x, 'x')
    wy = _limit(wy, y, 'y')
    pitch, roll = rotate2d(wx, wy, -yaw)

    # Altitud
    if z >= p.max_altitude - p.alt_hard_margin:
        events.append('techo_forzar_bajar')
        gaz = -abs(p.push_cmd_z)
    elif z >= p.max_altitude - p.alt_soft_margin and gaz > 0.0:
        events.append('techo_bloqueo')
        gaz = 0.0
    elif z <= p.min_altitude and gaz < 0.0:
        events.append('piso_bloqueo')
        gaz = 0.0

    return (pitch, roll, gaz, yaw_rate), events


def scale_trim_clamp(cmd, gains, trims, limits):
    """Aplica ganancia, trim y saturación por eje.

    Args:
        cmd:    (pitch, roll, gaz, yaw_rate)
        gains:  (k_pitch, k_roll, k_gaz, k_yaw)
        trims:  (t_pitch, t_roll)   — feedforward constante
        limits: (max_xy, max_z, max_yaw)
    """
    max_xy, max_z, max_yaw = limits
    pitch = clamp(gains[0] * cmd[0] + trims[0], -max_xy, max_xy)
    roll = clamp(gains[1] * cmd[1] + trims[1], -max_xy, max_xy)
    gaz = clamp(gains[2] * cmd[2], -max_z, max_z)
    yaw_rate = clamp(gains[3] * cmd[3], -max_yaw, max_yaw)
    return pitch, roll, gaz, yaw_rate


def is_stale(now_s, last_s, timeout_s):
    """True si el último mensaje es más viejo que el timeout (o no existe)."""
    return last_s is None or (now_s - last_s) > timeout_s
