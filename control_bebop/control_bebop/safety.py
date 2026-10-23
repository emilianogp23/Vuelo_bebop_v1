"""
Lógica de seguridad pura (sin ROS) para el Bebop 2, real y simulado.

Comandos ``cmd`` = (pitch, roll, gaz, yaw_rate) normalizados en [-1, 1],
con la misma semántica que ``geometry_msgs/Twist`` del driver:
    linear.x  → pitch    (+ adelante)
    linear.y  → roll     (+ izquierda)
    linear.z  → gaz      (+ subir)
    angular.z → yaw rate (+ CCW)
"""

import dataclasses
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
    hard_margin: float = 0.3     # A partir de |p| >= half-hard: empuja dentro
    push_cmd: float = 0.08       # Empuje mínimo hacia el centro
    push_gain: float = 0.5       # Empuje extra por metro pasado del límite duro
    # Antes del límite duro, la velocidad hacia afuera se limita a distancia/lookahead
    # y el comando hacia afuera a brake_gain·(v_permitida − v): frena con tiempo pese
    # al retardo del Bebop (0.2–0.9 s) y deja pasar la órbita junto al borde. Con los
    # modelos identificados de los bags y el piloto empujando hacia afuera, el peor
    # caso se detiene en |p| ≈ 2.02 m (test_geofence_stops_identified_bebop_models).
    lookahead: float = 2.5       # s
    brake_gain: float = 2.0
    max_push: float = 0.4        # Tope del empuje y del freno
    max_altitude: float = 2.0    # Techo absoluto
    alt_soft_margin: float = 0.3  # z >= max-soft: bloquea subir
    alt_hard_margin: float = 0.15  # z >= max-hard: fuerza bajar
    push_cmd_z: float = 0.25
    min_altitude: float = 0.4    # z <= min: bloquea bajar (usar 'land')
    # La z de ros2_bebop_driver es vz integrada y deriva (±0.3–0.9 m por vuelo).
    # Con False no se toca gaz por altura: el techo real se fija en el firmware.
    z_enabled: bool = True
    # Zona segura de despegue: círculo de este radio en el origen. Fuera de
    # ella no se baja de min_z_outside_safe (hay robots en el piso). 0 = off.
    safe_zone_radius: float = 0.5
    min_z_outside_safe: float = 0.0


def apply_geofence(cmd, pose, p, vel=None):
    """Filtra un comando para que el dron no salga del área permitida.

    Args:
        cmd:  (pitch, roll, gaz, yaw_rate) en marco del cuerpo.
        pose: (x, y, z, yaw) en marco de despegue.
        p:    GeofenceParams.
        vel:  (vx, vy) en marco de despegue, o None si no hay velocidad medida.

    Returns:
        (cmd_filtrado, lista_de_eventos)
    """
    pitch, roll, gaz, yaw_rate = cmd
    if not p.enabled:
        return (pitch, roll, gaz, yaw_rate), []

    x, y, z, yaw = pose
    vx, vy = vel if vel is not None else (0.0, 0.0)
    events = []

    # Comando horizontal en marco de despegue
    wx, wy = rotate2d(pitch, roll, yaw)
    hard = p.half_size - p.hard_margin

    def _limit(w, pos, v, name):
        out = math.copysign(1.0, pos)
        v_out = v * out
        dist = hard - abs(pos)
        if dist <= 0.0:
            events.append(f'empuje_{name}')
            push = p.push_cmd + p.push_gain * -dist
            return -out * min(p.max_push, max(push, p.brake_gain * v_out))
        allowed = p.brake_gain * (dist / p.lookahead - v_out)
        if w * out > allowed:
            events.append(f'bloqueo_{name}')
            return max(allowed, -p.max_push) * out
        return w

    wx = _limit(wx, x, vx, 'x')
    wy = _limit(wy, y, vy, 'y')

    # Bajo y en el borde de la zona segura (o fuera): no alejarse y subir
    r = math.hypot(x, y)
    low_outside = (p.z_enabled and p.min_z_outside_safe > 0.0 and z < p.min_z_outside_safe
                   and r >= p.safe_zone_radius)
    if low_outside:
        ux, uy = x / r, y / r
        outward = wx * ux + wy * uy
        if outward > 0.0:
            wx -= outward * ux
            wy -= outward * uy
            events.append('zona_segura_bloqueo')
    pitch, roll = rotate2d(wx, wy, -yaw)

    # Altitud
    if not p.z_enabled:
        pass
    elif z >= p.max_altitude - p.alt_hard_margin:
        events.append('techo_forzar_bajar')
        gaz = -abs(p.push_cmd_z)
    elif z >= p.max_altitude - p.alt_soft_margin and gaz > 0.0:
        events.append('techo_bloqueo')
        gaz = 0.0
    elif low_outside:
        events.append('zona_segura_subir')
        gaz = max(gaz, abs(p.push_cmd_z))
    elif z <= p.min_altitude and gaz < 0.0:
        events.append('piso_bloqueo')
        gaz = 0.0

    return (pitch, roll, gaz, yaw_rate), events


class CommandGate:
    """Último comando con watchdog y geocerca; lo usan real_drone_bridge y sim_odometry.

    La geocerca actúa desde el takeoff hasta el land o el corte de motores, también
    sobre los ceros del watchdog (hover). No actúa en tierra ni al aterrizar: ahí
    empujar hacia arriba (zona segura) impediría aterrizar fuera de ella.
    """

    def __init__(self, params, timeout=0.3):
        self.params = params
        self.timeout = timeout
        self.cmd = (0.0, 0.0, 0.0, 0.0)
        self.t = None
        self.flying = False

    def command(self, cmd, t):
        self.cmd = tuple(float(c) for c in cmd)
        self.t = t

    def takeoff(self):
        self.flying = True

    def land(self):
        self.flying = False

    def output(self, now, pose, vel=None, z_measured=True):
        """(comando, eventos). Ceros si el último comando es más viejo que timeout."""
        stale = self.t is None or now - self.t >= self.timeout
        cmd = (0.0, 0.0, 0.0, 0.0) if stale else self.cmd
        if not self.flying or pose is None:
            return cmd, []
        p = self.params
        if not z_measured:
            # Con la z integrada la altura mínima empujaría hacia arriba sin motivo
            p = dataclasses.replace(p, min_z_outside_safe=0.0)
        return apply_geofence(cmd, pose, p, vel)


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
