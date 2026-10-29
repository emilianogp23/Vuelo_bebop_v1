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


@dataclass
class SoftLandingParams:
    """Aterrizaje suave: descenso lento y frenado con la altura, y land al llegar al piso.

    El aterrizaje nativo del Bebop baja a ~0.7 m/s hasta el piso (bags del 27 oct):
    de ahí el golpe. La rutina baja con gaz a v_max arriba y reduce hasta v_touch al
    llegar a cut_height; ahí manda land, que ya solo recorre unos centímetros.

    La altura medida (sonar + barómetro, relativa al despegue) en el piso puede quedar en
    0.3–0.5 m (bags del 28 oct) y nunca llegar a cut_height; el dron se queda a ~10 cm
    del piso sin bajar. Por eso, bajo stall_height, si en stall_time la altura media ya
    no baja stall_descent, se da por llegado al piso y se manda land (comparar medias de
    la ventana tolera el ruido de ±0.1 m que sí reiniciaba un contador instantáneo).

    En esa zona el dron necesita ~−0.3 de gaz solo para quedarse quieto (vz≈0), y un
    control proporcional no lo vence. Con la vz de la odometría se cierra un PI de
    velocidad: si no baja a lo pedido, el gaz crece (hasta max_cmd) para atravesar la
    zona; si aun así no baja stall_confirm segundos, se manda land.
    """

    enabled: bool = True
    v_max: float = 0.25          # m/s de descenso por encima de la rampa
    v_touch: float = 0.10        # m/s al llegar a cut_height
    ramp_height: float = 0.70    # m sobre cut_height en que se va frenando
    cut_height: float = 0.08     # m: aquí se manda land (el sonar marca 0 en el piso)
    cmd_gain: float = 0.8        # m/s por unidad de gaz (medido 0.4–1.1 en los bags)
    kp: float = 0.6              # corrección de gaz por m/s de error de velocidad medida
    ki: float = 1.5              # gaz por (m/s de error · s): vence la resistencia cerca del piso
    i_max: float = 0.4           # tope del aporte integral al gaz
    max_cmd: float = 0.55        # tope de |gaz|
    slew: float = 0.5            # cambio máximo de gaz por segundo (arranque y frenado suaves)
    timeout: float = 25.0        # s: si no llega, land
    stall_height: float = 0.70   # altura medida bajo la cual un estancamiento cuenta como piso
    stall_time: float = 1.6      # s de ventana: se compara su primera mitad con la segunda
    stall_descent: float = 0.02  # m: si en la ventana bajó menos que esto, ya no baja
    stall_grace: float = 2.0     # s desde el inicio sin evaluar el estancamiento (arranque)
    stall_v: float = 0.05        # m/s: con la vz de odometría, bajar menos que esto = estancado
    stall_confirm: float = 0.5   # s seguidos estancado (empujando con gaz) antes de land
    contact_confirm: float = 0.3  # s seguidos con z ≤ cut_height (un pico del sonar dura ~0.15 s)
    fallback_speed: float = 0.15  # m/s sin altura medida
    fallback_time: float = 5.0    # s de descenso sin altura medida antes de land
    settle_time: float = 1.5      # s tras land hasta dar el aterrizaje por terminado
    vz_window: float = 1.0        # s de historia para medir la velocidad de descenso
    vz_tau: float = 0.3           # s del filtro de la vz de odometría (tiene picos de ±0.1–0.8)
    vz_stale: float = 0.5         # s sin vz de odometría → se usa la derivada de la altura


def landing_speed(z, p):
    """Velocidad de descenso deseada (m/s, positiva hacia abajo) a la altura z."""
    if z is None:
        return p.fallback_speed
    frac = clamp((z - p.cut_height) / max(p.ramp_height, 1e-6), 0.0, 1.0)
    return p.v_touch + (p.v_max - p.v_touch) * frac


class SoftLanding:
    """Rutina de aterrizaje suave. ``update(now, z, vz)`` se llama a ~20 Hz.

    ``z`` es la altura medida y ``vz`` la velocidad vertical de la odometría (m/s, + arriba);
    cualquiera puede ser None si no hay dato reciente.

    Devuelve (gaz, land_now, done, motivo): ``gaz`` es el comando vertical (≤ 0);
    ``land_now`` es True UNA vez, cuando hay que mandar land al dron; ``done`` es
    True cuando ya pasó settle_time desde el land.
    """

    def __init__(self, params):
        self.p = params
        self.t0 = None
        self.t_last = None
        self.gaz = 0.0
        self.history = []          # (t, z)
        self.land_sent_at = None
        self.stall_since = None
        self.contact_since = None
        self.v_lp = None           # vz de odometría filtrada, + hacia abajo
        self.v_lp_at = None
        self.i_term = 0.0
        self.reason = ''

    @property
    def active(self):
        return self.t0 is not None

    def start(self, now):
        self.__init__(self.p)
        self.t0 = self.t_last = now

    def descent_rate(self, now):
        """Velocidad de descenso medida (m/s, + hacia abajo) o None si falta historia."""
        old = [h for h in self.history if now - h[0] >= self.p.vz_window]
        if not old or not self.history:
            return None
        t_old, z_old = old[-1]
        t_new, z_new = self.history[-1]
        return (z_old - z_new) / (t_new - t_old) if t_new > t_old else None

    def stalled(self, now, z, v_odom):
        """True si bajo stall_height el dron ya no baja pese a empujar hacia abajo.

        Con vz de odometría: velocidad filtrada < stall_v con el gaz ya ≤ −0.15. Sin ella:
        la altura media dejó de bajar en la última ventana.
        """
        p = self.p
        if now - self.t0 < p.stall_grace:
            return False
        if v_odom is not None:
            return z <= p.stall_height and v_odom < p.stall_v and self.gaz <= -0.15
        win = [(t, z) for t, z in self.history if now - t <= p.stall_time]
        first = [z for t, z in win if now - t > p.stall_time / 2.0]
        second = [z for t, z in win if now - t <= p.stall_time / 2.0]
        if not first or not second or self.history[0][0] > now - p.stall_time:
            return False
        mean_first, mean_second = sum(first) / len(first), sum(second) / len(second)
        return (mean_first + mean_second) / 2.0 <= p.stall_height \
            and mean_first - mean_second < p.stall_descent

    def update(self, now, z, vz=None):
        p = self.p
        dt = clamp(now - self.t_last, 0.0, 0.2)
        self.t_last = now

        if self.land_sent_at is not None:           # land ya mandado: solo esperar
            self.gaz = 0.0
            return 0.0, False, now - self.land_sent_at >= p.settle_time, self.reason

        if z is not None:
            self.history.append((now, z))
            self.history = [h for h in self.history if now - h[0] <= 3.0 * p.vz_window]
        elapsed = now - self.t0
        if vz is not None:
            a = dt / (p.vz_tau + dt)
            self.v_lp = -vz if self.v_lp is None else self.v_lp + a * (-vz - self.v_lp)
            self.v_lp_at = now
        v_odom = (self.v_lp if self.v_lp_at is not None and now - self.v_lp_at <= p.vz_stale
                  else None)
        v_meas = v_odom if v_odom is not None else (
            self.descent_rate(now) if z is not None else None)

        reason = ''
        if z is None or z > p.cut_height:
            self.contact_since = None
        if elapsed >= p.timeout:
            reason = 'tiempo máximo'
        elif z is None and elapsed >= p.fallback_time:
            reason = 'sin altura medida'
        elif z is not None and z <= p.cut_height:
            self.contact_since = self.contact_since if self.contact_since is not None else now
            if now - self.contact_since >= p.contact_confirm:
                reason = 'altura de contacto'
        elif z is not None and self.stalled(now, z, v_odom):
            self.stall_since = self.stall_since if self.stall_since is not None else now
            if now - self.stall_since >= p.stall_confirm:
                reason = 'ya no baja (en el piso)'
        else:
            self.stall_since = None
        if reason:
            self.land_sent_at, self.reason = now, reason
            return 0.0, True, False, reason

        v_target = landing_speed(z, p)
        base = v_target / p.cmd_gain
        cmd = base
        if v_meas is not None:
            err = v_target - v_meas                 # baja más lento de lo pedido → más gaz
            cmd += p.kp * err + self.i_term
            if elapsed >= 1.5 and not (err > 0.0 and cmd >= p.max_cmd):   # tras el arranque
                self.i_term += p.ki * err * dt * (1.0 if err > 0.0 else 3.0)  # suelta rápido
                self.i_term = clamp(self.i_term, 0.0, p.i_max)
        target = -clamp(cmd, 0.0, p.max_cmd)
        step = p.slew * dt
        self.gaz += clamp(target - self.gaz, -step, step)
        return self.gaz, False, False, ''


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


def _crosses(a, b, box, pad):
    """True si el segmento a-b entra al interior de box (xmin, ymin, xmax, ymax) + pad."""
    lo_x, lo_y, hi_x, hi_y = box[0] - pad, box[1] - pad, box[2] + pad, box[3] + pad
    dx, dy = b[0] - a[0], b[1] - a[1]
    t0, t1 = 0.0, 1.0
    # Liang-Barsky; tocar un borde o una esquina no cuenta como cruzar
    for p, q in ((-dx, a[0] - lo_x), (dx, hi_x - a[0]), (-dy, a[1] - lo_y), (dy, hi_y - a[1])):
        if abs(p) < 1e-12:
            if q <= 0.0:
                return False
        elif p < 0.0:
            t0 = max(t0, q / p)
        else:
            t1 = min(t1, q / p)
    return t1 - t0 > 1e-9


def route_around(start, goal, boxes, margin):
    """Camino más corto de start a goal en xy que rodea las cajas prohibidas.

    boxes: [(xmin, ymin, xmax, ymax), ...]. Pasa por las esquinas de cada caja
    agrandada margin. Devuelve los waypoints sin start, el último es goal;
    sin camino posible devuelve [goal].
    """
    pad = 0.9 * margin   # las esquinas quedan a margin: los lados entre ellas no chocan

    def inside(p, box):
        return box[0] - pad < p[0] < box[2] + pad and box[1] - pad < p[1] < box[3] + pad

    def clear(a, b):
        # Un extremo ya dentro del margen (vuelo manual pegado al edificio): solo la caja real
        return not any(_crosses(a, b, box, 0.0 if inside(a, box) or inside(b, box) else pad)
                       for box in boxes)

    nodes = [tuple(start), tuple(goal)] + [
        (x, y) for b in boxes
        for x in (b[0] - margin, b[2] + margin) for y in (b[1] - margin, b[3] + margin)]
    dist, prev, todo = {0: 0.0}, {}, set(range(len(nodes)))
    while todo:
        i = min(todo, key=lambda k: dist.get(k, math.inf))
        if i == 1 or i not in dist:
            break
        todo.remove(i)
        for j in todo:
            d = dist[i] + math.dist(nodes[i], nodes[j])
            if d < dist.get(j, math.inf) and clear(nodes[i], nodes[j]):
                dist[j], prev[j] = d, i
    if 1 not in dist:
        return [tuple(goal)]
    path = [1]
    while path[-1] != 0:
        path.append(prev[path[-1]])
    return [nodes[k] for k in reversed(path[:-1])]
