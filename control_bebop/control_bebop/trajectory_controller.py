"""
Planificador de trayectoria circular para Bebop 2.

Solo se activa cuando el estado es AUTOMATIC (1).
Calcula y publica setpoints de posición en /goal.
Al completar una órbita, publica /trajectory_done = True
y deja que el flight_manager decida la siguiente acción.

Además publica /goal_vel (Twist, marco de despegue) con la velocidad del
setpoint (feedforward): linear.x/y = d(goal)/dt, angular.z = d(yaw_goal)/dt.
control_PID lo suma al lazo de posición para no ir siempre "detrás" del goal.

Fases internas:
    idle       : Sin actividad (esperando AUTOMATIC)
    goto_start : Setpoint en rampa (approach_speed) hacia el punto de entrada
    settling   : Quieto en el punto de entrada (settle_time) para matar inercia
    orbiting   : Avanzando theta con ω en rampa (omega_ramp_time)
    completed  : Órbita completada — sin publicar goals
    paused     : Trayectoria en pausa (se puede reanudar)

Modos de yaw (parámetro yaw_mode):
    fixed  : mantiene el yaw que tenía al entrar a AUTOMATIC (recomendado en
             interiores: no rota, el flujo óptico y el magnetómetro no se
             degradan y el círculo no se deforma).
    lookat : apunta siempre a (lookat_x, lookat_y). Con lookat_center=True
             ese punto es el centro del círculo (target_x, target_y).

Seguimiento (orbiting): θ avanza solo si el dron va cerca del goal. Si el
error de posición o de yaw crece, ω se reduce hasta pausar θ; así la órbita
no se "completa" con el dron atrasado.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Int32, Bool


def quaternion_from_yaw(yaw):
    """Crea cuaternión a partir de ángulo yaw."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_from_quaternion(x, y, z, w):
    """Extrae yaw de un cuaternión."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def _gate(err, slow, stop):
    """1.0 si err <= slow, 0.0 si err >= stop, lineal en medio."""
    if stop <= slow:
        return 1.0 if err < stop else 0.0
    return max(0.0, min(1.0, (stop - err) / (stop - slow)))


class TrajectoryPlanner(Node):
    """Planificador de trayectoria circular para Bebop 2."""

    def __init__(self):
        super().__init__('trajectory_planner')

        # ── Parámetros de trayectoria ──
        self.declare_parameter('target_x', 1.0)        # Centro del círculo X (+1m adelante)
        self.declare_parameter('target_y', 0.0)       # Centro del círculo Y
        self.declare_parameter('circle_radius', 0.50)  # Radio (m)
        self.declare_parameter('circle_height', 1.0)   # Altura (m)
        # True: usa la altura que tenga el dron al entrar a AUTOMATIC (sin
        # escalón en z). La z del Bebop es velocidad integrada y deriva, así
        # que un circle_height absoluto no es fiable tras ~30 s de vuelo.
        self.declare_parameter('hold_entry_altitude', True)
        self.declare_parameter('circle_speed', 0.20)   # Velocidad angular (rad/s)
        self.declare_parameter('arena_half_size', 1.8)  # Mitad del lado de la arena (m)
        self.declare_parameter('safety_margin', 0.20)   # Margen respecto al borde (m)
        self.declare_parameter('frequency', 20.0)       # Frecuencia del timer (Hz)
        self.declare_parameter('start_tolerance', 0.10)  # Tolerancia goto_start (m)
        self.declare_parameter('yaw_mode', 'fixed')     # 'fixed' | 'lookat'
        self.declare_parameter('approach_speed', 0.15)  # m/s de la rampa de entrada
        self.declare_parameter('settle_time', 1.5)      # s quieto antes de orbitar
        self.declare_parameter('omega_ramp_time', 3.0)  # s para alcanzar ω
        self.declare_parameter('settle_yaw_tolerance', 0.20)  # rad para empezar órbita

        # ── Seguimiento: reducir/pausar θ si el dron se atrasa ──
        self.declare_parameter('track_err_slow', 0.12)  # m: empieza a frenar θ
        self.declare_parameter('track_err_stop', 0.30)  # m: θ detenido
        self.declare_parameter('yaw_err_slow', 0.25)    # rad (~14°)
        self.declare_parameter('yaw_err_stop', 0.60)    # rad (~34°)
        # Máxima velocidad de giro alcanzable por el dron (rad/s). En lookat
        # el yaw gira a ω, así que ω se limita a este valor.
        self.declare_parameter('max_yaw_rate', 0.15)
        # Órbita atascada: si θ queda prácticamente detenido (gate < 0.05)
        # durante este tiempo, publica /trajectory_stalled para que el
        # flight_manager pase a HOVER (hover nativo del Bebop). 0 = desactivado.
        self.declare_parameter('stall_timeout', 3.0)

        # ── Parámetros del objeto a observar (centro del círculo) ──
        self.declare_parameter('lookat_center', True)
        self.declare_parameter('lookat_x', 1.0)
        self.declare_parameter('lookat_y', 0.0)
        self.declare_parameter('lookat_z', 1.0)

        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.radius = self.get_parameter('circle_radius').value
        self.height_param = self.get_parameter('circle_height').value
        self.height = self.height_param
        self.hold_entry_alt = self.get_parameter('hold_entry_altitude').value
        self.omega = self.get_parameter('circle_speed').value
        self.arena = self.get_parameter('arena_half_size').value
        self.margin = self.get_parameter('safety_margin').value
        self.freq = self.get_parameter('frequency').value
        self.tol = self.get_parameter('start_tolerance').value
        self.yaw_mode = str(self.get_parameter('yaw_mode').value).lower()
        self.approach_speed = self.get_parameter('approach_speed').value
        self.settle_time = self.get_parameter('settle_time').value
        self.omega_ramp_time = self.get_parameter('omega_ramp_time').value
        self.settle_yaw_tol = self.get_parameter('settle_yaw_tolerance').value
        self.track_err_slow = self.get_parameter('track_err_slow').value
        self.track_err_stop = self.get_parameter('track_err_stop').value
        self.yaw_err_slow = self.get_parameter('yaw_err_slow').value
        self.yaw_err_stop = self.get_parameter('yaw_err_stop').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.stall_timeout = self.get_parameter('stall_timeout').value

        if self.get_parameter('lookat_center').value:
            self.lookat_x = self.target_x
            self.lookat_y = self.target_y
        else:
            self.lookat_x = self.get_parameter('lookat_x').value
            self.lookat_y = self.get_parameter('lookat_y').value
        self.lookat_z = self.get_parameter('lookat_z').value

        # ── Calcular radio seguro ──
        safe_arena = self.arena - self.margin
        max_safe_radius = min(
            safe_arena - abs(self.target_x),
            safe_arena - abs(self.target_y)
        )
        max_safe_radius = max(0.05, max_safe_radius)

        if self.radius > max_safe_radius:
            self.get_logger().warn(
                f'Radio {self.radius:.2f}m excede área segura. '
                f'Ajustando a {max_safe_radius:.2f}m'
            )
            self.radius = max_safe_radius

        # ── ω alcanzable: en lookat el dron debe girar a la misma ω ──
        if self.yaw_mode == 'lookat' and abs(self.omega) > self.max_yaw_rate:
            self.get_logger().warn(
                f'circle_speed={self.omega:.3f} rad/s supera max_yaw_rate='
                f'{self.max_yaw_rate:.3f} rad/s (el dron no gira tan rápido). '
                f'Limitando ω.')
            self.omega = math.copysign(self.max_yaw_rate, self.omega)

        self.get_logger().info(
            f'Trayectoria: círculo en ({self.target_x}, {self.target_y}) '
            f'r={self.radius:.2f}m h={"entrada" if self.hold_entry_alt else self.height}'
            f' ω={self.omega} rad/s '
            f'(v={abs(self.omega) * self.radius:.3f} m/s, vuelta en '
            f'{2 * math.pi / max(abs(self.omega), 1e-6):.0f}s) yaw_mode={self.yaw_mode}'
        )
        self.get_logger().info(
            f'Arena segura: ±{safe_arena:.2f}m | '
            f'X: [{self.target_x - self.radius:.2f}, '
            f'{self.target_x + self.radius:.2f}] | '
            f'Y: [{self.target_y - self.radius:.2f}, '
            f'{self.target_y + self.radius:.2f}]'
        )
        if self.yaw_mode == 'lookat':
            self.get_logger().info(
                f'Mirando hacia objeto en '
                f'({self.lookat_x}, {self.lookat_y}, {self.lookat_z})'
            )

        # ── Publishers ──
        self.pub_goal = self.create_publisher(Pose, '/goal', 10)
        self.pub_goal_vel = self.create_publisher(Twist, '/goal_vel', 10)
        self.pub_done = self.create_publisher(Bool, '/trajectory_done', 10)
        self.pub_stalled = self.create_publisher(Bool, '/trajectory_stalled', 10)

        # ── Subscribers ──
        self.sub_state = self.create_subscription(
            Int32, '/flight_state', self.state_callback, 10
        )
        self.sub_pose = self.create_subscription(
            Pose, '/bebop1/pose', self.pose_callback, 10
        )

        # ── Estado interno ──
        self.phase = 'idle'
        self.theta = 0.0
        self.th_i = 0.0
        self.orbit_started = False   # True una vez que th_i es válido
        self.phase_time = 0.0        # s en la fase actual
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.fixed_yaw = 0.0
        # Setpoint en rampa durante goto_start
        self.ref_x = 0.0
        self.ref_y = 0.0
        # Último setpoint publicado (para feedforward y error de seguimiento)
        self._last_sp = None         # (x, y, z, yaw)
        self._last_gate_log = 0.0
        self._time = 0.0
        self._stall_time = 0.0       # s seguidos con θ detenido

        # ── Timer ──
        self.dt = 1.0 / self.freq
        self.timer = self.create_timer(self.dt, self.publish_trajectory)

        self.get_logger().info(
            'Planificador listo. Esperando estado AUTOMATIC (1)...')

    # ──────────────────────────────────────────────────────────
    def _set_phase(self, phase):
        self.phase = phase
        self.phase_time = 0.0
        self._stall_time = 0.0

    def _begin_approach(self):
        """Arranca la rampa desde la posición actual hacia el punto theta."""
        self.ref_x = self.current_x
        self.ref_y = self.current_y
        self._last_sp = None
        self._set_phase('goto_start')

    def state_callback(self, msg):
        """Escucha cambios de estado desde el flight manager."""
        if msg.data == 1:  # AUTOMATIC
            if self.phase in ('idle', 'completed'):
                # Calcular ángulo inicial: punto del círculo más cercano
                dx = self.current_x - self.target_x
                dy = self.current_y - self.target_y
                self.theta = math.atan2(dy, dx)
                self.fixed_yaw = self.current_yaw
                self.orbit_started = False
                if self.hold_entry_alt:
                    # Sin escalón en z: mantener la altura actual
                    self.height = max(0.5, min(1.8, self.current_z))
                else:
                    self.height = self.height_param
                self._begin_approach()
                self.get_logger().info(
                    f'Fase GOTO_START: yendo al punto inicial '
                    f'θ₀={math.degrees(self.theta):.1f}° a '
                    f'{self.approach_speed:.2f} m/s. Tolerancia={self.tol}m '
                    f'| altura={self.height:.2f}m'
                )
            elif self.phase == 'paused':
                # Al reanudar, el dron pudo derivar en HOVER: regresar en
                # rampa al punto actual del círculo, estabilizar y seguir.
                self._begin_approach()
                self.get_logger().info(
                    'Reanudando: regresando al punto de la órbita '
                    f'θ={math.degrees(self.theta):.1f}°...')

        else:
            # Cualquier estado que no sea AUTOMATIC
            if self.phase in ('goto_start', 'settling', 'orbiting'):
                self._set_phase('paused')
                self._publish_zero_vel()
                self.get_logger().info('Trayectoria pausada')

    # ──────────────────────────────────────────────────────────
    def pose_callback(self, msg):
        """Actualiza posición actual del dron."""
        self.current_x = msg.position.x
        self.current_y = msg.position.y
        self.current_z = msg.position.z
        self.current_yaw = yaw_from_quaternion(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w)

    # ──────────────────────────────────────────────────────────
    def _clamp_xy(self, x, y):
        safe = self.arena - self.margin
        return max(-safe, min(safe, x)), max(-safe, min(safe, y))

    def _yaw_for(self, x, y):
        if self.yaw_mode == 'lookat':
            return math.atan2(self.lookat_y - y, self.lookat_x - x)
        return self.fixed_yaw

    def _goal_point(self):
        """Calcula el punto (x, y, z, yaw) del círculo en self.theta."""
        x = self.target_x + self.radius * math.cos(self.theta)
        y = self.target_y + self.radius * math.sin(self.theta)
        x, y = self._clamp_xy(x, y)
        return x, y, self.height, self._yaw_for(x, y)

    def _tracking_errors(self):
        """Error de posición y de yaw respecto al último setpoint publicado."""
        if self._last_sp is None:
            return 0.0, 0.0
        lx, ly, _, lyaw = self._last_sp
        e_pos = math.hypot(self.current_x - lx, self.current_y - ly)
        e_yaw = abs(normalize_angle(lyaw - self.current_yaw))
        return e_pos, e_yaw

    def _publish_zero_vel(self):
        self.pub_goal_vel.publish(Twist())

    # ──────────────────────────────────────────────────────────
    def publish_trajectory(self):
        """Genera y publica el siguiente setpoint."""
        self._time += self.dt
        if self.phase in ('idle', 'completed', 'paused'):
            return

        self.phase_time += self.dt
        sx, sy, z, yaw = self._goal_point()

        if self.phase == 'goto_start':
            # Mover el setpoint en rampa (evita saltos → saturación e inercia)
            ex, ey = sx - self.ref_x, sy - self.ref_y
            d = math.hypot(ex, ey)
            step = self.approach_speed * self.dt
            if d <= step:
                self.ref_x, self.ref_y = sx, sy
            else:
                self.ref_x += ex / d * step
                self.ref_y += ey / d * step
            x, y = self.ref_x, self.ref_y

            ref_arrived = (self.ref_x, self.ref_y) == (sx, sy)
            dist = math.hypot(self.current_x - sx, self.current_y - sy)
            if ref_arrived and dist < self.tol:
                self._set_phase('settling')
                self.get_logger().info(
                    f'En punto de entrada (dist={dist:.3f}m). '
                    f'Estabilizando {self.settle_time:.1f}s...')

        elif self.phase == 'settling':
            x, y = sx, sy
            dist = math.hypot(self.current_x - sx, self.current_y - sy)
            yaw_err = abs(normalize_angle(yaw - self.current_yaw))
            if dist > 2.0 * self.tol or yaw_err > self.settle_yaw_tol:
                # Fuera de tolerancia (posición o yaw): seguir esperando
                self.phase_time = 0.0
            elif self.phase_time >= self.settle_time:
                if not self.orbit_started:
                    self.th_i = self.theta
                    self.orbit_started = True
                self._set_phase('orbiting')
                self.get_logger().info('¡Iniciando órbita!')

        else:  # orbiting
            # Rampa de ω para no dar un tirón tangencial
            ramp = 1.0
            if self.omega_ramp_time > 0.0:
                ramp = min(1.0, self.phase_time / self.omega_ramp_time)
            # Frenar θ si el dron se atrasa (posición o yaw)
            e_pos, e_yaw = self._tracking_errors()
            gate = min(_gate(e_pos, self.track_err_slow, self.track_err_stop),
                       _gate(e_yaw, self.yaw_err_slow, self.yaw_err_stop))
            if gate < 1.0 and self._time - self._last_gate_log > 2.0:
                self._last_gate_log = self._time
                self.get_logger().info(
                    f'Seguimiento: e_pos={e_pos:.2f}m e_yaw='
                    f'{math.degrees(e_yaw):.0f}° → ω×{gate:.2f}')
            self.theta += self.omega * ramp * gate * self.dt
            dif = self.theta - self.th_i

            # Órbita atascada: el dron no alcanza el setpoint (deriva)
            self._stall_time = self._stall_time + self.dt if gate < 0.05 else 0.0
            if self.stall_timeout > 0.0 and self._stall_time >= self.stall_timeout:
                self.get_logger().warn(
                    f'Órbita atascada {self._stall_time:.1f}s (e_pos={e_pos:.2f}m, '
                    f'e_yaw={math.degrees(e_yaw):.0f}°) → /trajectory_stalled')
                self._stall_time = 0.0
                self.pub_stalled.publish(Bool(data=True))

            # Detectar órbita completa (soporta omega negativo)
            if abs(dif) >= 2.0 * math.pi:
                self._set_phase('completed')
                self._publish_zero_vel()
                done_msg = Bool()
                done_msg.data = True
                self.pub_done.publish(done_msg)
                self.get_logger().info(' ¡Órbita completada!')
                return  # No publicar goal — flight_manager decide

            # Recalcular con theta actualizado
            x, y, z, yaw = self._goal_point()

        x, y = self._clamp_xy(x, y)
        yaw = self._yaw_for(x, y) if self.phase != 'orbiting' else yaw

        # Feedforward: velocidad del setpoint (diferencia finita, el setpoint
        # es suave por las rampas de approach y de ω)
        vel = Twist()
        if self._last_sp is not None:
            lx, ly, lz, lyaw = self._last_sp
            vel.linear.x = (x - lx) / self.dt
            vel.linear.y = (y - ly) / self.dt
            vel.linear.z = (z - lz) / self.dt
            vel.angular.z = normalize_angle(yaw - lyaw) / self.dt
        self._last_sp = (x, y, z, yaw)

        # Construir y publicar mensaje Pose
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        goal = Pose()
        goal.position.x = x
        goal.position.y = y
        goal.position.z = z
        goal.orientation.x = qx
        goal.orientation.y = qy
        goal.orientation.z = qz
        goal.orientation.w = qw

        self.pub_goal.publish(goal)
        self.pub_goal_vel.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
