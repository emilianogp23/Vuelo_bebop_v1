"""
Flight Manager — Máquina de estados maestra para Bebop 2.

Es el ÚNICO nodo autorizado para publicar /flight_state y /bebop1/enable.
Centraliza transiciones, deadman switch, y generación de goals para los
estados que no dependen del trajectory planner.

Estados:
    0  IDLE          En tierra, motores off
    1  AUTOMATIC     Trayectoria automática (goal viene de trajectory_planner)
    2  TAKING_OFF    Subiendo a altura objetivo
    3  LANDING       Descenso controlado (PID sigue goal con z decreciente)
    4  EMERGENCY     Parada inmediata, motores off
    5  HOVER         Mantiene posición actual
    6  MANUAL        Control con joystick (goal integrado de joy_cmd)
    7  RECOVERY      Regreso al centro (automático por seguridad)
    8  RETURN_HOME   Regreso a home (manual, intencional)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Int32, Bool


# ── Constantes de estado ──────────────────────────────────────
IDLE = 0
AUTOMATIC = 1
TAKING_OFF = 2
LANDING = 3
EMERGENCY = 4
HOVER = 5
MANUAL = 6
RECOVERY = 7
RETURN_HOME = 8

STATE_NAMES = {
    IDLE: 'IDLE', AUTOMATIC: 'AUTOMATIC', TAKING_OFF: 'TAKING_OFF',
    LANDING: 'LANDING', EMERGENCY: 'EMERGENCY', HOVER: 'HOVER',
    MANUAL: 'MANUAL', RECOVERY: 'RECOVERY', RETURN_HOME: 'RETURN_HOME',
}

# ── Constantes de comandos del joystick ───────────────────────
CMD_TAKEOFF = 10
CMD_LAND = 11
CMD_EMERGENCY = 12
CMD_AUTO = 13
CMD_MANUAL = 14
CMD_RECOVERY = 15
CMD_RESET = 16
CMD_RETURN_HOME = 17
CMD_FLATTRIM = 18
CMD_KILL = 19

# ── Estados que requieren motores habilitados ─────────────────
ENABLED_STATES = {AUTOMATIC, TAKING_OFF, LANDING, HOVER, MANUAL,
                  RECOVERY, RETURN_HOME}


def quaternion_from_yaw(yaw):
    """Crea cuaternión a partir de ángulo yaw."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def euler_yaw_from_quaternion(x, y, z, w):
    """Extrae yaw de un cuaternión."""
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


class FlightManager(Node):
    """Máquina de estados maestra para el Bebop 2."""

    def __init__(self):
        super().__init__('flight_manager')

        # ── Parámetros ────────────────────────────────────────
        self.declare_parameter('use_real_drone', False)
        self.declare_parameter('takeoff_height', 1.0)
        self.declare_parameter('arena_half_size', 2.0)
        self.declare_parameter('max_altitude', 2.0)
        self.declare_parameter('safety_margin', 0.20)
        self.declare_parameter('auto_land_after_orbit', True)
        self.declare_parameter('auto_land_on_home', True)
        self.declare_parameter('deadman_timeout', 0.5)
        self.declare_parameter('comm_loss_land_timeout', 3.0)
        self.declare_parameter('takeoff_settle_time', 4.0)
        self.declare_parameter('landing_settle_time', 4.5)
        self.declare_parameter('landing_rate', 0.3)
        self.declare_parameter('position_tolerance', 0.15)
        self.declare_parameter('touchdown_z', 0.08)
        self.declare_parameter('frequency', 20.0)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)

        self.use_real_drone = self.get_parameter('use_real_drone').value
        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.arena = self.get_parameter('arena_half_size').value
        self.max_alt = self.get_parameter('max_altitude').value
        self.margin = self.get_parameter('safety_margin').value
        self.auto_land_after_orbit = self.get_parameter(
            'auto_land_after_orbit').value
        self.auto_land_on_home = self.get_parameter(
            'auto_land_on_home').value
        self.deadman_timeout = self.get_parameter('deadman_timeout').value
        self.comm_loss_land_timeout = self.get_parameter(
            'comm_loss_land_timeout').value
        self.takeoff_settle_time = self.get_parameter(
            'takeoff_settle_time').value
        self.landing_settle_time = self.get_parameter(
            'landing_settle_time').value
        self.landing_rate = self.get_parameter('landing_rate').value
        self.pos_tol = self.get_parameter('position_tolerance').value
        self.touchdown_z = self.get_parameter('touchdown_z').value
        self.freq = self.get_parameter('frequency').value
        self.home_x = self.get_parameter('home_x').value
        self.home_y = self.get_parameter('home_y').value

        # ── Publishers ────────────────────────────────────────
        self.pub_state = self.create_publisher(Int32, '/flight_state', 10)
        self.pub_goal = self.create_publisher(Pose, '/goal', 10)
        self.pub_enable = self.create_publisher(Bool, '/bebop1/enable', 10)
        
        from std_msgs.msg import Empty
        self.pub_takeoff = self.create_publisher(Empty, 'cmd_takeoff', 1)
        self.pub_land = self.create_publisher(Empty, 'cmd_land', 1)
        self.pub_emergency = self.create_publisher(Empty, '/bridge/emergency', 1)
        self.pub_flattrim = self.create_publisher(Empty, '/bridge/flattrim', 1)

        # ── Subscribers ───────────────────────────────────────
        self.create_subscription(
            Int32, '/joy_commands', self._joy_cmd_callback, 10)
        self.create_subscription(
            Twist, '/joy_cmd', self._joy_twist_callback, 10)
        self.create_subscription(
            Bool, '/deadman_active', self._deadman_callback, 10)
        self.create_subscription(
            Pose, '/bebop1/pose', self._pose_callback, 10)
        self.create_subscription(
            Bool, '/trajectory_done', self._trajectory_done_callback, 10)

        # ── Estado interno ────────────────────────────────────
        self.state = IDLE
        self.dt = 1.0 / self.freq

        # Posición actual del dron
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        # Goal para estados controlados por el flight_manager
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_z = 0.0
        self.goal_yaw = 0.0

        # Landing
        self.landing_z = 0.0
        self.landing_x = 0.0
        self.landing_y = 0.0
        self.landing_yaw = 0.0

        # Deadman
        self.deadman_active = False
        self.last_deadman_time = self.get_clock().now()
        self.last_joy_msg_time = self.get_clock().now()
        self.state_enter_time = self.get_clock().now()

        # Joy manual
        self.joy_vx = 0.0
        self.joy_vy = 0.0
        self.joy_vz = 0.0
        self.joy_vyaw = 0.0

        # ── Timer principal ───────────────────────────────────
        self.timer = self.create_timer(self.dt, self._control_loop)

        # Publicar estado inicial
        self._publish_state()
        self._publish_enable(False)

        self.get_logger().info(
            '🧠 Flight Manager iniciado — Estado: IDLE — '
            f'use_real_drone={self.use_real_drone} — '
            'Esperando comando de takeoff...')

    # ══════════════════════════════════════════════════════════
    #  TRANSICIONES DE ESTADO
    # ══════════════════════════════════════════════════════════

    # Tabla de transiciones válidas
    _VALID_TRANSITIONS = {
        IDLE:        {TAKING_OFF},
        TAKING_OFF:  {HOVER, EMERGENCY},
        HOVER:       {AUTOMATIC, MANUAL, LANDING, RECOVERY,
                      RETURN_HOME, EMERGENCY},
        AUTOMATIC:   {HOVER, MANUAL, RECOVERY, RETURN_HOME, EMERGENCY},
        MANUAL:      {HOVER, AUTOMATIC, LANDING, RECOVERY,
                      RETURN_HOME, EMERGENCY},
        RECOVERY:    {HOVER, EMERGENCY},
        RETURN_HOME: {HOVER, LANDING, EMERGENCY},
        LANDING:     {IDLE, EMERGENCY},
        EMERGENCY:   {IDLE},
    }

    def _transition_to(self, new_state):
        """Transición de estado con validación."""
        if new_state == self.state:
            return

        valid = self._VALID_TRANSITIONS.get(self.state, set())
        if new_state not in valid:
            self.get_logger().warn(
                f'Transición inválida: {STATE_NAMES.get(self.state)} → '
                f'{STATE_NAMES.get(new_state)}')
            return

        old_name = STATE_NAMES.get(self.state, '?')
        new_name = STATE_NAMES.get(new_state, '?')
        self.get_logger().info(f'Estado: {old_name} → {new_name}')

        self.state = new_state
        self.state_enter_time = self.get_clock().now()
        self._publish_state()
        self._publish_enable(new_state in ENABLED_STATES)

        # ── Acciones de entrada al nuevo estado ───────────────
        if new_state == TAKING_OFF:
            self._enter_takeoff()
        elif new_state == HOVER:
            self._enter_hover()
        elif new_state == LANDING:
            self._enter_landing()
        elif new_state == RECOVERY:
            self._enter_recovery()
        elif new_state == RETURN_HOME:
            self._enter_return_home()
        elif new_state == MANUAL:
            self._enter_manual()
        elif new_state == IDLE:
            self._publish_enable(False)
        elif new_state == EMERGENCY:
            self._publish_enable(False)
            self.get_logger().warn('EMERGENCY — Motores desactivados')

    def _force_landing(self, reason):
        """Aterrizaje que ignora la tabla de transiciones (seguridad)."""
        self.get_logger().warn(f'{reason} — Estado: '
                               f'{STATE_NAMES.get(self.state)} → LANDING')
        self.state = LANDING
        self.state_enter_time = self.get_clock().now()
        self._publish_state()
        self._publish_enable(True)
        self._enter_landing()

    # ══════════════════════════════════════════════════════════
    #  ACCIONES DE ENTRADA
    # ══════════════════════════════════════════════════════════

    def _enter_takeoff(self):
        from std_msgs.msg import Empty
        if self.use_real_drone:
            self.pub_takeoff.publish(Empty())
            self.get_logger().info('🛫 Takeoff enviado al bridge (/bridge/takeoff)')
        else:
            self.get_logger().warn(
                'use_real_drone=False → NO se envía takeoff al dron real')
        self.goal_x = self.current_x
        self.goal_y = self.current_y
        self.goal_z = self.takeoff_height
        self.goal_yaw = self.current_yaw
        self.get_logger().info(
            f'Despegando a {self.takeoff_height}m...')

    def _enter_hover(self):
        self.goal_x = self.current_x
        self.goal_y = self.current_y
        self.goal_z = max(self.current_z, 0.3)
        self.goal_yaw = self.current_yaw
        self.get_logger().info(
            f'Hover en ({self.goal_x:.2f}, {self.goal_y:.2f}, '
            f'{self.goal_z:.2f})')

    def _enter_landing(self):
        from std_msgs.msg import Empty
        if self.use_real_drone:
            self.pub_land.publish(Empty())
        self.landing_x = self.current_x
        self.landing_y = self.current_y
        self.landing_z = self.current_z
        self.landing_yaw = self.current_yaw
        self.get_logger().info('Aterrizando')

    def _enter_recovery(self):
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_z = self.takeoff_height
        self.goal_yaw = 0.0
        self.get_logger().warn(
            'Regresando')

    def _enter_return_home(self):
        self.goal_x = self.home_x
        self.goal_y = self.home_y
        self.goal_z = self.takeoff_height
        self.goal_yaw = 0.0
        self.get_logger().info('Regresando a home')

    def _enter_manual(self):
        self.goal_x = self.current_x
        self.goal_y = self.current_y
        self.goal_z = max(self.current_z, 0.3)
        self.goal_yaw = self.current_yaw
        self.get_logger().info(' Control manual activado')

    # ══════════════════════════════════════════════════════════
    #  CALLBACKS
    # ══════════════════════════════════════════════════════════

    def _pose_callback(self, msg):
        self.current_x = msg.position.x
        self.current_y = msg.position.y
        self.current_z = msg.position.z
        self.current_yaw = euler_yaw_from_quaternion(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w)

    def _deadman_callback(self, msg):
        # Cualquier mensaje = el mando sigue vivo (joy_node publica a 20 Hz)
        self.last_joy_msg_time = self.get_clock().now()
        self.deadman_active = msg.data
        if msg.data:
            self.last_deadman_time = self.get_clock().now()

    def _joy_twist_callback(self, msg):
        """Velocidades de sticks del joystick para modo MANUAL."""
        self.joy_vx = msg.linear.x
        self.joy_vy = msg.linear.y
        self.joy_vz = msg.linear.z
        self.joy_vyaw = msg.angular.z

    def _joy_cmd_callback(self, msg):
        """Comandos de botones del joystick."""
        cmd = msg.data

        if cmd == CMD_TAKEOFF:
            if not self.deadman_active:
                self.get_logger().warn(
                    'Takeoff rechazado: mantén L2 (deadman) presionado')
            elif self.state != IDLE:
                self.get_logger().warn(
                    f'Takeoff ignorado: estado actual {STATE_NAMES.get(self.state)} '
                    '(debe ser IDLE; usa Options para reset si está en EMERGENCY)')
            else:
                self._transition_to(TAKING_OFF)

        elif cmd == CMD_LAND:
            if self.state in (HOVER, MANUAL, AUTOMATIC, TAKING_OFF,
                              RECOVERY, RETURN_HOME):
                self._force_landing('Aterrizaje solicitado (○)')
            elif self.use_real_drone:
                from std_msgs.msg import Empty
                self.pub_land.publish(Empty())
                self.get_logger().info('○ Land enviado (estado IDLE, por seguridad)')

        elif cmd == CMD_EMERGENCY:
            # △ = aterrizaje inmediato (NO corta motores en el aire)
            if self.use_real_drone:
                if self.state != IDLE:
                    self._force_landing('△ Parada → aterrizaje inmediato')
            elif self.state != IDLE:
                self._transition_to(EMERGENCY)

        elif cmd == CMD_KILL:
            from std_msgs.msg import Empty
            self.get_logger().error('🚨 KILL recibido: forzando EMERGENCY y apagando motores')
            if self.use_real_drone:
                self.pub_emergency.publish(Empty())
            if self.state != IDLE:
                self._transition_to(EMERGENCY)

        elif cmd == CMD_FLATTRIM:
            from std_msgs.msg import Empty
            if self.state == IDLE:
                self.get_logger().info('📐 Solicitud FlatTrim en IDLE')
                if self.use_real_drone:
                    self.pub_flattrim.publish(Empty())
            else:
                self.get_logger().warn('FlatTrim ignorado: el dron no está en tierra (IDLE)')

        elif cmd == CMD_AUTO:
            if self.deadman_active and self.state in (HOVER, MANUAL):
                self._transition_to(AUTOMATIC)

        elif cmd == CMD_MANUAL:
            if self.deadman_active and self.state in (HOVER, AUTOMATIC):
                self._transition_to(MANUAL)

        elif cmd == CMD_RETURN_HOME:
            if self.state in (HOVER, MANUAL, AUTOMATIC):
                self._transition_to(RETURN_HOME)

        elif cmd == CMD_RECOVERY:
            if self.state in (HOVER, MANUAL, AUTOMATIC):
                self._transition_to(RECOVERY)

        elif cmd == CMD_RESET:
            if self.state == EMERGENCY:
                self._transition_to(IDLE)
                self.get_logger().info(' Reset desde Emergency → IDLE')

    def _trajectory_done_callback(self, msg):
        """Señal de órbita completada del trajectory planner."""
        if msg.data and self.state == AUTOMATIC:
            self.get_logger().info('✅ Órbita completada')
            if self.auto_land_after_orbit:
                self._transition_to(RETURN_HOME)
            else:
                self._transition_to(HOVER)

    # ══════════════════════════════════════════════════════════
    #  LOOP PRINCIPAL
    # ══════════════════════════════════════════════════════════

    def _control_loop(self):
        """Timer a frecuencia configurada — actualiza estado y publica."""

        self._publish_state()

        # ── 1. Verificar deadman ──────────────────────────────
        self._check_deadman()

        # ── 2. Verificar límites de arena ─────────────────────
        self._check_arena_limits()

        # ── 3. Lógica específica por estado ───────────────────
        if self.state == IDLE or self.state == EMERGENCY:
            return

        elif self.state == TAKING_OFF:
            self._update_takeoff()

        elif self.state == HOVER:
            self._publish_goal(self.goal_x, self.goal_y,
                               self.goal_z, self.goal_yaw)

        elif self.state == AUTOMATIC:
            # El trajectory_planner publica los goals directamente.
            # El flight_manager no publica goal en este estado.
            pass

        elif self.state == MANUAL:
            self._update_manual()

        elif self.state == LANDING:
            self._update_landing()

        elif self.state == RECOVERY:
            self._update_go_to_point(
                0.0, 0.0, self.takeoff_height, 0.0,
                on_arrival_state=HOVER,
                log_msg='Recovery completado → Hover')

        elif self.state == RETURN_HOME:
            on_arrival = (LANDING if self.auto_land_on_home else HOVER)
            log = ('En home → Aterrizando'
                   if self.auto_land_on_home
                   else ' En home → Hover')
            self._update_go_to_point(
                self.home_x, self.home_y, self.takeoff_height, 0.0,
                on_arrival_state=on_arrival,
                log_msg=log)

    # ══════════════════════════════════════════════════════════
    #  LÓGICA DE ACTUALIZACIÓN POR ESTADO
    # ══════════════════════════════════════════════════════════

    def _check_deadman(self):
        """Si deadman no activo por más del timeout → HOVER / LANDING."""
        now = self.get_clock().now()
        elapsed = (now - self.last_deadman_time).nanoseconds / 1e9

        # Si estamos en vuelo activo y se suelta el deadman
        if self.state in (AUTOMATIC, MANUAL):
            if elapsed > self.deadman_timeout:
                self.get_logger().warn(' Deadman suelto → HOVER')
                self._transition_to(HOVER)

        # En dron real: si el mando deja de enviar mensajes → aterrizar
        if self.use_real_drone and self.state in (HOVER, MANUAL, AUTOMATIC):
            silence = (now - self.last_joy_msg_time).nanoseconds / 1e9
            if silence > self.comm_loss_land_timeout:
                self.get_logger().error(
                    f'🚨 Mando sin señal ({silence:.1f}s) → Aterrizando por seguridad...')
                self._transition_to(LANDING)

    def _check_arena_limits(self):
        """Si el dron sale de la arena o excede altura máxima → RECOVERY."""
        if self.state not in (AUTOMATIC, MANUAL, HOVER):
            return
        safe = self.arena - self.margin
        out_of_bounds = (abs(self.current_x) > safe or abs(self.current_y) > safe)
        too_high = self.current_z > self.max_alt

        if out_of_bounds or too_high:
            self.get_logger().warn(
                f'⚠️ Fuera de límites seguros '
                f'(x={self.current_x:.2f}, y={self.current_y:.2f}, z={self.current_z:.2f}) '
                f'→ RECOVERY')
            self._transition_to(RECOVERY)

    def _update_takeoff(self):
        """Maneja el despegue. En dron real usa temporizador nativo; en simulador usa z."""
        self._publish_goal(self.goal_x, self.goal_y,
                           self.goal_z, self.goal_yaw)

        if self.use_real_drone:
            elapsed = (self.get_clock().now() - self.state_enter_time).nanoseconds / 1e9
            if elapsed >= self.takeoff_settle_time:
                self.get_logger().info(
                    f'🛫 Despegue físico completado ({elapsed:.1f}s) → HOVER')
                self._transition_to(HOVER)
        else:
            if abs(self.current_z - self.takeoff_height) < self.pos_tol:
                self.get_logger().info(
                    f'🛫 Altura alcanzada ({self.current_z:.2f}m) → HOVER')
                self._transition_to(HOVER)

    def _update_manual(self):
        """Integra velocidades del joystick en el goal."""
        safe = self.arena - self.margin

        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)

        global_vx = self.joy_vx * cos_yaw - self.joy_vy * sin_yaw
        global_vy = self.joy_vx * sin_yaw + self.joy_vy * cos_yaw

        self.goal_x += global_vx * self.dt
        self.goal_y += global_vy * self.dt
        self.goal_z += self.joy_vz * self.dt
        self.goal_yaw += self.joy_vyaw * self.dt

        # Clamp dentro de arena y altura permitida (hasta max_alt)
        self.goal_x = max(-safe, min(safe, self.goal_x))
        self.goal_y = max(-safe, min(safe, self.goal_y))
        self.goal_z = max(0.3, min(self.max_alt, self.goal_z))

        # Normalizar yaw
        while self.goal_yaw > math.pi:
            self.goal_yaw -= 2.0 * math.pi
        while self.goal_yaw < -math.pi:
            self.goal_yaw += 2.0 * math.pi

        self._publish_goal(self.goal_x, self.goal_y,
                           self.goal_z, self.goal_yaw)

    def _update_landing(self):
        """Descenso. En dron real usa temporizador nativo de aterrizaje; en simulador usa z."""
        if self.use_real_drone:
            elapsed = (self.get_clock().now() - self.state_enter_time).nanoseconds / 1e9
            if elapsed >= self.landing_settle_time:
                self.get_logger().info('🛬 Aterrizaje completado en dron real')
                self._transition_to(IDLE)
        else:
            self.landing_z -= self.landing_rate * self.dt
            self.landing_z = max(0.0, self.landing_z)

            self._publish_goal(self.landing_x, self.landing_y,
                               self.landing_z, self.landing_yaw)

            if self.current_z < self.touchdown_z:
                self.get_logger().info(' Aterrizaje completado')
                self._transition_to(IDLE)

    def _update_go_to_point(self, tx, ty, tz, tyaw,
                            on_arrival_state, log_msg):
        """Publica goal hacia un punto. Al llegar, transiciona."""
        self._publish_goal(tx, ty, tz, tyaw)
        dist = math.hypot(self.current_x - tx, self.current_y - ty)
        if dist < self.pos_tol:
            self.get_logger().info(log_msg)
            self._transition_to(on_arrival_state)

    # ══════════════════════════════════════════════════════════
    #  PUBLICACIÓN
    # ══════════════════════════════════════════════════════════

    def _publish_state(self):
        msg = Int32()
        msg.data = self.state
        self.pub_state.publish(msg)

    def _publish_enable(self, enabled):
        msg = Bool()
        msg.data = enabled
        self.pub_enable.publish(msg)

    def _publish_goal(self, x, y, z, yaw):
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        goal = Pose()
        goal.position.x = float(x)
        goal.position.y = float(y)
        goal.position.z = float(z)
        goal.orientation.x = qx
        goal.orientation.y = qy
        goal.orientation.z = qz
        goal.orientation.w = qw
        self.pub_goal.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    node = FlightManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Desactivar motores al salir
        enable_msg = Bool()
        enable_msg.data = False
        node.pub_enable.publish(enable_msg)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
