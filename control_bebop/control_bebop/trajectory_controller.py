"""
Planificador de trayectoria circular para Bebop 2.

Solo se activa cuando el estado es AUTOMATIC (1).
Calcula y publica setpoints de posición en /goal.
Al completar una órbita, publica /trajectory_done = True
y deja que el flight_manager decida la siguiente acción.

Fases internas:
    idle       : Sin actividad (esperando AUTOMATIC)
    goto_start : Publicando punto de entrada al círculo
    orbiting   : Avanzando theta a omega rad/s
    completed  : Órbita completada — sin publicar goals
    paused     : Trayectoria en pausa (se puede reanudar)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32, Bool


def quaternion_from_yaw(yaw):
    """Crea cuaternión a partir de ángulo yaw."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class TrajectoryPlanner(Node):
    """Planificador de trayectoria circular para Bebop 2."""

    def __init__(self):
        super().__init__('trajectory_planner')

        # ── Parámetros de trayectoria ──
        self.declare_parameter('target_x', 1.0)        # Centro del círculo X (+1m adelante)
        self.declare_parameter('target_y', 0.0)       # Centro del círculo Y (-1m derecha)
        self.declare_parameter('circle_radius', 0.15)  # Radio 15 cm -> Diámetro 30 cm
        self.declare_parameter('circle_height', 1.0)   # Altura (m)
        self.declare_parameter('circle_speed', 0.15)   # Velocidad suave (rad/s)
        self.declare_parameter('arena_half_size', 1.8)  # Mitad del lado de la arena (m)
        self.declare_parameter('safety_margin', 0.20)   # Margen respecto al borde (m)
        self.declare_parameter('frequency', 20.0)       # Frecuencia del timer (Hz)
        self.declare_parameter('start_tolerance', 0.08) # Tolerancia goto_start (m)

        # ── Parámetros del objeto a observar (centro del círculo) ──
        self.declare_parameter('lookat_x', 1.0)
        self.declare_parameter('lookat_y', 0.0)
        self.declare_parameter('lookat_z', 1.0)

        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.radius = self.get_parameter('circle_radius').value
        self.height = self.get_parameter('circle_height').value
        self.omega = self.get_parameter('circle_speed').value
        self.arena = self.get_parameter('arena_half_size').value
        self.margin = self.get_parameter('safety_margin').value
        self.freq = self.get_parameter('frequency').value
        self.tol = self.get_parameter('start_tolerance').value

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

        self.get_logger().info(
            f'Trayectoria: círculo en ({self.target_x}, {self.target_y}) '
            f'r={self.radius:.2f}m h={self.height}m ω={self.omega} rad/s'
        )
        self.get_logger().info(
            f'Arena segura: ±{safe_arena:.2f}m | '
            f'X: [{self.target_x - self.radius:.2f}, '
            f'{self.target_x + self.radius:.2f}] | '
            f'Y: [{self.target_y - self.radius:.2f}, '
            f'{self.target_y + self.radius:.2f}]'
        )
        self.get_logger().info(
            f'Mirando hacia objeto en '
            f'({self.lookat_x}, {self.lookat_y}, {self.lookat_z})'
        )

        # ── Publishers ──
        self.pub_goal = self.create_publisher(Pose, '/goal', 10)
        self.pub_done = self.create_publisher(Bool, '/trajectory_done', 10)

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
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        # ── Timer ──
        self.dt = 1.0 / self.freq
        self.timer = self.create_timer(self.dt, self.publish_trajectory)

        self.get_logger().info(
            'Planificador listo. Esperando estado AUTOMATIC (1)...')

    # ──────────────────────────────────────────────────────────
    def state_callback(self, msg):
        """Escucha cambios de estado desde el flight manager."""
        if msg.data == 1:  # AUTOMATIC
            if self.phase in ('idle', 'completed'):
                # Calcular ángulo inicial: punto del círculo más cercano
                dx = self.current_x - self.target_x
                dy = self.current_y - self.target_y
                self.theta = math.atan2(dy, dx)

                self.phase = 'goto_start'
                self.get_logger().info(
                    f'Fase GOTO_START: yendo al punto inicial '
                    f'θ₀={math.degrees(self.theta):.1f}°. '
                    f'Tolerancia={self.tol}m'
                )
            elif self.phase == 'paused':
                # Reanudar trayectoria desde donde se pausó
                self.phase = 'orbiting'
                self.get_logger().info('Reanudando órbita...')

        elif msg.data != 1:
            # Cualquier estado que no sea AUTOMATIC
            if self.phase in ('goto_start', 'orbiting'):
                self.phase = 'paused'
                self.get_logger().info('Trayectoria pausada')
            elif self.phase == 'completed':
                pass  # Mantener completed

    # ──────────────────────────────────────────────────────────
    def pose_callback(self, msg):
        """Actualiza posición actual del dron."""
        self.current_x = msg.position.x
        self.current_y = msg.position.y
        self.current_z = msg.position.z

    # ──────────────────────────────────────────────────────────
    def _goal_point(self):
        """Calcula el punto (x, y, z, yaw) del círculo en self.theta."""
        x = self.target_x + self.radius * math.cos(self.theta)
        y = self.target_y + self.radius * math.sin(self.theta)
        z = self.height

        # Clamp de emergencia
        safe = self.arena - self.margin
        x = max(-safe, min(safe, x))
        y = max(-safe, min(safe, y))

        # Yaw apuntando siempre al objeto
        yaw = math.atan2(self.lookat_y - y, self.lookat_x - x)
        return x, y, z, yaw

    # ──────────────────────────────────────────────────────────
    def publish_trajectory(self):
        """Genera y publica el siguiente setpoint."""
        if self.phase in ('idle', 'completed', 'paused'):
            return

        x, y, z, yaw = self._goal_point()

        if self.phase == 'goto_start':
            dist = math.hypot(self.current_x - x, self.current_y - y)
            if dist < self.tol:
                self.phase = 'orbiting'
                self.th_i = self.theta
                self.get_logger().info(
                    f'Dron en punto de inicio (dist={dist:.3f}m). '
                    f'¡Iniciando órbita!'
                )

        elif self.phase == 'orbiting':
            # Avanzar ángulo
            self.theta += self.omega * self.dt
            dif = self.theta - self.th_i

            # Detectar órbita completa (soporta omega negativo)
            if abs(dif) >= 2.0 * math.pi:
                self.phase = 'completed'
                done_msg = Bool()
                done_msg.data = True
                self.pub_done.publish(done_msg)
                self.get_logger().info(' ¡Órbita completada!')
                return  # No publicar goal — flight_manager decide

            # Recalcular con theta actualizado
            x, y, z, yaw = self._goal_point()

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
