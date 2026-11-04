#!/usr/bin/env python3
"""
Nodo de teleoperación para el dron Matrice 100 usando control PS5 DualSense.

Mapeo de controles:
  Stick izquierdo Y (axis 1) → Avance/retroceso  (linear.x)
  Stick izquierdo X (axis 0) → Lateral izq/der    (linear.y)
  Cruceta ↑↓ (axis 7)        → Subir/bajar        (linear.z)
  Cruceta ←→ (axis 6)        → Rotación yaw       (angular.z)
  X  (button 0)              → Armar (para subir: R1 + cruceta ↑)
  ○  (button 1)              → Aterrizar (descenso suave)
  △  (button 2)              → Parada de emergencia
  R1 (button 5)              → Dead-man switch (mantener para mover)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class DroneTeleop(Node):
    """Nodo que traduce mensajes Joy del PS5 a comandos de vuelo del dron."""

    def __init__(self):
        super().__init__('drone_teleop')

        # ---------- Parámetros configurables ----------
        self.declare_parameter('max_linear_speed', 2.0)    # m/s lateral y avance
        self.declare_parameter('max_vertical_speed', 1.5)   # m/s subir/bajar
        self.declare_parameter('max_yaw_rate', 1.0)         # rad/s rotación
        self.declare_parameter('takeoff_speed', 0.8)        # m/s velocidad de despegue
        self.declare_parameter('land_speed', 0.4)           # m/s velocidad de aterrizaje

        # Índices de ejes y botones (PS5 DualSense en Linux - ROS joy_node)
        self.declare_parameter('axis_linear_x', 1)   # Stick izq Y  (avance/retroceso)
        self.declare_parameter('axis_linear_y', 0)   # Stick izq X  (lateral izq/der)
        self.declare_parameter('axis_linear_z', 7)   # D-pad ↑↓     (subir/bajar)
        self.declare_parameter('axis_yaw', 6)        # D-pad ←→     (rotación yaw)
        self.declare_parameter('btn_takeoff', 0)     # X (Cruz)
        self.declare_parameter('btn_land', 1)        # ○ (Círculo)
        self.declare_parameter('btn_emergency', 2)   # △ (Triángulo)
        self.declare_parameter('btn_deadman', 5)     # R1

        # ---------- Estado ----------
        self.armed = False
        self.landing = False
        self._prev_buttons = []

        # ---------- Publishers ----------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.enable_pub = self.create_publisher(Bool, '/enable', 10)

        # ---------- Subscriber ----------
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # ---------- Timer para publicar cmd_vel continuamente ----------
        self.latest_twist = Twist()
        self.create_timer(0.05, self.publish_cmd)  # 20 Hz

        self.get_logger().info('═══════════════════════════════════════════')
        self.get_logger().info('  🎮  Drone Teleop — PS5 DualSense')
        self.get_logger().info('═══════════════════════════════════════════')
        self.get_logger().info('  X  = Armar (luego R1 + cruceta ↑ para subir)')
        self.get_logger().info('  ○  = Aterrizar')
        self.get_logger().info('  △  = Parada de emergencia')
        self.get_logger().info('  R1 = Mantener para mover (dead-man)')
        self.get_logger().info('  Stick Izq = Avance/Lateral')
        self.get_logger().info('  Cruceta ↑↓ = Altura, ←→ = Yaw')
        self.get_logger().info('═══════════════════════════════════════════')
        self.get_logger().info('Estado: DESARMADO — Presiona X para armar')

    # ------------------------------------------------------------------ #
    #  Helpers para leer parámetros
    # ------------------------------------------------------------------ #
    def _p(self, name: str):
        return self.get_parameter(name).value

    def _button_pressed(self, buttons: list, index: int) -> bool:
        """Devuelve True solo en el flanco de subida (botón recién presionado)."""
        if index >= len(buttons):
            return False
        current = buttons[index] == 1
        previous = (self._prev_buttons[index] == 1
                    if index < len(self._prev_buttons) else False)
        return current and not previous

    # ------------------------------------------------------------------ #
    #  Callback del joystick
    # ------------------------------------------------------------------ #
    def joy_callback(self, msg: Joy):
        axes = msg.axes
        buttons = msg.buttons

        # --- Botones de acción (flanco de subida) ---
        if self._button_pressed(buttons, self._p('btn_takeoff')):
            self._arm_and_takeoff()

        if self._button_pressed(buttons, self._p('btn_land')):
            self._start_landing()

        if self._button_pressed(buttons, self._p('btn_emergency')):
            self._emergency_stop()

        # --- Guardar estado de botones para detección de flancos ---
        self._prev_buttons = list(buttons)

        # --- Movimiento (solo si armado y R1 presionado) ---
        deadman_idx = self._p('btn_deadman')
        deadman_held = (deadman_idx < len(buttons) and buttons[deadman_idx] == 1)

        twist = Twist()

        if self.armed and deadman_held and not self.landing:
            max_lin = self._p('max_linear_speed')
            max_vert = self._p('max_vertical_speed')
            max_yaw = self._p('max_yaw_rate')

            ax_x = self._p('axis_linear_x')
            ax_y = self._p('axis_linear_y')
            ax_z = self._p('axis_linear_z')
            ax_yaw = self._p('axis_yaw')

            def apply_dz(val):
                return 0.0 if abs(val) < 0.15 else val

            twist.linear.x = apply_dz(axes[ax_x]) * max_lin if ax_x < len(axes) else 0.0
            twist.linear.y = apply_dz(axes[ax_y]) * max_lin if ax_y < len(axes) else 0.0
            twist.linear.z = apply_dz(axes[ax_z]) * max_vert if ax_z < len(axes) else 0.0
            twist.angular.z = apply_dz(axes[ax_yaw]) * max_yaw if ax_yaw < len(axes) else 0.0

        elif self.landing:
            # Descenso suave constante
            twist.linear.z = -self._p('land_speed')

        self.latest_twist = twist

    # ------------------------------------------------------------------ #
    #  Acciones de vuelo
    # ------------------------------------------------------------------ #
    def _arm_and_takeoff(self):
        """Arma motores y aplica throttle de despegue."""
        self.armed = True
        self.landing = False

        enable_msg = Bool()
        enable_msg.data = True
        self.enable_pub.publish(enable_msg)

        # Aplica impulso de despegue inicial
        twist = Twist()
        twist.linear.z = self._p('takeoff_speed')
        self.latest_twist = twist

        self.get_logger().info('🟢 ARMADO — Despegando...')

    def _start_landing(self):
        """Inicia descenso controlado."""
        if not self.armed:
            return
        self.landing = True
        self.get_logger().info('🔵 Aterrizando — descenso suave...')

    def _emergency_stop(self):
        """Parada de emergencia: desarma motores inmediatamente."""
        self.armed = False
        self.landing = False

        enable_msg = Bool()
        enable_msg.data = False
        self.enable_pub.publish(enable_msg)

        self.latest_twist = Twist()  # Velocidades a cero

        self.get_logger().warn('🔴 PARADA DE EMERGENCIA — Motores desarmados')

    # ------------------------------------------------------------------ #
    #  Timer: publica cmd_vel continuamente
    # ------------------------------------------------------------------ #
    def publish_cmd(self):
        self.cmd_pub.publish(self.latest_twist)


def main(args=None):
    rclpy.init(args=args)
    node = DroneTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Teleop detenido por el usuario.')
    finally:
        # Desarmar al salir
        enable_msg = Bool()
        enable_msg.data = False
        node.enable_pub.publish(enable_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

