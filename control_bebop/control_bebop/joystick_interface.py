"""
Joystick Interface — Traduce PS5 DualSense a comandos del sistema.

Suscribe:
    /joy (sensor_msgs/Joy)          ← Datos crudos del control

Publica:
    /joy_commands (std_msgs/Int32)   ← Comandos de botones (flancos)
    /joy_cmd (geometry_msgs/Twist)   ← Velocidades de sticks
    /deadman_active (std_msgs/Bool)  ← Estado del deadman switch (L2)

Mapeo PS5 DualSense (defaults, configurables via parámetros):
    L2 (axis 2)  = Deadman Switch (gatillo izquierdo)
    Stick Izq Y (axis 1)  = Pitch (avance/retroceso)
    Stick Izq X (axis 0)  = Roll (lateral)
    Stick Der Y (axis 4)  = Throttle (subir/bajar)
    Stick Der X (axis 3)  = Yaw (rotación)
    × Cruz    (btn 0) = Takeoff
    ○ Círculo (btn 1) = Land
    △ Triáng. (btn 2) = Emergency Stop
    □ Cuadrado(btn 3) = Toggle Auto/Manual
    L1        (btn 4) = Return Home
    R1        (btn 5) = Iniciar trayectoria automática
    Options   (btn 9) = Reset desde Emergency
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool


# Comandos — deben coincidir con flight_manager.py
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


class JoystickInterface(Node):
    """Traduce mensajes Joy del PS5 DualSense a comandos de vuelo."""

    def __init__(self):
        super().__init__('joystick_interface')

        # ── Parámetros de botones ─────────────────────────────
        self.declare_parameter('btn_takeoff', 0)       # ×
        self.declare_parameter('btn_land', 1)           # ○
        self.declare_parameter('btn_emergency', 2)      # △
        self.declare_parameter('btn_mode_toggle', 3)    # □
        self.declare_parameter('btn_return_home', 4)    # L1
        self.declare_parameter('btn_auto_start', 5)     # R1
        self.declare_parameter('btn_flattrim', 8)       # Create / Share
        self.declare_parameter('btn_reset', 9)          # Options

        # ── Parámetros de ejes ────────────────────────────────
        self.declare_parameter('deadman_axis', 2)        # L2 trigger
        self.declare_parameter('deadman_threshold', 0.0)
        self.declare_parameter('axis_pitch', 1)          # Stick izq Y
        self.declare_parameter('axis_roll', 0)           # Stick izq X
        self.declare_parameter('axis_throttle', 7)#4       # Stick der Y
        self.declare_parameter('axis_yaw', 6)            # Stick der X 3

        # ── Parámetros de velocidad ───────────────────────────
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_vertical_speed', 0.5)
        self.declare_parameter('max_yaw_rate', 0.5)
        self.declare_parameter('stick_deadzone', 0.55)

        # ── Publishers ────────────────────────────────────────
        self.pub_cmd = self.create_publisher(Int32, '/joy_commands', 10)
        self.pub_twist = self.create_publisher(Twist, '/joy_cmd', 10)
        self.pub_deadman = self.create_publisher(Bool, '/deadman_active', 10)

        # ── Subscriber ────────────────────────────────────────
        self.create_subscription(Joy, '/joy', self._joy_callback, 10)
        self.create_subscription(Int32, '/flight_state', self._state_callback, 10)

        # ── Estado interno ────────────────────────────────────
        self._prev_buttons = []
        self._flight_state = 0
        self._last_deadman = None

        self.get_logger().info(
            '🕹️  Joystick listo: L2=deadman  ×=takeoff  ○=land  △=aterrizaje inmediato  '
            'L1+R1+△=CORTE MOTORES  □=manual/auto  Share=flattrim  Options=reset')

        # self.get_logger().info(
        #     '🕹️  Joystick Interface — PS5 DualSense')
        # self.get_logger().info(
        #     '  L2=Deadman  ×=Takeoff  ○=Land  △=Emergency')
        # self.get_logger().info(
        #     '  □=Toggle Mode  L1=Home  R1=Auto  Options=Reset')

    # ──────────────────────────────────────────────────────────
    #  Helpers
    # ──────────────────────────────────────────────────────────

    def _p(self, name):
        """Shortcut para leer un parámetro."""
        return self.get_parameter(name).value

    def _state_callback(self, msg):
        self._flight_state = msg.data

    def _button_rising_edge(self, buttons, index):
        """True solo en flanco de subida (0 → 1)."""
        if index >= len(buttons):
            return False
        current = buttons[index] == 1
        previous = (self._prev_buttons[index] == 1
                    if index < len(self._prev_buttons) else False)
        return current and not previous

    def _apply_deadzone(self, value):
        """Aplica zona muerta al valor de un eje."""
        dz = self._p('stick_deadzone')
        return 0.0 if abs(value) < dz else value

    def _safe_axis(self, axes, index):
        """Lee un eje con bounds checking."""
        return axes[index] if index < len(axes) else 0.0

    # ──────────────────────────────────────────────────────────
    #  Callback principal
    # ──────────────────────────────────────────────────────────

    def _joy_callback(self, msg):
        axes = msg.axes
        buttons = msg.buttons

        # ── 1. Deadman switch (L2) ────────────────────────────
        deadman_axis = self._p('deadman_axis')
        deadman_thresh = self._p('deadman_threshold')
        deadman_val = self._safe_axis(axes, deadman_axis)
        deadman_pressed = deadman_val < deadman_thresh

        deadman_msg = Bool()
        deadman_msg.data = deadman_pressed
        self.pub_deadman.publish(deadman_msg)
        if deadman_pressed != self._last_deadman:
            self._last_deadman = deadman_pressed
            self.get_logger().info(
                f'Deadman (L2) {"ACTIVO" if deadman_pressed else "suelto"} '
                f'[axis {deadman_axis} = {deadman_val:.2f}]')

        # ── 2. Botones (flancos de subida) ────────────────────
        if self._button_rising_edge(buttons, self._p('btn_takeoff')):
            self._send_command(CMD_TAKEOFF)
            self.get_logger().info('🎮 Takeoff')

        if self._button_rising_edge(buttons, self._p('btn_land')):
            self._send_command(CMD_LAND)
            self.get_logger().info('🎮 Land')

        if self._button_rising_edge(buttons, self._p('btn_emergency')):
            btn_l1 = self._p('btn_return_home')
            btn_r1 = self._p('btn_auto_start')
            l1_held = (btn_l1 < len(buttons) and buttons[btn_l1] == 1)
            r1_held = (btn_r1 < len(buttons) and buttons[btn_r1] == 1)
            if l1_held and r1_held:
                self._send_command(CMD_KILL)
                self.get_logger().error('🚨 COMANDO KILL: CORTE TOTAL DE MOTORES (L1+R1+△)')
            else:
                self._send_command(CMD_EMERGENCY)
                self.get_logger().warn('🎮 EMERGENCY STOP (Aterrizaje rápido solicitado)')

        if self._button_rising_edge(buttons, self._p('btn_flattrim')):
            self._send_command(CMD_FLATTRIM)
            self.get_logger().info('🎮 Solicitud de Calibración FlatTrim')

        if self._button_rising_edge(buttons, self._p('btn_auto_start')):
            self._send_command(CMD_AUTO)
            self.get_logger().info('🎮 Automatic')

        if self._button_rising_edge(buttons, self._p('btn_mode_toggle')):
            # Toggle según el estado real: MANUAL → AUTO, cualquier otro → MANUAL
            if self._flight_state == 6:   # MANUAL
                self._send_command(CMD_AUTO)
                self.get_logger().info('🎮 Modo → Automático')
            else:
                self._send_command(CMD_MANUAL)
                self.get_logger().info('🎮 Modo → Manual')

        if self._button_rising_edge(buttons, self._p('btn_return_home')):
            self._send_command(CMD_RETURN_HOME)
            self.get_logger().info('🎮 Return Home')

        if self._button_rising_edge(buttons, self._p('btn_reset')):
            self._send_command(CMD_RESET)
            self.get_logger().info('🎮 Reset')

        # ── 3. Sticks → velocidades (solo si deadman activo) ──
        twist = Twist()
        if deadman_pressed:
            max_lin = self._p('max_linear_speed')
            max_vert = self._p('max_vertical_speed')
            max_yaw = self._p('max_yaw_rate')

            twist.linear.x =(
                self._apply_deadzone(
                    self._safe_axis(axes, self._p('axis_pitch')))
                * max_lin)
            twist.linear.y = (
                self._apply_deadzone(
                    self._safe_axis(axes, self._p('axis_roll')))
                * max_lin)
            twist.linear.z = (
                self._apply_deadzone(
                    self._safe_axis(axes, self._p('axis_throttle')))
                * max_vert)
            twist.angular.z = (
                self._apply_deadzone(
                    self._safe_axis(axes, self._p('axis_yaw')))
                * max_yaw)

        self.pub_twist.publish(twist)

        # ── 4. Guardar estado de botones ──────────────────────
        self._prev_buttons = list(buttons)

    def _send_command(self, cmd):
        """Publica un comando al flight manager."""
        msg = Int32()
        msg.data = cmd
        self.pub_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickInterface()
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
