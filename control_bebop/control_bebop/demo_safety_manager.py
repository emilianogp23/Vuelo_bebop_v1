#!/usr/bin/env python3
"""
Nodo Guardián de Seguridad para Modo Muestra Profesional (Parrot Bebop 2).

Mejoras clave para activación confiable:
1. Permite encender motores con:
   - Presionar Deadman (L2) directamente.
   - Presionar el botón Cruz (×).
   - Hacer clic en 'ACTIVAR MOTORES' en la interfaz gráfica.
2. Envía ráfaga de comandos a /bebop/takeoff y /bridge/takeoff para evitar
   pérdida de paquetes por latencia o ruido en el Wi-Fi del dron.
3. Envía FlatTrim automático antes del despegue si el dron está en reposo.
4. Mantiene velocidad neutral continua (cmd_vel = 0) para que las hélices giren
   en reposo/ralentí sin que el dron intente elevarse con fuerza.
5. Apaga motores con:
   - Botón Círculo (○).
   - Soltar Deadman (si stop_on_deadman_release está activo).
   - Botón Triángulo (△) o Parada de Emergencia.
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Int32, String, Bool

CMD_TAKEOFF = 10
CMD_LAND = 11
CMD_EMERGENCY = 12
CMD_RESET = 16
CMD_FLATTRIM = 18
CMD_KILL = 19

STATUS_IDLE = 0       # Motores apagados
STATUS_ARMED = 1      # Motores activos en modo muestra (transmitiendo odometría)
STATUS_EMERGENCY = 4  # Corte de emergencia activo


class DemoSafetyManager(Node):
    def __init__(self):
        super().__init__('demo_safety_manager')

        self.declare_parameter('frequency', 20.0)
        self.declare_parameter('stop_on_deadman_release', False)  # Si es True, apaga al soltar L2
        self.declare_parameter('allow_manual_nudge', False)
        self.declare_parameter('max_nudge_speed', 0.10)

        self.freq = self.get_parameter('frequency').value
        self.stop_on_release = self.get_parameter('stop_on_deadman_release').value
        self.allow_manual = self.get_parameter('allow_manual_nudge').value
        self.max_nudge = self.get_parameter('max_nudge_speed').value

        # ── Publishers al driver del Bebop ────────────────────────
        self.pub_takeoff = self.create_publisher(Empty, '/bebop/takeoff', 10)
        self.pub_land = self.create_publisher(Empty, '/bebop/land', 10)
        self.pub_reset = self.create_publisher(Empty, '/bebop/reset', 10)
        self.pub_flattrim = self.create_publisher(Empty, '/bebop/flattrim', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/bebop/cmd_vel', 10)

        # ── Publishers al puente interno (marco de despegue) ─────
        self.pub_bridge_takeoff = self.create_publisher(Empty, '/bridge/takeoff', 10)
        self.pub_bridge_land = self.create_publisher(Empty, '/bridge/land', 10)
        self.pub_bridge_emergency = self.create_publisher(Empty, '/bridge/emergency', 10)

        # ── Publisher de estado del modo muestra ──────────────────
        self.pub_demo_state = self.create_publisher(Int32, '/demo/status', 10)
        self.pub_demo_state_str = self.create_publisher(String, '/demo/status_text', 10)

        # ── Subscribers ──────────────────────────────────────────
        self.create_subscription(Int32, '/joy_commands', self.joy_commands_cb, 10)
        self.create_subscription(Twist, '/joy_cmd', self.joy_cmd_cb, 10)
        self.create_subscription(Bool, '/deadman_active', self.deadman_cb, 10)
        self.create_subscription(String, '/demo/command', self.gui_command_cb, 10)

        # ── Estado ───────────────────────────────────────────────
        self.state = STATUS_IDLE
        self.deadman_active = False
        self.last_joy_cmd = Twist()
        self.last_action_time = time.time()
        self._takeoff_burst_count = 0

        # Timer cíclico para mantener velocidad neutral garantizada
        self.timer = self.create_timer(1.0 / self.freq, self.loop)

        self.get_logger().info('🛡️ Demo Safety Manager listo y armado.')
        self.get_logger().info('   Para encender motores: Presiona Deadman (L2), botón Cruz (×) o botón GUI.')
        self.get_logger().info('   Para apagar motores: Botón Círculo (○) o Emergencia (△).')
        self._publish_state()

    def deadman_cb(self, msg: Bool):
        was_active = self.deadman_active
        self.deadman_active = msg.data

        # ── DISPARO DE ENCENDIDO CON DEADMAN (L2) ──
        if self.deadman_active and not was_active:
            self.get_logger().info(f'🕹️ Deadman (L2) PRESIONADO [Estado actual: {self.state}]')
            if self.state == STATUS_IDLE:
                self.get_logger().info('⚡ Encendiendo motores por Deadman (L2)...')
                self.arm_motors()

        # ── SI SE CONFIGURA APAGAR AL SOLTAR DEADMAN ──
        elif not self.deadman_active and was_active:
            self.get_logger().info('🕹️ Deadman (L2) SUELTO')
            if self.stop_on_release and self.state == STATUS_ARMED:
                self.get_logger().info('🛬 Apagando motores por soltar Deadman (L2)...')
                self.disarm_motors()

    def joy_commands_cb(self, msg: Int32):
        cmd = msg.data
        if cmd == CMD_TAKEOFF:
            self.get_logger().info('🎮 Botón Cruz (×) detectado → Encendiendo motores...')
            self.arm_motors()
        elif cmd == CMD_LAND:
            self.get_logger().info('🎮 Botón Círculo (○) detectado → Apagando motores...')
            self.disarm_motors()
        elif cmd in (CMD_EMERGENCY, CMD_KILL):
            self.get_logger().error('🚨 Emergencia / Parada solicitada (△) → Corte inmediato...')
            self.emergency_stop()
        elif cmd == CMD_FLATTRIM:
            self.get_logger().info('📐 Solicitud FlatTrim enviada')
            self.pub_flattrim.publish(Empty())
        elif cmd == CMD_RESET:
            self.state = STATUS_IDLE
            self.get_logger().info('Estado reseteado a IDLE.')
            self._publish_state()

    def gui_command_cb(self, msg: String):
        text = msg.data.strip().lower()
        if text in ('takeoff', 'arm', 'start'):
            self.get_logger().info('💻 Comando GUI: Encender motores')
            self.arm_motors()
        elif text in ('land', 'stop', 'disarm'):
            self.get_logger().info('💻 Comando GUI: Apagar motores')
            self.disarm_motors()
        elif text in ('emergency', 'kill', 'reset'):
            self.get_logger().error('💻 Comando GUI: Corte de emergencia')
            self.emergency_stop()
        elif text == 'flattrim':
            self.pub_flattrim.publish(Empty())

    def joy_cmd_cb(self, msg: Twist):
        self.last_joy_cmd = msg

    def arm_motors(self):
        # 1. Enviar FlatTrim breve para calibrar sensores en reposo
        self.pub_flattrim.publish(Empty())

        # 2. Enviar ráfaga de Takeoff (3 pulsos) tanto a /bebop/takeoff como a /bridge/takeoff
        for _ in range(3):
            self.pub_takeoff.publish(Empty())
            self.pub_bridge_takeoff.publish(Empty())
            time.sleep(0.02)

        self.state = STATUS_ARMED
        self.last_action_time = time.time()
        self.get_logger().info('🟢 MOTORES ARMADOS: Comando Takeoff transmitido con éxito.')
        self._publish_state()

    def disarm_motors(self):
        for _ in range(3):
            self.pub_land.publish(Empty())
            self.pub_bridge_land.publish(Empty())
            time.sleep(0.02)

        self.state = STATUS_IDLE
        self.last_action_time = time.time()
        self.get_logger().info('🔴 MOTORES DETENIDOS (Land transmitido).')
        self._publish_state()

    def emergency_stop(self):
        for _ in range(3):
            self.pub_reset.publish(Empty())
            self.pub_bridge_emergency.publish(Empty())
            time.sleep(0.02)

        self.state = STATUS_EMERGENCY
        self.last_action_time = time.time()
        self.get_logger().error('🚨 CORTE TOTAL DE EMERGENCIA: Motores desenergizados (/bebop/reset).')
        self._publish_state()

    def loop(self):
        cmd = Twist()

        if self.state == STATUS_ARMED:
            # Control suave si allow_manual y deadman activo
            if self.allow_manual and self.deadman_active:
                cmd.linear.x = max(-self.max_nudge, min(self.max_nudge, self.last_joy_cmd.linear.x))
                cmd.linear.y = max(-self.max_nudge, min(self.max_nudge, self.last_joy_cmd.linear.y))
                cmd.linear.z = 0.0  # Nunca aceleración vertical en modo muestra
                cmd.angular.z = max(-self.max_nudge, min(self.max_nudge, self.last_joy_cmd.angular.z))
            else:
                # Hover neutral absoluto (ceros)
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.linear.z = 0.0
                cmd.angular.z = 0.0

            self.pub_cmd_vel.publish(cmd)
        elif self.state == STATUS_IDLE:
            self.pub_cmd_vel.publish(cmd)

        self._publish_state()

    def _publish_state(self):
        s_msg = Int32()
        s_msg.data = self.state
        self.pub_demo_state.publish(s_msg)

        txt_msg = String()
        if self.state == STATUS_ARMED:
            txt_msg.data = "MOTORES ACTIVOS (ODOMETRÍA EN VIVO)"
        elif self.state == STATUS_EMERGENCY:
            txt_msg.data = "EMERGENCIA ACTIVADA (MOTORES CORTADOS)"
        else:
            txt_msg.data = "IDLE (MOTORES APAGADOS)"
        self.pub_demo_state_str.publish(txt_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DemoSafetyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd_vel.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
