#!/usr/bin/env python3
"""
Guardián del Modo Muestra (Parrot Bebop 2): despegar y quedarse en hover nativo.

El Bebop no tiene un modo "motores en ralentí": takeoff lo hace volar a ~1 m
y con comando cero se queda en hover nativo. Este nodo solo hace eso, con los
mismos controles que el vuelo real (real_flight.launch.py):

    L2 + ×        despegar (L2 es el deadman; sin él se ignora)
    ○             aterrizar (suave: baja despacio y frena al llegar al piso)
    △             aterrizar (la misma rutina suave)
    L1 + R1 + △   corte total de motores (el dron cae)
    Share         FlatTrim (solo en tierra)
    Options       reset desde emergencia
Botones de la GUI (/demo/command): takeoff, land, emergency (= corte de motores).
El aterrizaje lo hace real_drone_bridge (SoftLanding); mientras dura, el estado es
LANDING: no se acepta despegue ni movimiento hasta que /bridge/landing vuelva a False.

Todo va por real_drone_bridge (/bridge/*), que aplica la geocerca y el watchdog.
Con allow_manual_nudge y L2 sostenido, el stick izquierdo (avance/lateral) y la
cruceta (altura/giro) mueven el dron a ≤ max_nudge_speed. Al soltar L2 vuelve el
hover nativo (comando cero). El stick derecho no se usa.
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

STATUS_IDLE = 0       # En tierra
STATUS_ARMED = 1      # En el aire, hover nativo
STATUS_LANDING = 2    # Aterrizaje suave en curso (lo ejecuta real_drone_bridge)
STATUS_EMERGENCY = 4  # Motores cortados

STATUS_TEXT = {
    STATUS_IDLE: 'IDLE (MOTORES APAGADOS)',
    STATUS_ARMED: 'MOTORES ACTIVOS: EN EL AIRE (HOVER)',
    STATUS_LANDING: 'ATERRIZANDO (SUAVE)',
    STATUS_EMERGENCY: 'EMERGENCIA ACTIVADA (MOTORES CORTADOS)',
}


def _clip(value, limit):
    return max(-limit, min(limit, value))


def compute_nudge(joy_cmd: Twist, limit: float, full_scale: float = 0.5) -> Twist:
    """Comando manual a partir de /joy_cmd, con tope ``limit`` en x, y, z y yaw.

    ``full_scale`` es el valor de /joy_cmd con el stick a fondo (max_linear_speed
    del joystick_interface). Se escala para que el recorrido completo del stick
    llegue a ``limit`` en vez de saturar a la mitad.
    """
    gain = limit / full_scale if full_scale > 0.0 else 1.0
    cmd = Twist()
    cmd.linear.x = _clip(joy_cmd.linear.x * gain, limit)
    cmd.linear.y = _clip(joy_cmd.linear.y * gain, limit)
    cmd.linear.z = _clip(joy_cmd.linear.z * gain, limit)
    cmd.angular.z = _clip(joy_cmd.angular.z * gain, limit)
    return cmd


class DemoSafetyManager(Node):
    def __init__(self):
        super().__init__('demo_safety_manager')

        self.declare_parameter('frequency', 20.0)
        self.declare_parameter('stop_on_deadman_release', False)  # True: aterriza al soltar L2
        self.declare_parameter('allow_manual_nudge', False)
        self.declare_parameter('max_nudge_speed', 0.10)
        self.declare_parameter('joy_full_scale', 0.5)   # = max_linear_speed del joystick
        self.declare_parameter('landing_max_time', 40.0)  # s: tope de espera al aterrizaje

        self.stop_on_release = self.get_parameter('stop_on_deadman_release').value
        self.allow_manual = self.get_parameter('allow_manual_nudge').value
        self.max_nudge = self.get_parameter('max_nudge_speed').value
        self.joy_full_scale = self.get_parameter('joy_full_scale').value
        self.landing_max_time = self.get_parameter('landing_max_time').value

        self.pub_takeoff = self.create_publisher(Empty, '/bridge/takeoff', 10)
        self.pub_land = self.create_publisher(Empty, '/bridge/land', 10)
        self.pub_emergency = self.create_publisher(Empty, '/bridge/emergency', 10)
        self.pub_flattrim = self.create_publisher(Empty, '/bridge/flattrim', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/bridge/cmd_vel_raw', 10)

        self.pub_demo_state = self.create_publisher(Int32, '/demo/status', 10)
        self.pub_demo_state_str = self.create_publisher(String, '/demo/status_text', 10)
        self.pub_manual_active = self.create_publisher(Bool, '/demo/manual_active', 10)

        self.create_subscription(Int32, '/joy_commands', self.joy_commands_cb, 10)
        self.create_subscription(Twist, '/joy_cmd', self.joy_cmd_cb, 10)
        self.create_subscription(Bool, '/deadman_active', self.deadman_cb, 10)
        self.create_subscription(Bool, '/bridge/landing', self.landing_cb, 10)
        self.create_subscription(String, '/demo/command', self.gui_command_cb, 10)

        self.state = STATUS_IDLE
        self.deadman_active = False
        self.last_joy_cmd = Twist()
        self.landing_t0 = 0.0
        self.landing_seen = False       # el bridge confirmó (True) que arrancó el aterrizaje

        self.timer = self.create_timer(1.0 / self.get_parameter('frequency').value, self.loop)

        self.get_logger().info(
            '🛡️ Modo muestra: L2+× despega (vuela a ~1 m y queda en hover), ○ aterriza, '
            '△ aterrizaje inmediato, L1+R1+△ corta motores')
        if self.allow_manual:
            self.get_logger().info(
                f'🕹️ Manual ON (≤ {self.max_nudge:.2f}): con L2 sostenido, stick izq = '
                'avance/lateral, cruceta = altura/giro; al soltar L2 hay hover')
        else:
            self.get_logger().info('Manual OFF: los sticks se ignoran (solo hover)')
        self._publish_state()

    def deadman_cb(self, msg: Bool):
        released = self.deadman_active and not msg.data
        self.deadman_active = msg.data
        if released and self.stop_on_release and self.state == STATUS_ARMED:
            self.get_logger().info('🛬 Deadman (L2) suelto → aterrizando')
            self.land()

    def joy_commands_cb(self, msg: Int32):
        cmd = msg.data
        if cmd == CMD_TAKEOFF:
            if self.deadman_active:
                self.takeoff()
            else:
                self.get_logger().warn('Takeoff rechazado: mantén L2 (deadman) presionado')
        elif cmd in (CMD_LAND, CMD_EMERGENCY):
            self.land()
        elif cmd == CMD_KILL:
            self.kill()
        elif cmd == CMD_FLATTRIM:
            if self.state == STATUS_IDLE:
                self.pub_flattrim.publish(Empty())
                self.get_logger().info('📐 FlatTrim enviado')
            else:
                self.get_logger().warn('FlatTrim ignorado: el dron no está en tierra')
        elif cmd == CMD_RESET and self.state == STATUS_EMERGENCY:
            self.state = STATUS_IDLE
            self.get_logger().info('Reset desde emergencia → IDLE')
            self._publish_state()

    def gui_command_cb(self, msg: String):
        text = msg.data.strip().lower()
        if text in ('takeoff', 'arm', 'start'):
            self.takeoff()
        elif text in ('land', 'stop', 'disarm'):
            self.land()
        elif text in ('emergency', 'kill', 'reset'):
            self.kill()
        elif text == 'flattrim' and self.state == STATUS_IDLE:
            self.pub_flattrim.publish(Empty())

    def joy_cmd_cb(self, msg: Twist):
        self.last_joy_cmd = msg

    def landing_cb(self, msg: Bool):
        if self.state != STATUS_LANDING:
            return
        if msg.data:
            self.landing_seen = True
        elif self.landing_seen:
            self.finish_landing('Aterrizaje suave terminado')

    def finish_landing(self, text):
        self.state = STATUS_IDLE
        self.get_logger().info(f'🛬 {text} → IDLE')
        self._publish_state()

    def takeoff(self):
        if self.state == STATUS_LANDING:
            self.get_logger().warn('Takeoff rechazado: aterrizando')
            return
        if self.state != STATUS_IDLE:
            self.get_logger().warn(f'Takeoff ignorado: {STATUS_TEXT[self.state]}')
            return
        self.pub_takeoff.publish(Empty())
        self.state = STATUS_ARMED
        self.get_logger().info('🛫 Takeoff enviado: el dron sube a ~1 m y queda en hover')
        self._publish_state()

    def land(self):
        if self.state in (STATUS_EMERGENCY, STATUS_LANDING):
            return
        self.pub_land.publish(Empty())
        if self.state == STATUS_ARMED:
            self.state = STATUS_LANDING
            self.landing_t0 = time.time()
            self.landing_seen = False
        self.get_logger().info('🛬 Land enviado (el bridge baja suave)')
        self._publish_state()

    def kill(self):
        self.pub_emergency.publish(Empty())
        self.state = STATUS_EMERGENCY
        self.get_logger().error('🚨 CORTE TOTAL DE MOTORES (/bebop/reset): el dron cae')
        self._publish_state()

    def loop(self):
        cmd = Twist()   # ceros = hover nativo
        if self.state == STATUS_LANDING:
            # Si el bridge aterrizó directo (ya en el piso) nunca manda True
            waited = time.time() - self.landing_t0
            if (not self.landing_seen and waited > 2.0) or waited > self.landing_max_time:
                self.finish_landing('Aterrizaje sin confirmación del bridge')
        manual = self.state == STATUS_ARMED and self.allow_manual and self.deadman_active
        if manual:
            cmd = compute_nudge(self.last_joy_cmd, self.max_nudge, self.joy_full_scale)
        self.pub_cmd_vel.publish(cmd)
        self.pub_manual_active.publish(Bool(data=manual))
        self._publish_state()

    def _publish_state(self):
        self.pub_demo_state.publish(Int32(data=self.state))
        self.pub_demo_state_str.publish(String(data=STATUS_TEXT[self.state]))


def main(args=None):
    rclpy.init(args=args)
    node = DemoSafetyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub_cmd_vel.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
