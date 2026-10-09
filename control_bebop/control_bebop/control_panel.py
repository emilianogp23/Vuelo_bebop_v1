"""
Panel de control interactivo por terminal para Bebop 2.

Alternativa al control PS5 cuando no hay joystick disponible.
Publica comandos al flight_manager via /joy_commands.

Uso:
    ros2 run control_bebop control_panel
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import threading


# Comandos — deben coincidir con flight_manager.py
CMD_TAKEOFF = 10
CMD_LAND = 11
CMD_EMERGENCY = 12
CMD_AUTO = 13
CMD_MANUAL = 14
CMD_RECOVERY = 15
CMD_RESET = 16
CMD_RETURN_HOME = 17

STATE_NAMES = {
    0: 'IDLE', 1: 'AUTOMATIC', 2: 'TAKING_OFF', 3: 'LANDING',
    4: 'EMERGENCY', 5: 'HOVER', 6: 'MANUAL', 7: 'RECOVERY',
    8: 'RETURN_HOME',
}

HELP_TEXT = """
╔══════════════════════════════════════════════════╗
║        PANEL DE CONTROL - BEBOP 2               ║
╠══════════════════════════════════════════════════╣
║  t  → Takeoff (despegue)                        ║
║  s  → Start trajectory (trayectoria automática)  ║
║  m  → Manual mode (control manual)               ║
║  l  → Land (aterrizar)                           ║
║  r  → Return Home (regresar a home)              ║
║  e  → Emergency Stop (parada de emergencia)      ║
║  x  → Reset (salir de Emergency → IDLE)          ║
║  h  → Help (mostrar este menú)                   ║
║  q  → Quit (salir)                               ║
╚══════════════════════════════════════════════════╝

Nota: El deadman switch se simula como siempre activo
desde el panel de control (para volar sin joystick).
"""


class ControlPanel(Node):
    """Panel de control interactivo por terminal."""

    def __init__(self):
        super().__init__('control_panel')

        # Publicar comandos al flight manager
        self.pub_cmd = self.create_publisher(Int32, '/joy_commands', 10)

        # Simular deadman siempre activo (para uso sin joystick)
        from std_msgs.msg import Bool
        self.pub_deadman = self.create_publisher(Bool, '/deadman_active', 10)
        self.deadman_timer = self.create_timer(0.05, self._publish_deadman)

        # Suscribirse al estado para mostrar en terminal
        self.sub_state = self.create_subscription(
            Int32, '/flight_state', self._state_callback, 10)
        self.current_state = 0

        self.get_logger().info('Panel de control iniciado')

    def _publish_deadman(self):
        """Simula deadman siempre activo."""
        from std_msgs.msg import Bool
        msg = Bool()
        msg.data = True
        self.pub_deadman.publish(msg)

    def _state_callback(self, msg):
        """Actualiza estado actual."""
        if msg.data != self.current_state:
            self.current_state = msg.data
            name = STATE_NAMES.get(msg.data, 'UNKNOWN')
            print(f'\n  Estado actual: {name} ({msg.data})')

    def send_command(self, cmd):
        """Publica un comando al flight manager."""
        msg = Int32()
        msg.data = cmd
        self.pub_cmd.publish(msg)


def input_thread(panel, stop_event):
    """Hilo separado para leer input del usuario."""
    print(HELP_TEXT)
    print('Estado actual: IDLE (0)')
    print('Escribe un comando y presiona Enter:\n')

    while not stop_event.is_set():
        try:
            key = input('>> ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if key == 't':
            print('🛫 TAKEOFF')
            panel.send_command(CMD_TAKEOFF)
        elif key == 's':
            print('📐 AUTOMATIC')
            panel.send_command(CMD_AUTO)
        elif key == 'm':
            print('🕹️  MANUAL')
            panel.send_command(CMD_MANUAL)
        elif key == 'l':
            print('🔵 LANDING')
            panel.send_command(CMD_LAND)
        elif key == 'r':
            print('🏠 RETURN HOME')
            panel.send_command(CMD_RETURN_HOME)
        elif key == 'e':
            print('🔴 EMERGENCY STOP!')
            panel.send_command(CMD_EMERGENCY)
        elif key == 'x':
            print('🔄 RESET')
            panel.send_command(CMD_RESET)
        elif key == 'h':
            print(HELP_TEXT)
        elif key == 'q':
            print('Saliendo del panel...')
            stop_event.set()
            break
        else:
            print(f'Comando "{key}" no reconocido. Escribe "h" para ayuda.')


def main(args=None):
    rclpy.init(args=args)
    panel = ControlPanel()

    stop_event = threading.Event()

    # Lanzar hilo de input
    thread = threading.Thread(
        target=input_thread, args=(panel, stop_event), daemon=True)
    thread.start()

    try:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(panel, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        panel.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
