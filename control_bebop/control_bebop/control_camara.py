#!/usr/bin/env python3
"""
Nodo de Control de Cámara Ojo de Pez (Gimbal Digital) para Parrot Bebop 2.

Permite orientar el lente ojo de pez del dron mediante:
1. Control de mando PS5 DualSense (Cruceta / D-Pad y botones configurables).
2. Tópicos de ROS 2:
   - /bebop/camera/set_tilt (std_msgs/Float32)
   - /bebop/camera/set_pan  (std_msgs/Float32)
   - /bebop/camera/preset   (std_msgs/String: 'horizon', 'people', 'down', 'floor', 'center')
3. Publicación continua o por evento hacia:
   - /bebop/move_camera (geometry_msgs/Vector3)
     * x: tilt (grados: -85.0° a +17.0°)
     * y: pan  (grados: -35.0° a +35.0°)
     * z: 0.0

Límites de hardware del Parrot Bebop 2:
  Tilt (inclinación vertical):
    -85° = Suelo vertical
     0°  = Frente horizontal (Horizonte)
    +17° = Hacia arriba (máximo)
  Pan (giro horizontal):
    -35° (Izquierda) a +35° (Derecha)
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, String


PRESETS = {
    'horizon': (0.0, 0.0),
    'front': (0.0, 0.0),
    'center': (0.0, 0.0),
    'people': (10.0, 0.0),
    'desk': (-15.0, 0.0),
    'down': (-35.0, 0.0),
    'floor': (-75.0, 0.0),
}


class ControlCamaraBebop(Node):
    def __init__(self):
        super().__init__('control_camara')

        # ── Parámetros ────────────────────────────────────────────────────────
        self.declare_parameter('tilt_default', 0.0)      # 0° = mirando al frente
        self.declare_parameter('pan_default', 0.0)       # 0° = centro
        self.declare_parameter('tilt_step', 5.0)         # Incremento en grados
        self.declare_parameter('pan_step', 5.0)
        self.declare_parameter('publish_rate', 2.0)      # Hz de refresco continuo
        self.declare_parameter('dpad_axis_x', 6)         # D-Pad horizontal PS5
        self.declare_parameter('dpad_axis_y', 7)         # D-Pad vertical PS5
        self.declare_parameter('btn_reset_center', 10)   # Botón R3 o PS

        self.current_tilt = float(self.get_parameter('tilt_default').value)
        self.current_pan = float(self.get_parameter('pan_default').value)
        self.tilt_step = float(self.get_parameter('tilt_step').value)
        self.pan_step = float(self.get_parameter('pan_step').value)
        pub_rate = float(self.get_parameter('publish_rate').value)

        # ── Publicador a driver del Bebop ────────────────────────────────────
        self.pub_move_cam = self.create_publisher(Vector3, '/bebop/move_camera', 10)

        # ── Suscripciones para control remoto y UI ────────────────────────────
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(Float32, '/bebop/camera/set_tilt', self._set_tilt_cb, 10)
        self.create_subscription(Float32, '/bebop/camera/set_pan', self._set_pan_cb, 10)
        self.create_subscription(String, '/bebop/camera/preset', self._preset_cb, 10)

        # ── Timer de publicación periódica ───────────────────────────────────
        # Mantiene la orientación de la cámara activa en el firmware del Bebop
        timer_period = 1.0 / max(0.5, pub_rate)
        self.timer = self.create_timer(timer_period, self._publish_camera)

        # Estado para detección de pulsaciones en D-Pad
        self._prev_dpad_x = 0.0
        self._prev_dpad_y = 0.0
        self._last_btn_reset = 0

        self.get_logger().info(
            f"📹 [Control Cámara Bebop] Inicializado. Tilt inicial: {self.current_tilt:.1f}°, Pan inicial: {self.current_pan:.1f}°"
        )
        self.get_logger().info(
            "🎮 Mando PS5: D-Pad Arriba/Abajo = Inclinación | D-Pad Izq/Der = Giro | R3 = Centrar horizonte (0°, 0°)"
        )
        self.get_logger().info(
            "📡 Publicando periódicamente en: /bebop/move_camera"
        )

        # Enviar de inmediato la posición inicial para forzar alineación
        self._publish_camera()

    def _clamp(self, val, min_val, max_val):
        return max(min_val, min(max_val, val))

    def _publish_camera(self):
        msg = Vector3()
        # Límites seguros de hardware Bebop 2: Tilt [-85°, +17°], Pan [-35°, +35°]
        msg.x = float(self._clamp(self.current_tilt, -85.0, 17.0))
        msg.y = float(self._clamp(self.current_pan, -35.0, 35.0))
        msg.z = 0.0
        self.pub_move_cam.publish(msg)

    def set_orientation(self, tilt: float, pan: float):
        self.current_tilt = self._clamp(tilt, -85.0, 17.0)
        self.current_pan = self._clamp(pan, -35.0, 35.0)
        self._publish_camera()
        self.get_logger().info(
            f"🎥 Cámara orientada -> Tilt: {self.current_tilt:+.1f}° | Pan: {self.current_pan:+.1f}°"
        )

    def _set_tilt_cb(self, msg: Float32):
        self.set_orientation(msg.data, self.current_pan)

    def _set_pan_cb(self, msg: Float32):
        self.set_orientation(self.current_tilt, msg.data)

    def _preset_cb(self, msg: String):
        name = msg.data.strip().lower()
        if name in PRESETS:
            t, p = PRESETS[name]
            self.set_orientation(t, p)
            self.get_logger().info(f"🎯 Preset de cámara aplicado: '{name}' -> Tilt: {t}°, Pan: {p}°")
        else:
            self.get_logger().warn(f"⚠️ Preset desconocido: '{name}'. Opciones: {list(PRESETS.keys())}")

    def _joy_cb(self, msg: Joy):
        ax_x = int(self.get_parameter('dpad_axis_x').value)
        ax_y = int(self.get_parameter('dpad_axis_y').value)
        btn_reset = int(self.get_parameter('btn_reset_center').value)

        # Leer D-Pad
        dpad_x = msg.axes[ax_x] if ax_x < len(msg.axes) else 0.0
        dpad_y = msg.axes[ax_y] if ax_y < len(msg.axes) else 0.0

        changed = False

        # D-Pad Vertical (Tilt): +1.0 = Arriba, -1.0 = Abajo
        if dpad_y > 0.5 and self._prev_dpad_y <= 0.5:
            self.current_tilt = self._clamp(self.current_tilt + self.tilt_step, -85.0, 17.0)
            changed = True
        elif dpad_y < -0.5 and self._prev_dpad_y >= -0.5:
            self.current_tilt = self._clamp(self.current_tilt - self.tilt_step, -85.0, 17.0)
            changed = True

        # D-Pad Horizontal (Pan): -1.0 = Derecha, +1.0 = Izquierda
        if dpad_x > 0.5 and self._prev_dpad_x <= 0.5:
            self.current_pan = self._clamp(self.current_pan - self.pan_step, -35.0, 35.0)
            changed = True
        elif dpad_x < -0.5 and self._prev_dpad_x >= -0.5:
            self.current_pan = self._clamp(self.current_pan + self.pan_step, -35.0, 35.0)
            changed = True

        self._prev_dpad_x = dpad_x
        self._prev_dpad_y = dpad_y

        # Botón Reset al horizonte
        if btn_reset < len(msg.buttons):
            val_btn = msg.buttons[btn_reset]
            if val_btn == 1 and self._last_btn_reset == 0:
                self.current_tilt = 0.0
                self.current_pan = 0.0
                changed = True
            self._last_btn_reset = val_btn

        if changed:
            self._publish_camera()
            self.get_logger().info(
                f"🎮 [Joy D-Pad] Cámara -> Tilt: {self.current_tilt:+.1f}° | Pan: {self.current_pan:+.1f}°"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ControlCamaraBebop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
