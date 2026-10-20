"""
Nodo de grabación de video para Bebop 2.

Graba las imágenes de la cámara del dron cuando está volando.
Se activa/desactiva según el estado de vuelo (/flight_state).

Fixes aplicados vs versión anterior:
    - QoS BEST_EFFORT para compatibilidad con bridge de Gazebo
    - FPS configurable; los cuadros se repiten o descartan según su marca
      de tiempo (FramePacer) para que el video dure lo mismo que el vuelo
      aunque la cámara publique a otra frecuencia
    - Nombre de archivo con timestamp para no sobrescribir
    - f-string corregido en logs de error
    - Hook destroy_node() correcto de ROS2
    - CvBridge instanciado una sola vez
    - Grabación controlada por estado de vuelo
"""

import os
from datetime import datetime

from control_bebop.frame_pacer import FramePacer
from control_bebop.paths import data_dir

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2


# Estados en los que se graba video (volando)
RECORDING_STATES = {1, 2, 5, 6, 7, 8}  # AUTO, TAKEOFF, HOVER, MANUAL,
                                         # RECOVERY, RETURN_HOME


class NodoVideo(Node):
    def __init__(self):
        super().__init__('nodo_video')
        self.bridge = CvBridge()

        # ── Parámetros ────────────────────────────────────────
        # Frecuencia del AVI. La de la cámara puede ser otra (Bebop ~30 Hz,
        # Gazebo 15 Hz en sim.yaml): FramePacer ajusta por marca de tiempo.
        self.declare_parameter('camera_fps', 30.0)
        self.declare_parameter(
            'image_topic', '/bebop/camera/image_raw')
        # Vacío = <paquete>/videos (relativo al proyecto, ver paths.py)
        self.declare_parameter('output_dir', '')

        self.fps = self.get_parameter('camera_fps').value
        self.output_dir = (self.get_parameter('output_dir').value
                           or data_dir('videos'))
        topic_name = self.get_parameter('image_topic').value

        # ── QoS compatible con driver real y simulador ────────
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ── Suscripciones ─────────────────────────────────────
        self.subscription = self.create_subscription(
            Image, topic_name, self.image_callback, camera_qos)

        self.sub_state = self.create_subscription(
            Int32, '/flight_state', self.state_callback, 10)

        # ── Estado interno ────────────────────────────────────
        self.out = None
        self.is_recording = False
        self.current_state = 0
        self.frame_count = 0
        self.pacer = FramePacer(self.fps)

        self.get_logger().info(
            f'Nodo de video escuchando: {topic_name} (video a {self.fps:g} fps)')

    def state_callback(self, msg):
        """Controla grabación según estado de vuelo."""
        self.current_state = msg.data
        should_record = msg.data in RECORDING_STATES

        if should_record and not self.is_recording:
            self.is_recording = True
            self.get_logger().info('Grabación iniciada')
        elif not should_record and self.is_recording:
            self._finalize_video()

    def _stamp(self, msg):
        """Marca de tiempo del cuadro (s); la hora del nodo si viene vacía."""
        st = msg.header.stamp
        if st.sec or st.nanosec:
            return st.sec + st.nanosec * 1e-9
        return self.get_clock().now().nanoseconds * 1e-9

    def image_callback(self, msg):
        if not self.is_recording:
            return
        try:
            copies = self.pacer.copies(self._stamp(msg))
            if copies == 0:
                return

            cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')

            if self.out is None:
                height, width, _ = cv_image.shape
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                os.makedirs(self.output_dir, exist_ok=True)

                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                nombre = f'vuelo_{timestamp}.avi'
                ruta = os.path.join(self.output_dir, nombre)
                self.out = cv2.VideoWriter(
                    ruta, fourcc, self.fps, (width, height))
                self.get_logger().info(f'Grabando a: {ruta}')

            for _ in range(copies):
                self.out.write(cv_image)
            self.frame_count += copies
        except Exception as e:
            self.get_logger().error(f'Error en video: {e}')

    def _finalize_video(self):
        """Cierra el VideoWriter actual y prepara para nueva grabación."""
        if self.out is not None:
            self.out.release()
            self.out = None
            self.get_logger().info(
                f'✅ Video guardado ({self.frame_count} frames)')
            self.frame_count = 0
        self.pacer.reset()
        self.is_recording = False

    def destroy_node(self):
        self._finalize_video()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo_video = NodoVideo()
    try:
        rclpy.spin(nodo_video)
    except KeyboardInterrupt:
        pass
    finally:
        nodo_video.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()