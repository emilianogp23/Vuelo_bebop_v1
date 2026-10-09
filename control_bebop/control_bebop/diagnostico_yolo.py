#!/usr/bin/env python3
"""
Script de Diagnóstico Rápido para Cámara y Detección YOLOv8 del Bebop 2.

Ejecución:
  ros2 run control_bebop diagnostico_yolo

Qué hace este diagnóstico:
1. Comprueba si el tópico /bebop/camera/image_raw está publicando y mide FPS.
2. Centra automáticamente la cámara del dron hacia el frente (0°, 0°) en /bebop/move_camera.
3. Ejecuta inferencia YOLOv8 en vivo e imprime cada detección en la terminal.
4. Guarda un frame anotado en /tmp/diagnostico_yolo.jpg para verificación visual inmediata.
5. Abre una ventana de OpenCV (si hay entorno gráfico disponible).
"""

import sys
import os

# Asegurar acceso a rob_env donde está instalado ultralytics y torch
for site_pkg in [
    '/home/emiliano/rob_env/lib/python3.12/site-packages',
    os.path.expanduser('~/rob_env/lib/python3.12/site-packages'),
]:
    if os.path.exists(site_pkg) and site_pkg not in sys.path:
        sys.path.insert(0, site_pkg)

import time
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge

# Importar detector YOLO optimizado
from control_bebop.yolo_person_detector import YoloPersonDetector


class DiagnosticoYoloNode(Node):
    def __init__(self):
        super().__init__('diagnostico_yolo')
        self.bridge = CvBridge()

        # Iniciar detector en modo DEMO (personas + objetos de prueba)
        self.detector = YoloPersonDetector(conf_threshold=0.20, mode='demo')

        # Publicador de orientación de cámara: Forzar al frente
        self.pub_cam = self.create_publisher(Vector3, '/bebop/move_camera', 10)
        self._align_camera()

        # Suscriptor a imagen con QoS SensorData y BestEffort
        self.sub_cam = self.create_subscription(
            Image, '/bebop/camera/image_raw', self._image_cb, qos_profile_sensor_data
        )

        self.frames_received = 0
        self.start_time = time.time()
        self.last_print_time = time.time()

        print("\n" + "="*70)
        print("🔍 [DIAGNÓSTICO EN VIVO] Cámara y YOLOv8 Bebop 2")
        print("="*70)
        print("📡 Escuchando en: /bebop/camera/image_raw")
        print("🎥 Forzando cámara al horizonte (Tilt: 0.0°, Pan: 0.0°)...")
        print("🤖 Umbral YOLO: 20% | Modo: DEMO (Personas + Objetos cotidianos)")
        print("💾 Las capturas se guardarán en: /tmp/diagnostico_yolo.jpg")
        print("="*70 + "\n")

        # Timer para avisar si no llegan frames
        self.create_timer(3.0, self._check_frames_timeout)
        self.create_timer(1.0, self._align_camera)

    def _align_camera(self):
        msg = Vector3()
        msg.x = 0.0  # Frente horizontal
        msg.y = 0.0  # Centro
        msg.z = 0.0
        self.pub_cam.publish(msg)

    def _check_frames_timeout(self):
        if self.frames_received == 0:
            print("⚠️  [ALERTA]: No se han recibido frames todavía de '/bebop/camera/image_raw'.")
            print("    Posibles causas:")
            print("    1. 'ros2 launch ros2_bebop_driver bebop_node_launch.xml' no está activo.")
            print("    2. La conexión Wi-Fi con el Bebop 2 se interrumpió.")
            print("    3. Verifica con: 'ros2 topic hz /bebop/camera/image_raw'")

    def _image_cb(self, msg: Image):
        self.frames_received += 1
        now = time.time()

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            print(f"❌ Error decodificando imagen: {e}")
            return

        annotated_img, detections = self.detector.detect(cv_img)

        # Imprimir telemetría periódica
        if now - self.last_print_time >= 1.0:
            fps = self.frames_received / max(0.1, (now - self.start_time))
            h, w = cv_img.shape[:2]
            det_summary = ", ".join([f"{d['class']}({d['conf']*100:.0f}%, {d['dist_m']:.2f}m)" for d in detections])
            if detections:
                print(f"🟢 [FRAME #{self.frames_received:04d}] ({w}x{h} @ {fps:.1f} FPS) -> {len(detections)} OBJETO(S): {det_summary}")
            else:
                print(f"⚪ [FRAME #{self.frames_received:04d}] ({w}x{h} @ {fps:.1f} FPS) -> Sin detecciones en este frame.")
            self.last_print_time = now

        # Si hay detecciones, guardar muestra en /tmp
        if detections or self.frames_received % 30 == 0:
            cv2.imwrite('/tmp/diagnostico_yolo.jpg', annotated_img)

        # Mostrar ventana si hay display
        if 'DISPLAY' in os.environ and os.environ['DISPLAY']:
            try:
                cv2.imshow("Diagnostico Bebop YOLOv8", annotated_img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("Cerrando diagnóstico por el usuario...")
                    rclpy.shutdown()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticoYoloNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
