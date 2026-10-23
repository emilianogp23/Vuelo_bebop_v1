#!/usr/bin/env python3
"""
Calibración de la cámara del Bebop 2 con un tablero ChArUco.

Con el driver corriendo (dron encendido, motores apagados), se mueve el
tablero frente a la cámara. El nodo toma una vista cuando el tablero está
quieto, se ven al menos min_corners esquinas y la vista es distinta de las
anteriores (posición, tamaño o inclinación). Con `samples` vistas calibra y
escribe config/bebop2_camera_calib.yaml en el formato que carga el driver.

Entradas:  /bebop/camera/image_raw (image_topic), /bebop/camera/camera_info
Salidas:   /calibracion/imagen (vista anotada, para rqt_image_view)
           /bebop/move_camera a 1 Hz si camera_tilt_deg no es NaN
Ventana (show_window): c = calibrar ya (≥ 10 vistas), r = reiniciar, q = salir.

Imprimir el tablero con scripts/generar_charuco.py y pasar el lado MEDIDO del
cuadro en square_length. No lanzar junto con aruco_localizer (los dos mueven
la cámara). Guía: GUIA_PRUEBAS_Y_LOCALIZACION.md §4.
"""
import math
import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from control_bebop.calibracion import (Charuco, calibrate, camera_info_yaml, coverage,
                                       is_new_view, view_features)
from control_bebop.paths import source_config_path


class CalibrarCamara(Node):
    def __init__(self):
        super().__init__('calibrar_camara')
        for name, default in (
                ('image_topic', '/bebop/camera/image_raw'),
                ('camera_info_topic', '/bebop/camera/camera_info'),
                ('squares_x', 7), ('squares_y', 5),
                ('square_length', 0.035),        # m, MEDIDO en la impresión
                ('marker_length', 0.026),        # m
                ('dictionary', 'DICT_5X5_100'),
                ('min_corners', 12),
                ('samples', 30),
                ('min_view_diff', 0.15),
                ('still_px', 2.0),               # px de movimiento máx. para tomar la vista
                ('camera_tilt_deg', -28.0),      # el de vuelo; NaN = no mover la cámara
                ('output', ''),                  # vacío = config/bebop2_camera_calib.yaml
                ('camera_name', 'bebop_front'),
                ('show_window', True)):
            self.declare_parameter(name, default)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        if gp('marker_length') >= gp('square_length'):
            raise ValueError('marker_length debe ser menor que square_length')
        self.charuco = Charuco(gp('squares_x'), gp('squares_y'), gp('square_length'),
                               gp('marker_length'), gp('dictionary'))
        self.min_corners = gp('min_corners')
        self.samples = gp('samples')
        self.min_diff = gp('min_view_diff')
        self.still_px = gp('still_px')
        self.tilt = gp('camera_tilt_deg')
        self.output = gp('output') or source_config_path('bebop2_camera_calib.yaml')
        self.camera_name = gp('camera_name')
        self.show = gp('show_window') and bool(os.environ.get('DISPLAY'))

        self.bridge = CvBridge()
        self.views, self.features = [], []
        self.prev = None          # (ids, esquinas) del cuadro anterior
        self.done = False
        self.current_fx = None

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, gp('image_topic'), self.image_cb, qos)
        self.create_subscription(CameraInfo, gp('camera_info_topic'), self.info_cb, qos)
        self.pub_img = self.create_publisher(Image, '/calibracion/imagen', 1)
        self.pub_cam = self.create_publisher(Vector3, '/bebop/move_camera', 1)
        if not math.isnan(self.tilt):
            self.create_timer(1.0, lambda: self.pub_cam.publish(
                Vector3(x=float(max(-85.0, min(17.0, self.tilt))))))
        self.get_logger().info(
            f'Calibración ChArUco {gp("squares_x")}x{gp("squares_y")}, cuadro '
            f'{gp("square_length") * 100:.1f} cm, {gp("dictionary")}. '
            f'Faltan {self.samples} vistas: '
            'mueve el tablero por todo el cuadro, cerca y lejos, e inclínalo ±30°. '
            f'Resultado → {self.output}')

    def info_cb(self, msg):
        self.current_fx = msg.k[0]

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        size = (frame.shape[1], frame.shape[0])
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self.charuco.detect(gray)
        status = f'{len(self.views)}/{self.samples} vistas'
        if ids is not None and len(ids) >= self.min_corners and not self.done:
            status += self._try_add(corners, ids, size)
        elif ids is not None:
            status += f' | {len(ids)} esquinas (mín. {self.min_corners})'
        if ids is not None:
            for p in corners.astype(int):
                cv2.circle(frame, tuple(p), 3, (0, 255, 0), -1)
        for f in self.features:   # centros de las vistas tomadas
            cv2.circle(frame, (int(f[0] * size[0]), int(f[1] * size[1])), 6, (255, 128, 0), 2)
        cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header = msg.header
        self.pub_img.publish(out)

        if len(self.views) >= self.samples and not self.done:
            self.run_calibration(size)
        if self.show:
            cv2.imshow('Calibracion Bebop (c calibrar, r reiniciar, q salir)', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('c') and len(self.views) >= 10:
                self.run_calibration(size)
            elif key == ord('r'):
                self.views, self.features, self.done = [], [], False
                self.get_logger().info('Vistas borradas')
            elif key == ord('q'):
                raise KeyboardInterrupt

    def _try_add(self, corners, ids, size):
        prev, self.prev = self.prev, (ids, corners)
        if prev is None or not np.array_equal(prev[0], ids):
            return ' | quieto...'
        if np.abs(prev[1] - corners).max() > self.still_px:
            return ' | quieto...'
        feat = view_features(self.charuco, corners, ids, size)
        if not is_new_view(feat, self.features, self.min_diff):
            return ' | mueve o inclina el tablero'
        self.views.append((corners, ids))
        self.features.append(feat)
        cov = coverage(self.features)
        self.get_logger().info(
            f'Vista {len(self.views)}/{self.samples}: {len(ids)} esquinas | cubierto x '
            f'{cov["x"][0]:.2f}–{cov["x"][1]:.2f}, y {cov["y"][0]:.2f}–{cov["y"][1]:.2f}, '
            f'tamaño {cov["size"][0]:.2f}–{cov["size"][1]:.2f}, inclinación máx. '
            f'{cov["tilt_max_deg"]:.0f}°')
        return ' | VISTA TOMADA'

    def run_calibration(self, size):
        self.done = True
        cov = coverage(self.features)
        if cov['tilt_max_deg'] < 20.0:
            self.get_logger().warn('Casi sin inclinación del tablero: la focal puede salir mal')
        rms, K, D, errors = calibrate(self.charuco, self.views, size)
        worst = int(np.argmax(errors))
        self.get_logger().info(
            f'Calibración: error RMS {rms:.3f} px (bueno < 0.5, aceptable < 1.0) | peor vista '
            f'{worst + 1}: {errors[worst]:.2f} px | fx {K[0, 0]:.1f} fy {K[1, 1]:.1f} '
            f'cx {K[0, 2]:.1f} cy {K[1, 2]:.1f} | D {np.round(D, 4).tolist()}')
        if self.current_fx:
            self.get_logger().info(f'fx que publica hoy el driver: {self.current_fx:.1f}')
        if rms > 1.0:
            self.get_logger().error('RMS > 1 px: NO se guarda. Revisa square_length y repite (r)')
            return
        comment = (f'Calibrado con calibrar_camara (ChArUco) el {datetime.now():%Y-%m-%d %H:%M}\n'
                   f'{len(self.views)} vistas, error RMS {rms:.3f} px, tilt {self.tilt}°')
        os.makedirs(os.path.dirname(self.output), exist_ok=True)
        with open(self.output, 'w', encoding='utf-8') as f:
            f.write(camera_info_yaml(K, D, size, self.camera_name, comment))
        self.get_logger().info(
            f'Guardado en {self.output}. Para usarlo, relanza el driver con '
            f'camera_calibration_file:=file://{os.path.abspath(self.output)}')


def main(args=None):
    rclpy.init(args=args)
    node = CalibrarCamara()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
