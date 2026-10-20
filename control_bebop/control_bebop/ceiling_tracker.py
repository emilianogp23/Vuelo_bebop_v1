#!/usr/bin/env python3
"""
Localizador desde el techo: una cámara fija (Logitech C920) ve el ArUco
pegado arriba del dron.

Entrada:  la cámara USB, leída directo con OpenCV (V4L2).
Salida:   /vision/ceiling/pose (PoseWithCovarianceStamped, marco corral)

Calibración (intrínsecos y posición de la cámara) en config/ceiling_camera.yaml,
generada con:  ros2 run control_bebop calibrar_camara_techo {intrinsecos|extrinsecos}
Sin extrínsecos calibrados el nodo no publica.
"""
import math
import time

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

from control_bebop.aruco_localizer import pose_msg
from control_bebop.paths import config_file
from control_bebop.vision import (camera_in_world, ceiling_drone_pose, detect,
                                  load_marker_map, make_detector,
                                  marker_side_px, measurement_sigma)


def open_camera(cfg):
    """Abre la C920 con foco y exposición fijos (si el foco cambia, la calibración no vale)."""
    cap = cv2.VideoCapture(int(cfg.get('device', 0)), cv2.CAP_V4L2)
    # MJPG: en YUYV la C920 solo da ~10 fps a 720p
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cfg.get('width', 1280)))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cfg.get('height', 720)))
    cap.set(cv2.CAP_PROP_FPS, float(cfg.get('fps', 30)))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FOCUS, float(cfg.get('focus', 0)))
    if float(cfg.get('exposure', -1)) >= 0:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # 1 = manual en V4L2
        cap.set(cv2.CAP_PROP_EXPOSURE, float(cfg['exposure']))
    return cap


def load_camera_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


class CeilingTracker(Node):
    def __init__(self):
        super().__init__('ceiling_tracker')
        self.declare_parameter('camera_file', '')   # vacío = config/ceiling_camera.yaml
        self.declare_parameter('map_file', '')      # vacío = config/aruco_map.yaml
        self.declare_parameter('drone_yaw_offset_deg', 0.0)
        self.declare_parameter('min_marker_px', 20.0)
        self.declare_parameter('max_reproj_px', 2.0)
        self.declare_parameter('sigma_xy', 0.015)    # a 1.5 m
        self.declare_parameter('sigma_yaw_deg', 1.5)

        gp = self.get_parameter
        cam_path = gp('camera_file').value or config_file('ceiling_camera.yaml')
        self.cfg = load_camera_config(cam_path)
        dict_name, _, drone = load_marker_map(gp('map_file').value or config_file('aruco_map.yaml'))
        self.drone_id = int(drone['id'])
        self.drone_size = float(drone['size'])
        self.detector = make_detector(dict_name)
        self.K = np.array(self.cfg['camera_matrix'], dtype=np.float64).reshape(3, 3)
        self.D = np.array(self.cfg['dist_coeffs'], dtype=np.float64)
        ext = self.cfg.get('extrinsics') or {}
        self.ready = bool(ext.get('calibrated'))
        if self.ready:
            self.R_wc, self.cam_pos = camera_in_world(ext['rvec'], ext['tvec'])
        else:
            self.get_logger().error(
                'Cámara del techo sin extrínsecos: corre '
                '"ros2 run control_bebop calibrar_camara_techo extrinsecos". No se publica nada.')
        if not self.cfg.get('intrinsics_calibrated'):
            self.get_logger().warn('Intrínsecos nominales de la C920: calibrarlos (fase 1).')

        self.yaw_offset = math.radians(gp('drone_yaw_offset_deg').value)
        self.min_px = gp('min_marker_px').value
        self.max_err = gp('max_reproj_px').value
        self.sigma_xy = gp('sigma_xy').value
        self.sigma_yaw = math.radians(gp('sigma_yaw_deg').value)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/vision/ceiling/pose', 10)
        self.cap = open_camera(self.cfg)
        if not self.cap.isOpened():
            self.get_logger().error(f'No se pudo abrir la cámara /dev/video{self.cfg.get("device", 0)}')
        self.stats = {'frames': 0, 'poses': 0}
        self.create_timer(5.0, self.status_cb)
        self.get_logger().info(f'ceiling_tracker: marcador del dron id {self.drone_id}, {cam_path}')

    def step(self):
        """Lee un cuadro y publica la pose si ve el marcador del dron."""
        ok, frame = self.cap.read()
        stamp = self.get_clock().now().to_msg()
        if not ok:
            time.sleep(0.1)   # cámara desconectada: no girar en vacío
            return
        self.stats['frames'] += 1
        if not self.ready:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        c = detect(self.detector, gray).get(self.drone_id)
        if c is None or marker_side_px(c) < self.min_px:
            return
        res = ceiling_drone_pose(c, self.drone_size, self.K, self.D,
                                 self.R_wc, self.cam_pos, self.yaw_offset)
        if res is None or res[1] > self.max_err:
            return
        (x, y, z, yaw), _, dist = res
        sxy, syaw = measurement_sigma(dist, 1, self.sigma_xy, self.sigma_yaw)
        self.pub.publish(pose_msg(stamp, x, y, z, yaw, sxy, syaw))
        self.stats['poses'] += 1

    def status_cb(self):
        self.get_logger().info(
            f'Cuadros {self.stats["frames"]}, poses del dron {self.stats["poses"]} (últimos 5 s)')
        self.stats = {'frames': 0, 'poses': 0}


def main(args=None):
    rclpy.init(args=args)
    node = CeilingTracker()
    try:
        while rclpy.ok():
            node.step()   # cap.read() bloquea hasta el siguiente cuadro
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
