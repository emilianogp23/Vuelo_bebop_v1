#!/usr/bin/env python3
"""
Localizador a bordo: la cámara del Bebop ve los ArUco de las vallas.

Entradas:
    /bebop/camera/image_raw, /bebop/camera/camera_info
Salidas:
    /vision/onboard/pose (PoseWithCovarianceStamped, marco corral, stamp de la imagen)
    /bebop/move_camera   (Vector3: tilt, pan) a 1 Hz si camera_tilt_deg no es NaN
Con obj_suelo=true (objeto en el piso, p. ej. el Pioneer 3DX) la cámara baja a
camera_tilt_suelo_deg mientras dura la órbita (AUTOMATIC) y vuelve a
camera_tilt_deg en los demás estados para ver los ArUco de las vallas.

El mapa de marcadores está en config/aruco_map.yaml (ver vision.py).
No lanzar junto con control_camara: los dos mueven la cámara.
"""
import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32

from control_bebop.frames import quaternion_from_yaw
from control_bebop.paths import config_file
from control_bebop.vision import (camera_in_world, detect, load_marker_map,
                                  make_detector, marker_side_px,
                                  MeasurementJumpFilter, measurement_sigma, onboard_drone_pose,
                                  solve_pnp)


def pose_msg(stamp, x, y, z, yaw, sigma_xy, sigma_yaw):
    """PoseWithCovarianceStamped en marco corral."""
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = 'corral'
    p = msg.pose.pose
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quaternion_from_yaw(yaw)
    cov = [0.0] * 36
    cov[0] = cov[7] = sigma_xy ** 2
    cov[14] = (2.0 * sigma_xy) ** 2
    cov[21] = cov[28] = 1e3   # roll/pitch: no se miden
    cov[35] = sigma_yaw ** 2
    msg.pose.covariance = cov
    return msg


class ArucoLocalizer(Node):
    def __init__(self):
        super().__init__('aruco_localizer')
        self.declare_parameter('map_file', '')            # vacío = aruco_map.yaml
        self.declare_parameter('image_topic', '/bebop/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/bebop/camera/camera_info')
        self.declare_parameter('rate', 15.0)              # Hz de procesamiento
        self.declare_parameter('camera_tilt_deg', -28.0)  # NaN = no mover la cámara
        self.declare_parameter('obj_suelo', False)
        self.declare_parameter('camera_tilt_suelo_deg', -70.0)
        self.declare_parameter('camera_pan_deg', 0.0)
        self.declare_parameter('cam_offset_x', 0.09)      # cámara por delante del centro (m)
        self.declare_parameter('min_marker_px', 18.0)
        self.declare_parameter('max_reproj_px', 2.0)
        self.declare_parameter('min_markers', 1)          # 2 en el real: con uno solo falla 0.3–0.5 m
        self.declare_parameter('max_distance', 4.5)
        self.declare_parameter('sigma_xy', 0.03)          # a 1.5 m con un marcador
        self.declare_parameter('sigma_yaw_deg', 2.0)
        # Poses volteadas de PnP (0.5–1 m, 5–9 % de las mediciones con 2 marcadores
        # lejanos, 27 oct): se rechaza un salto > max_meas_jump salvo que se confirme
        self.declare_parameter('max_meas_jump', 0.5)      # 0 = sin filtro
        self.declare_parameter('sigma_few_factor', 1.5)   # σ ×k con menos de 3 marcadores
        # Simulación: la imagen trae hora de Gazebo y la odometría del reloj del nodo
        # shortcut: ignora la latencia del render (~1 cuadro), usar /clock si importa
        self.declare_parameter('stamp_with_receive_time', False)

        gp = self.get_parameter
        path = config_file(gp('map_file').value or 'aruco_map.yaml')   # nombre en config/ o ruta
        dict_name, self.map = load_marker_map(path)
        self.detector = make_detector(dict_name)
        self.period = 1.0 / gp('rate').value
        self.tilt = gp('camera_tilt_deg').value
        self.obj_suelo = gp('obj_suelo').value
        self.tilt_suelo = gp('camera_tilt_suelo_deg').value
        self.flight_state = 0
        self.pan = math.radians(gp('camera_pan_deg').value)
        self.cam_offset_x = gp('cam_offset_x').value
        self.min_px = gp('min_marker_px').value
        self.max_err = gp('max_reproj_px').value
        self.min_markers = gp('min_markers').value
        self.max_dist = gp('max_distance').value
        self.sigma_xy = gp('sigma_xy').value
        self.sigma_yaw = math.radians(gp('sigma_yaw_deg').value)
        self.receive_stamp = gp('stamp_with_receive_time').value
        self.sigma_few = gp('sigma_few_factor').value
        self.jump = (MeasurementJumpFilter(max_jump=gp('max_meas_jump').value)
                     if gp('max_meas_jump').value > 0 else None)

        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.last_proc = 0.0
        self.stats = {'frames': 0, 'poses': 0, 'saltos': 0}

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, gp('image_topic').value, self.image_cb, qos)
        self.create_subscription(CameraInfo, gp('camera_info_topic').value, self.info_cb, qos)
        self.create_subscription(Int32, '/flight_state', self._state_cb, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/vision/onboard/pose', 10)
        self.pub_cam = self.create_publisher(Vector3, '/bebop/move_camera', 1)
        if not math.isnan(self.tilt):
            self.create_timer(1.0, self.camera_cb)
        self.create_timer(5.0, self.status_cb)
        self.get_logger().info(
            f'aruco_localizer: {len(self.map)} marcadores de {path}, tilt {self.tilt}°'
            + (f', objeto en el piso: {self.tilt_suelo}° en la órbita' if self.obj_suelo else ''))

    def _state_cb(self, msg):
        changed = self.obj_suelo and (msg.data == 1) != (self.flight_state == 1)
        self.flight_state = msg.data
        if changed and not math.isnan(self.tilt):
            self.camera_cb()   # cambio inmediato, sin esperar al timer de 1 Hz
            self.get_logger().info(
                f'Cámara a {self.current_tilt()}° ({"órbita" if msg.data == 1 else "marcadores"})')

    def current_tilt(self):
        # Solo en AUTOMATIC (1) se mira al objeto en el piso
        return self.tilt_suelo if self.obj_suelo and self.flight_state == 1 else self.tilt

    def camera_cb(self):
        msg = Vector3()
        msg.x = float(max(-85.0, min(17.0, self.current_tilt())))
        msg.y = float(math.degrees(self.pan))
        self.pub_cam.publish(msg)

    def info_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64)
            self.get_logger().info(f'Intrínsecos recibidos: fx={self.K[0, 0]:.1f}')

    def image_cb(self, msg):
        if self.K is None:
            return
        stamp = self.get_clock().now().to_msg() if self.receive_stamp else msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        if t - self.last_proc < self.period:
            return
        self.last_proc = t
        self.stats['frames'] += 1

        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        found = {i: c for i, c in detect(self.detector, gray).items()
                 if i in self.map and marker_side_px(c) >= self.min_px}
        if len(found) < max(1, self.min_markers):
            return
        obj = np.vstack([self.map[i] for i in found])
        img = np.vstack(list(found.values()))
        sol = solve_pnp(obj, img, self.K, self.D, level_camera=True)
        if sol is None or sol[2] > self.max_err:
            return
        R_wc, cam = camera_in_world(sol[0], sol[1])
        dist = float(np.mean(np.linalg.norm(obj - cam, axis=1)))
        if dist > self.max_dist:
            return
        x, y, z, yaw = onboard_drone_pose(R_wc, cam, self.cam_offset_x)
        if self.jump is not None and not self.jump.accept(t, x, y):
            self.stats['saltos'] += 1
            return
        sxy, syaw = measurement_sigma(dist, len(found), self.sigma_xy, self.sigma_yaw)
        if len(found) < 3:
            sxy, syaw = sxy * self.sigma_few, syaw * self.sigma_few
        self.pub.publish(pose_msg(stamp, x, y, z, yaw - self.pan, sxy, syaw))
        self.stats['poses'] += 1

    def status_cb(self):
        if self.K is None:
            self.get_logger().warn('Sin camera_info: ¿driver corriendo?')
        else:
            self.get_logger().info(
                f'Cuadros procesados {self.stats["frames"]}, poses {self.stats["poses"]}, '
                f'saltos rechazados {self.stats["saltos"]} (últimos 5 s)')
        self.stats = {'frames': 0, 'poses': 0, 'saltos': 0}


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizer()
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
