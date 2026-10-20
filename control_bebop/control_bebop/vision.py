"""
Localización visual con ArUco (lógica pura, sin ROS) para el Bebop 2.

Marco "corral": origen en el centro de la zona segura de despegue, en el piso;
x hacia donde apunta el dron al despegar, y a la izquierda, z arriba. Si el
dron despega en el origen mirando a +x, coincide con el marco de despegue.

Dos fuentes de medición:
    - A bordo: la cámara del Bebop ve marcadores en las vallas (mapa conocido)
      → pose de la cámara → pose del dron.
    - Techo: una cámara fija (C920) ve un marcador pegado sobre el dron.

Ambas se fusionan con la odometría en ``OffsetFilter``: un Kalman de 3
estados que estima el error acumulado (dx, dy, dyaw) de la odometría.
"""

import math
from collections import deque

import cv2
import numpy as np
import yaml

from control_bebop.frames import normalize_angle, rotate2d


# ── Mapa de marcadores ────────────────────────────────────────
def marker_corners(center, size, normal_yaw=None, floor_yaw=None):
    """Esquinas 3D (orden de OpenCV: TL, TR, BR, BL) de un marcador.

    ``normal_yaw``: marcador vertical en una valla; ángulo (rad) hacia donde
    mira su cara. ``floor_yaw``: marcador en el piso mirando arriba; ángulo
    (rad) hacia donde apunta su borde superior.
    """
    c = np.asarray(center, dtype=np.float64)
    h = size / 2.0
    if normal_yaw is not None:
        # Quien lo mira ve hacia -n; su derecha es (-n_y, n_x, 0)
        right = np.array([-math.sin(normal_yaw), math.cos(normal_yaw), 0.0])
        up = np.array([0.0, 0.0, 1.0])
    elif floor_yaw is not None:
        up = np.array([math.cos(floor_yaw), math.sin(floor_yaw), 0.0])
        right = np.array([math.sin(floor_yaw), -math.cos(floor_yaw), 0.0])
    else:
        raise ValueError('marcador sin normal_yaw_deg ni floor_yaw_deg')
    return np.array([c + h * (-right + up), c + h * (right + up),
                     c + h * (right - up), c + h * (-right - up)])


def load_marker_map(path):
    """Lee ``aruco_map.yaml``.

    Returns:
        (dictionary_name, {id: esquinas (4, 3)}, drone_marker dict o None)
    """
    with open(path) as f:
        data = yaml.safe_load(f)
    corners = {}
    for m in data.get('markers', []):
        nyaw = m.get('normal_yaw_deg')
        fyaw = m.get('floor_yaw_deg')
        corners[int(m['id'])] = marker_corners(
            m['center'], float(m['size']),
            None if nyaw is None else math.radians(nyaw),
            None if fyaw is None else math.radians(fyaw))
    return data.get('dictionary', 'DICT_4X4_50'), corners, data.get('drone_marker')


def make_detector(dictionary_name):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    if hasattr(cv2.aruco, 'ArucoDetector'):
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        return cv2.aruco.ArucoDetector(dictionary, params)
    # OpenCV < 4.7 (python3-opencv de Ubuntu, el que usan los nodos con
    # /usr/bin/python3): API vieja; DetectorParameters() ahí da segfault
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return dictionary, params


def detect(detector, image):
    """{id: esquinas en la imagen (4, 2)} de los marcadores detectados."""
    if isinstance(detector, tuple):
        corners, ids, _ = cv2.aruco.detectMarkers(image, detector[0], parameters=detector[1])
    else:
        corners, ids, _ = detector.detectMarkers(image)
    if ids is None:
        return {}
    return {int(i): c.reshape(4, 2).astype(np.float64) for i, c in zip(ids.flatten(), corners)}


def marker_side_px(img_corners):
    """Lado más corto (px) del marcador en la imagen."""
    return min(np.linalg.norm(img_corners[i] - img_corners[(i + 1) % 4]) for i in range(4))


# ── PnP ───────────────────────────────────────────────────────
def solve_pnp(obj_pts, img_pts, K, D, ambiguity_ratio=2.0, level_camera=False):
    """Pose mundo → cámara a partir de puntos 3D-2D.

    Con un solo marcador (4 puntos coplanares) usa IPPE, que da dos
    soluciones; si sus errores son parecidos la pose es ambigua. Con
    ``level_camera`` (cámara sin roll, como el video estabilizado del Bebop)
    se elige la solución con la cámara nivelada; si no, se descarta.

    Returns:
        (rvec, tvec, error_reproyeccion_px) o None
    """
    obj = np.asarray(obj_pts, dtype=np.float64).reshape(-1, 3)
    img = np.asarray(img_pts, dtype=np.float64).reshape(-1, 2)
    K = np.asarray(K, dtype=np.float64)
    D = np.asarray(D, dtype=np.float64)
    if len(obj) < 4:
        return None
    if len(obj) == 4:
        n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
            obj, img, K, D, flags=cv2.SOLVEPNP_IPPE)
        if n == 0:
            return None
        errs = np.asarray(errs).flatten()
        order = np.argsort(errs)
        best = order[0]
        if n > 1 and errs[order[1]] < ambiguity_ratio * max(errs[order[0]], 1e-3):
            if not level_camera:
                return None
            # |componente z del eje x de la cámara| = cuánto roll tiene
            roll = [abs(cv2.Rodrigues(rvecs[i])[0][0, 2]) for i in order[:2]]
            if min(roll) > 0.1 or max(roll) < 0.2:
                return None
            best = order[int(np.argmin(roll))]
        rvec, tvec = rvecs[best], tvecs[best]
    else:
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_SQPNP)
        if not ok:
            return None
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, D, rvec, tvec)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
    return rvec.reshape(3), tvec.reshape(3), err


def camera_in_world(rvec, tvec):
    """(R_wc, posición de la cámara en el mundo) a partir de mundo → cámara."""
    R_cw, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    R_wc = R_cw.T
    return R_wc, -R_wc @ np.asarray(tvec, dtype=np.float64).reshape(3)


def onboard_drone_pose(R_wc, cam_pos, cam_offset_x=0.0):
    """Pose (x, y, z, yaw) del dron a partir de la pose de su cámara.

    El video del Bebop está estabilizado: la cámara no tiene roll, así que su
    eje x (derecha de la imagen) es horizontal y el frente del dron es ese eje
    girado +90°. No depende del tilt. ``cam_offset_x``: la cámara está esa
    distancia por delante del centro del dron.
    """
    rx, ry = R_wc[0, 0], R_wc[1, 0]
    yaw = math.atan2(rx, -ry)
    ox, oy = rotate2d(cam_offset_x, 0.0, yaw)
    return cam_pos[0] - ox, cam_pos[1] - oy, cam_pos[2], yaw


def drone_marker_corners_local(size):
    """Esquinas del marcador del dron en su propio marco (x derecha, y arriba)."""
    h = size / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


def ceiling_drone_pose(img_corners, size, K, D, R_wc, cam_pos, yaw_offset=0.0):
    """Pose del dron vista desde la cámara del techo.

    El marcador va pegado arriba del dron con su borde superior hacia el
    frente. Returns ((x, y, z, yaw), error_px, distancia_m) o None.
    """
    n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
        drone_marker_corners_local(size), img_corners.reshape(4, 1, 2),
        np.asarray(K, dtype=np.float64), np.asarray(D, dtype=np.float64),
        flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if n == 0:
        return None
    i = int(np.argmin(np.asarray(errs).flatten()))
    R_cm, _ = cv2.Rodrigues(rvecs[i])
    t_cm = tvecs[i].reshape(3)
    R_wm = R_wc @ R_cm
    p = R_wc @ t_cm + cam_pos
    # El ángulo del eje y del marcador en el plano casi no depende de la ambigüedad
    yaw = normalize_angle(math.atan2(R_wm[1, 1], R_wm[0, 1]) + yaw_offset)
    return (float(p[0]), float(p[1]), float(p[2]), yaw), float(np.asarray(errs).flatten()[i]), \
        float(np.linalg.norm(t_cm))


def measurement_sigma(distance, n_markers, sigma_xy0, sigma_yaw0, d0=1.5):
    """Desviación estándar de una medición: crece con d² y baja con √n."""
    k = max(1.0, distance / d0) ** 2 / math.sqrt(max(1, n_markers))
    return sigma_xy0 * k, sigma_yaw0 * k


# ── Fusión ────────────────────────────────────────────────────
class OdomHistory:
    """Historial corto de poses de odometría para compensar el retraso del video."""

    def __init__(self, max_age=2.0):
        self.max_age = max_age
        self.buf = deque()

    def add(self, t, x, y, yaw):
        self.buf.append((t, x, y, yaw))
        while self.buf and t - self.buf[0][0] > self.max_age:
            self.buf.popleft()

    def at(self, t, tolerance=0.1):
        """Pose (x, y, yaw) interpolada en ``t`` o None si está fuera del historial."""
        if not self.buf or t < self.buf[0][0] - tolerance or t > self.buf[-1][0] + tolerance:
            return None
        if t >= self.buf[-1][0]:
            return self.buf[-1][1:]
        if t <= self.buf[0][0]:
            return self.buf[0][1:]
        prev = self.buf[0]
        for cur in self.buf:
            if cur[0] >= t:
                a = (t - prev[0]) / max(cur[0] - prev[0], 1e-9)
                return (prev[1] + a * (cur[1] - prev[1]),
                        prev[2] + a * (cur[2] - prev[2]),
                        normalize_angle(prev[3] + a * normalize_angle(cur[3] - prev[3])))
            prev = cur
        return self.buf[-1][1:]


class OffsetFilter:
    """Kalman de 3 estados del error de la odometría: pose_corral = offset ∘ pose_odom.

    La incertidumbre crece con el tiempo (la odometría deriva) y cada medición
    visual la reduce según su covarianza, así que las dos cámaras se combinan
    solas. Mediciones con innovación grande se rechazan; tras ``max_rejects``
    rechazos seguidos se reinicia con la última (re-localización).
    """

    def __init__(self, q_xy=0.0025, q_yaw=1e-4, gate_xy=0.5,
                 gate_yaw=math.radians(15.0), max_rejects=5):
        self.q_xy = q_xy
        self.q_yaw = q_yaw
        self.gate_xy = gate_xy
        self.gate_yaw = gate_yaw
        self.max_rejects = max_rejects
        self.reset()

    def reset(self):
        self.offset = [0.0, 0.0, 0.0]
        self.P = [0.0, 0.0, 0.0]
        self.initialized = False
        self.rejects = 0
        self.last_t = None

    def predict(self, t):
        if self.last_t is not None and t > self.last_t:
            dt = t - self.last_t
            self.P[0] += self.q_xy * dt
            self.P[1] += self.q_xy * dt
            self.P[2] += self.q_yaw * dt
        self.last_t = t if self.last_t is None else max(t, self.last_t)

    def apply(self, x, y, yaw):
        """Pose de odometría → pose en el corral."""
        dx, dy, dyaw = self.offset
        rx, ry = rotate2d(x, y, dyaw)
        return rx + dx, ry + dy, normalize_angle(yaw + dyaw)

    def _init_from(self, meas, odom, sigma_xy, sigma_yaw):
        dyaw = normalize_angle(meas[2] - odom[2])
        rx, ry = rotate2d(odom[0], odom[1], dyaw)
        self.offset = [meas[0] - rx, meas[1] - ry, dyaw]
        self.P = [sigma_xy ** 2, sigma_xy ** 2, sigma_yaw ** 2]
        self.initialized = True
        self.rejects = 0

    def update(self, meas, odom, sigma_xy, sigma_yaw):
        """Incorpora una medición visual.

        Args:
            meas: (x, y, yaw) visual en el corral, en el instante de la imagen.
            odom: (x, y, yaw) de odometría en ese mismo instante.

        Returns:
            'init', 'reinit', 'ok' o 'rejected'
        """
        if not self.initialized:
            self._init_from(meas, odom, sigma_xy, sigma_yaw)
            return 'init'
        px, py, pyaw = self.apply(*odom)
        innov_yaw = normalize_angle(meas[2] - pyaw)
        if math.hypot(meas[0] - px, meas[1] - py) > self.gate_xy or abs(innov_yaw) > self.gate_yaw:
            self.rejects += 1
            if self.rejects >= self.max_rejects:
                # shortcut: no se comprueba que los rechazos sean coherentes entre sí
                self._init_from(meas, odom, sigma_xy, sigma_yaw)
                return 'reinit'
            return 'rejected'
        self.rejects = 0
        k = self.P[2] / (self.P[2] + sigma_yaw ** 2)
        self.offset[2] = normalize_angle(self.offset[2] + k * innov_yaw)
        self.P[2] *= (1.0 - k)
        rx, ry = rotate2d(odom[0], odom[1], self.offset[2])
        for i, (m, r) in enumerate(((meas[0], rx), (meas[1], ry))):
            k = self.P[i] / (self.P[i] + sigma_xy ** 2)
            self.offset[i] += k * (m - r - self.offset[i])
            self.P[i] *= (1.0 - k)
        return 'ok'
