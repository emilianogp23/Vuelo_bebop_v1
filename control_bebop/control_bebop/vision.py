"""
Localización visual con ArUco (lógica pura, sin ROS) para el Bebop 2.

Marco "corral": origen en el centro de la zona segura de despegue, en el piso;
x hacia donde apunta el dron al despegar, y a la izquierda, z arriba. Si el
dron despega en el origen mirando a +x, coincide con el marco de despegue.

La cámara del Bebop ve marcadores en las vallas y el piso (mapa conocido)
→ pose de la cámara → pose del dron. La posición la da la odometría; cada
detección la corrige en ``OffsetFilter``: un Kalman de 3 estados que estima el
error acumulado (dx, dy, dyaw) de la odometría. Entre detecciones (puede pasar
un buen rato sin ver marcadores) solo crece su incertidumbre.
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
        (dictionary_name, {id: esquinas (4, 3)})
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
    return data.get('dictionary', 'DICT_4X4_50'), corners


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
    # El IPPE de OpenCV falla si el plano no pasa por el origen (piso a z 0.003):
    # se resuelve con los puntos centrados y se deshace al final
    center = obj.mean(axis=0)
    obj = obj - center
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
    R_cw, _ = cv2.Rodrigues(rvec)
    return rvec.reshape(3), tvec.reshape(3) - R_cw @ center, err


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
    visual la reduce según su covarianza. Mediciones a más de ``gate_sigma``
    desviaciones de lo esperado se rechazan. No hay re-localización forzada:
    mientras se rechaza, la incertidumbre sigue creciendo y el gate se abre
    solo (un error real de 0.5 m entra en ~10 s; una racha de PnP espejo de un
    marcador lejano, de 2-3 m, se queda fuera).

    ``offset`` es la estimación; ``apply`` usa ``applied``, que la sigue a lo
    más a ``rate_xy`` m/s y ``rate_yaw`` rad/s para que una corrección después
    de un hueco no le meta un escalón al control. En init salta.
    """

    def __init__(self, q_xy=0.0025, q_yaw=1e-4, gate_sigma=3.0,
                 rate_xy=0.10, rate_yaw=math.radians(5.0)):
        self.q_xy = q_xy
        self.q_yaw = q_yaw
        self.gate_sigma = gate_sigma
        self.rate_xy = rate_xy
        self.rate_yaw = rate_yaw
        self.reset()

    def reset(self, sigma_xy=None, sigma_yaw=None):
        """Sin sigmas: sin inicializar. Con sigmas: offset cero (corral = odometría)."""
        self.offset = [0.0, 0.0, 0.0]
        self.applied = [0.0, 0.0, 0.0]
        self.initialized = sigma_xy is not None
        self.P = ([sigma_xy ** 2, sigma_xy ** 2, sigma_yaw ** 2] if self.initialized
                  else [0.0, 0.0, 0.0])
        self.last_t = None

    @property
    def sigma_xy(self):
        return math.sqrt(max(self.P[0], self.P[1])) if self.initialized else math.inf

    def predict(self, t):
        if self.last_t is not None and t > self.last_t:
            dt = t - self.last_t
            self.P[0] += self.q_xy * dt
            self.P[1] += self.q_xy * dt
            self.P[2] += self.q_yaw * dt
            for i, rate in ((0, self.rate_xy), (1, self.rate_xy), (2, self.rate_yaw)):
                err = self.offset[i] - self.applied[i]
                if i == 2:
                    err = normalize_angle(err)
                step = max(-rate * dt, min(rate * dt, err))
                self.applied[i] += step
            self.applied[2] = normalize_angle(self.applied[2])
        self.last_t = t if self.last_t is None else max(t, self.last_t)

    @staticmethod
    def _compose(offset, x, y, yaw):
        dx, dy, dyaw = offset
        rx, ry = rotate2d(x, y, dyaw)
        return rx + dx, ry + dy, normalize_angle(yaw + dyaw)

    def apply(self, x, y, yaw):
        """Pose de odometría → pose en el corral (con la corrección suavizada)."""
        return self._compose(self.applied, x, y, yaw)

    def _init_from(self, meas, odom, sigma_xy, sigma_yaw):
        dyaw = normalize_angle(meas[2] - odom[2])
        rx, ry = rotate2d(odom[0], odom[1], dyaw)
        self.offset = [meas[0] - rx, meas[1] - ry, dyaw]
        self.applied = list(self.offset)
        self.P = [sigma_xy ** 2, sigma_xy ** 2, sigma_yaw ** 2]
        self.initialized = True

    def update(self, meas, odom, sigma_xy, sigma_yaw):
        """Incorpora una medición visual.

        Args:
            meas: (x, y, yaw) visual en el corral, en el instante de la imagen.
            odom: (x, y, yaw) de odometría en ese mismo instante.

        Returns:
            'init', 'ok' o 'rejected'
        """
        if not self.initialized:
            self._init_from(meas, odom, sigma_xy, sigma_yaw)
            return 'init'
        px, py, pyaw = self._compose(self.offset, *odom)
        innov_yaw = normalize_angle(meas[2] - pyaw)
        s_xy = (self.P[0] + sigma_xy ** 2, self.P[1] + sigma_xy ** 2)
        d2_xy = (meas[0] - px) ** 2 / s_xy[0] + (meas[1] - py) ** 2 / s_xy[1]
        d2_yaw = innov_yaw ** 2 / (self.P[2] + sigma_yaw ** 2)
        if max(d2_xy, d2_yaw) > self.gate_sigma ** 2:
            return 'rejected'
        k = self.P[2] / (self.P[2] + sigma_yaw ** 2)
        self.offset[2] = normalize_angle(self.offset[2] + k * innov_yaw)
        self.P[2] *= (1.0 - k)
        rx, ry = rotate2d(odom[0], odom[1], self.offset[2])
        for i, (m, r) in enumerate(((meas[0], rx), (meas[1], ry))):
            k = self.P[i] / (self.P[i] + sigma_xy ** 2)
            self.offset[i] += k * (m - r - self.offset[i])
            self.P[i] *= (1.0 - k)
        return 'ok'


class VisionFusion:
    """Odometría + poses ArUco. La usan real_drone_bridge y sim_odometry.

    ``locked``: la posición es confiable (incertidumbre < ``max_sigma``). No
    depende de ver marcadores ahora: entre detecciones manda la odometría.
    """

    def __init__(self, max_sigma=0.20, init_sigma_xy=0.05,
                 init_sigma_yaw=math.radians(3.0), max_dz=0.30, **filter_kw):
        self.max_sigma = max_sigma
        self.max_dz = max_dz
        self.init_sigma = (init_sigma_xy, init_sigma_yaw)
        self.hist = OdomHistory()
        self.filter = OffsetFilter(**filter_kw)

    def takeoff(self, reanchor):
        """Al despegar. ``reanchor``: el marco de la odometría se reinició aquí.

        El dron despega en el origen del corral mirando a +x (aruco_map.yaml),
        así que sin otra medición el offset arranca en cero.
        """
        if reanchor:
            self.hist = OdomHistory()
        if reanchor or not self.filter.initialized:
            self.filter.reset(*self.init_sigma)

    def odom(self, t, x, y, yaw):
        """Pose corregida (x, y, yaw) o None si el filtro no está inicializado."""
        self.hist.add(t, x, y, yaw)
        self.filter.predict(t)
        return self.filter.apply(x, y, yaw) if self.filter.initialized else None

    def vision(self, t, meas, sigma_xy, sigma_yaw, z_err=None):
        """'init', 'ok', 'rejected', 'rejected_z' o 'sin_odom'.

        ``z_err``: z visual − altura medida; si pasa de ``max_dz`` el PnP está mal.
        """
        if z_err is not None and abs(z_err) > self.max_dz:
            return 'rejected_z'
        odom = self.hist.at(t)
        if odom is None:
            return 'sin_odom'
        return self.filter.update(meas, odom, sigma_xy, sigma_yaw)

    @property
    def locked(self):
        return self.filter.sigma_xy < self.max_sigma
