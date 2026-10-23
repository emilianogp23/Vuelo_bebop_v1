"""
Calibración de la cámara del Bebop 2 con un tablero ChArUco (lógica sin ROS).

Un tablero ChArUco es un tablero de ajedrez con un ArUco en cada cuadro
blanco: las esquinas se identifican aunque el tablero se vea incompleto. El
diccionario por defecto (DICT_5X5_100) es distinto del del corral (DICT_4X4_50)
para que aruco_localizer nunca confunda el tablero con un marcador del mapa.

Funciona con la API vieja de OpenCV (< 4.7, python3-opencv de Ubuntu) y con
la nueva. Usar un número impar de filas (squares_y): con filas pares las dos
API dibujan tableros distintos.

Lo usa el nodo calibrar_camara; el resultado se escribe en el formato de
camera_info_manager que carga ros2_bebop_driver (camera_calibration_file).
"""

import math

import cv2
import numpy as np

OLD_API = hasattr(cv2.aruco, 'CharucoBoard_create')


class Charuco:
    """Tablero ChArUco y su detector."""

    def __init__(self, squares_x=7, squares_y=5, square=0.035, marker=0.026,
                 dictionary='DICT_5X5_100'):
        self.size = (squares_x, squares_y)
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        if OLD_API:
            self.board = cv2.aruco.CharucoBoard_create(
                squares_x, squares_y, square, marker, self.dictionary)
            self.corners3d = np.asarray(self.board.chessboardCorners, dtype=np.float32)
        else:
            self.board = cv2.aruco.CharucoBoard(self.size, square, marker, self.dictionary)
            self.corners3d = np.asarray(self.board.getChessboardCorners(), dtype=np.float32)
            self.detector = cv2.aruco.CharucoDetector(self.board)

    @property
    def n_corners(self):
        return len(self.corners3d)

    def image(self, width_px, height_px, margin_px=0):
        draw = self.board.draw if OLD_API else self.board.generateImage
        return draw((width_px, height_px), marginSize=margin_px)

    def detect(self, gray):
        """(esquinas (N, 2), ids (N,)) de las esquinas internas vistas, o (None, None)."""
        if OLD_API:
            markers, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
            if ids is None:
                return None, None
            n, corners, cids = cv2.aruco.interpolateCornersCharuco(markers, ids, gray, self.board)
            if not n:
                return None, None
        else:
            corners, cids, _, _ = self.detector.detectBoard(gray)
            if cids is None:
                return None, None
        return corners.reshape(-1, 2).astype(np.float32), cids.flatten().astype(int)


def nominal_camera(image_size):
    """K aproximada (fx ≈ 0.63·ancho, como el Bebop) para medir la pose del tablero."""
    w, h = image_size
    f = 0.63 * w
    return np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1]], dtype=np.float64)


def view_features(charuco, corners, ids, image_size):
    """(x, y, tamaño, inclinación x, inclinación y) normalizados de una vista.

    Sirven para pedir variedad: el tablero en todo el cuadro, cerca y lejos, e
    inclinado (sin inclinación la distancia focal queda mal determinada).
    """
    w, h = image_size
    cx, cy = corners.mean(axis=0)
    area = cv2.contourArea(cv2.convexHull(corners))
    tilt = (0.0, 0.0)
    if len(ids) >= 6:
        ok, rvec, _ = cv2.solvePnP(charuco.corners3d[ids], corners, nominal_camera(image_size),
                                   None, flags=cv2.SOLVEPNP_ITERATIVE)
        if ok:
            normal = cv2.Rodrigues(rvec)[0][:, 2]
            tilt = (float(normal[0]), float(normal[1]))
    return np.array([cx / w, cy / h, math.sqrt(area / (w * h)), *tilt])


def is_new_view(features, previous, min_diff=0.15):
    return all(np.linalg.norm(features - p) >= min_diff for p in previous)


def coverage(previous):
    """Rangos cubiertos: x, y, tamaño y la inclinación máxima en grados."""
    if not previous:
        return None
    f = np.array(previous)
    tilt = np.degrees(np.arcsin(np.clip(np.hypot(f[:, 3], f[:, 4]), 0.0, 1.0)))
    return {'x': (f[:, 0].min(), f[:, 0].max()), 'y': (f[:, 1].min(), f[:, 1].max()),
            'size': (f[:, 2].min(), f[:, 2].max()), 'tilt_max_deg': float(tilt.max())}


def calibrate(charuco, views, image_size):
    """Calibra con las vistas [(esquinas, ids), ...].

    Returns:
        (rms_px, K, D (5,), error_px por vista)
    """
    obj = [charuco.corners3d[ids] for _, ids in views]
    img = [corners for corners, _ in views]
    # K3 fija como el calibrador de ROS: con pocas vistas sobreajusta el borde
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        obj, img, tuple(image_size), None, None, flags=cv2.CALIB_FIX_K3)
    errors = []
    for o, i, r, t in zip(obj, img, rvecs, tvecs):
        proj, _ = cv2.projectPoints(o, r, t, K, D)
        errors.append(float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - i) ** 2, axis=1)))))
    return float(rms), K, D.flatten()[:5], errors


def _matrix(name, rows, cols, values):
    data = ', '.join(f'{v:.6f}' for v in np.asarray(values, dtype=float).flatten())
    return f'{name}:\n  rows: {rows}\n  cols: {cols}\n  data: [{data}]\n'


def camera_info_yaml(K, D, image_size, camera_name='bebop_front', comment=''):
    """Texto en el formato de camera_info_manager (el de bebop2_camera_calib.yaml)."""
    w, h = image_size
    new_k, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 0.0)
    P = np.hstack([new_k, np.zeros((3, 1))])
    head = ''.join(f'# {line}\n' for line in comment.splitlines())
    return (head + f'image_width: {w}\nimage_height: {h}\ncamera_name: {camera_name}\n'
            + _matrix('camera_matrix', 3, 3, K)
            + 'distortion_model: plumb_bob\n'
            + _matrix('distortion_coefficients', 1, 5, D)
            + _matrix('rectification_matrix', 3, 3, np.eye(3))
            + _matrix('projection_matrix', 3, 4, P))
