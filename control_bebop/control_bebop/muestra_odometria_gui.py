#!/usr/bin/env python3
"""
GUI de la Muestra Profesional del Parrot Bebop 2 (ROS 2, PyQt5).

Vistas (botones del encabezado o teclas 1, 2, 3):
    Mapa        mapa de odometría 2D o 3D (tecla M); el 3D muestra la altura y el dron
    Cámara      video del dron con detección YOLOv8 (hilo aparte, solo el último frame)
    Dividida    mapa y cámara lado a lado

Además: horizonte artificial, telemetría, monitor del mando PS5 y botones de
despegue, aterrizaje y emergencia (van por /demo/command al demo_safety_manager).
La inclinación de la cámara (/bebop/move_camera) se maneja solo desde aquí.

Parámetros ROS: gui_view (map|camera|split), gui_map (2d|3d), arena_x_max, max_z y
safe_zone_radius (los mismos de real_drone.yaml, dibujan el corral).
"""

import collections
import csv
import math
import os
import signal
import sys
import threading
import time
from datetime import datetime

# Entorno virtual ~/rob_env (ultralytics, torch) de la versión de Python actual
from control_bebop.paths import add_user_venvs, data_dir  # noqa: E402
add_user_venvs(('rob_env',))

import cv2  # noqa: E402

# Conflicto entre el Qt de opencv-python y el PyQt5 del sistema en Linux
os.environ.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)
_QT_PLUGINS = '/usr/lib/x86_64-linux-gnu/qt5/plugins'
if os.path.isdir(_QT_PLUGINS):
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = _QT_PLUGINS

from PyQt5 import QtWidgets  # noqa: E402
from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer  # noqa: E402
from PyQt5.QtGui import (QColor, QFont, QImage, QKeySequence, QPainter,  # noqa: E402
                         QPainterPath, QPen, QPixmap, QPolygonF)
from PyQt5.QtWidgets import (QApplication, QButtonGroup, QCheckBox,  # noqa: E402
                             QComboBox, QFileDialog, QFrame, QGridLayout, QGroupBox,
                             QHBoxLayout, QLabel, QMainWindow, QPushButton, QShortcut,
                             QSizePolicy, QSlider, QSplitter, QStackedWidget,
                             QVBoxLayout, QWidget)

import rclpy  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from geometry_msgs.msg import Pose, Vector3  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rclpy.utilities import remove_ros_args  # noqa: E402
from sensor_msgs.msg import Image, Joy  # noqa: E402
from std_msgs.msg import Bool, Int32, String  # noqa: E402

from .yolo_person_detector import YoloPersonDetector  # noqa: E402

# ─────────────────────────────────────────────────────────────────────────────
#  TEMA: oscuro minimalista. Un solo acento; verde/ámbar/rojo solo para estado.
# ─────────────────────────────────────────────────────────────────────────────
THEME = {
    'bg': '#1b1d21',
    'plot': '#16181b',
    'surface': '#23262b',
    'surface_alt': '#2c3036',
    'border': '#33373d',
    'text': '#d7dae0',
    'muted': '#8a8f98',
    'accent': '#6aa3d8',
    'accent_dim': '#2b3d50',
    'ok': '#6fbf8b',
    'ok_dim': '#233229',
    'warn': '#d9b25f',
    'warn_dim': '#38311f',
    'danger': '#d97575',
    'danger_dim': '#3a2527',
}

FONT_UI = 'Ubuntu'
FONT_MONO = 'DejaVu Sans Mono'


def qc(name, alpha=255):
    c = QColor(THEME[name])
    c.setAlpha(alpha)
    return c


STYLESHEET = f"""
QMainWindow, QWidget {{ background-color: {THEME['bg']}; color: {THEME['text']};
    font-family: '{FONT_UI}', 'Segoe UI', 'DejaVu Sans', sans-serif; font-size: 13px; }}
QLabel {{ background: transparent; }}
QLabel#muted {{ color: {THEME['muted']}; }}
QLabel#title {{ font-size: 15px; font-weight: 600; }}
QFrame#bar {{ background-color: {THEME['surface']}; border: 1px solid {THEME['border']};
    border-radius: 6px; }}
QGroupBox {{ background-color: {THEME['surface']}; border: 1px solid {THEME['border']};
    border-radius: 6px; margin-top: 14px; padding: 10px 8px 8px 8px; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px;
    color: {THEME['muted']}; background-color: {THEME['bg']}; }}
QPushButton {{ background-color: {THEME['surface']}; border: 1px solid {THEME['border']};
    border-radius: 4px; padding: 6px 12px; color: {THEME['text']}; }}
QPushButton:hover {{ background-color: {THEME['surface_alt']}; }}
QPushButton:pressed {{ background-color: {THEME['border']}; }}
QPushButton:checked {{ background-color: {THEME['accent_dim']}; border-color: {THEME['accent']};
    color: {THEME['accent']}; }}
QPushButton#primary {{ color: {THEME['ok']}; border-color: {THEME['ok_dim']}; }}
QPushButton#primary:hover {{ background-color: {THEME['ok_dim']}; }}
QPushButton#warn {{ color: {THEME['warn']}; border-color: {THEME['warn_dim']}; }}
QPushButton#warn:hover {{ background-color: {THEME['warn_dim']}; }}
QPushButton#danger {{ color: {THEME['danger']}; border-color: {THEME['danger_dim']}; }}
QPushButton#danger:hover {{ background-color: {THEME['danger_dim']}; }}
QPushButton#flat {{ padding: 4px 10px; }}
QCheckBox {{ spacing: 6px; }}
QComboBox {{ background-color: {THEME['surface']}; border: 1px solid {THEME['border']};
    border-radius: 4px; padding: 4px 8px; }}
QComboBox QAbstractItemView {{ background-color: {THEME['surface']};
    selection-background-color: {THEME['accent_dim']}; border: 1px solid {THEME['border']}; }}
QSlider::groove:horizontal {{ height: 4px; background: {THEME['border']}; border-radius: 2px; }}
QSlider::sub-page:horizontal {{ background: {THEME['accent']}; border-radius: 2px; }}
QSlider::handle:horizontal {{ width: 12px; height: 12px; margin: -4px 0;
    background: {THEME['text']}; border-radius: 6px; }}
QSplitter::handle {{ background-color: {THEME['border']}; }}
QSplitter::handle:horizontal {{ width: 1px; }}
QToolTip {{ background-color: {THEME['surface_alt']}; color: {THEME['text']};
    border: 1px solid {THEME['border']}; }}
"""


def chip_style(fg, bg):
    return (f'color: {THEME[fg]}; background-color: {THEME[bg]}; border-radius: 4px; '
            'padding: 3px 10px;')


# ─────────────────────────────────────────────────────────────────────────────
#  GEOMETRÍA (sin Qt, para poder probarla)
# ─────────────────────────────────────────────────────────────────────────────
def euler_from_quaternion(x, y, z, w):
    """Roll, pitch, yaw (rad) desde un cuaternión."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def project_point(p, az, el, scale=1.0):
    """Proyección ortográfica de un punto del mundo (x adelante, y izquierda, z arriba).

    ``az``: giro de la vista alrededor de z (rad). ``el``: elevación de la cámara (rad),
    0 = a ras de piso, π/2 = vista desde arriba. Devuelve (u, v, profundidad): u a la
    derecha y v HACIA ARRIBA en pantalla; mayor profundidad = más lejos de la cámara.
    """
    x, y, z = p
    c, s = math.cos(az), math.sin(az)
    right = c * x + s * y
    away = -s * x + c * y
    v = z * math.cos(el) + away * math.sin(el)
    depth = away * math.cos(el) - z * math.sin(el)
    return right * scale, v * scale, depth


def body_to_world(p, roll, pitch, yaw, origin=(0.0, 0.0, 0.0)):
    """Punto del cuerpo del dron (x adelante, y izquierda, z arriba) al mundo: Rz·Ry·Rx."""
    x, y, z = p
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    y1, z1 = cr * y - sr * z, sr * y + cr * z                    # roll (x)
    x2, z2 = cp * x + sp * z1, -sp * x + cp * z1                  # pitch (y)
    x3, y3 = cy * x2 - sy * y1, sy * x2 + cy * y1                 # yaw (z)
    return origin[0] + x3, origin[1] + y3, origin[2] + z2


def circle_points(cx, cy, z, r, n=40):
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n), z)
            for k in range(n + 1)]


def _mono(size, bold=False):
    f = QFont(FONT_MONO, size)
    f.setBold(bold)
    return f


def _ui(size, bold=False):
    f = QFont(FONT_UI, size)
    f.setBold(bold)
    return f


def _decimate(points, limit=900):
    step = max(1, len(points) // limit)
    return points[::step]


# ─────────────────────────────────────────────────────────────────────────────
#  MAPA 2D
# ─────────────────────────────────────────────────────────────────────────────
class SpatialMapWidget(QWidget):
    """Vista cenital: x del dron hacia arriba en pantalla, y hacia la izquierda."""

    def __init__(self, arena=2.0, safe_radius=0.5, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 260)
        self.arena = arena
        self.safe_radius = safe_radius
        self.points = []           # (x, y, z, velocidad, yaw)
        self.cur = (0.0, 0.0, 0.0)
        self.cur_yaw = 0.0
        self.scale = 90.0
        self.auto_fit = True
        self.follow = False
        self.pan = QPointF(0, 0)
        self.origin_offset = (0.0, 0.0)
        self._last_mouse = None

    def add_point(self, x, y, z, vx, vy, vz, yaw):
        ax, ay = x - self.origin_offset[0], y - self.origin_offset[1]
        self.cur, self.cur_yaw = (ax, ay, z), yaw
        speed = math.hypot(vx, vy)
        if not self.points or math.hypot(ax - self.points[-1][0], ay - self.points[-1][1]) > 0.015:
            self.points.append((ax, ay, z, speed, yaw))
            if len(self.points) > 4000:
                self.points.pop(0)
        self.update()

    def set_attitude(self, roll_deg, pitch_deg):
        pass    # la vista cenital solo usa el rumbo

    def reset_map(self, raw_x=None, raw_y=None):
        if raw_x is not None and raw_y is not None:
            self.origin_offset = (raw_x, raw_y)
        self.points.clear()
        self.cur = (0.0, 0.0, self.cur[2])
        self.pan = QPointF(0, 0)
        self.update()

    def reset_view(self):
        self.auto_fit = True
        self.follow = False
        self.pan = QPointF(0, 0)
        self.update()

    def set_follow(self, on):
        self.follow = on
        self.update()

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._last_mouse = e.pos()

    def mouseMoveEvent(self, e):
        if e.buttons() & Qt.LeftButton and self._last_mouse is not None:
            d = e.pos() - self._last_mouse
            self.pan += QPointF(d.x(), d.y())
            self._last_mouse = e.pos()
            self.update()

    def mouseReleaseEvent(self, _):
        self._last_mouse = None

    def wheelEvent(self, e):
        self.auto_fit = False
        self.scale = max(15.0, min(600.0, self.scale * (1.15 if e.angleDelta().y() > 0 else 0.87)))
        self.update()

    def _to_px(self, x, y, cx, cy):
        return QPointF(cx - y * self.scale, cy - x * self.scale)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        if self.auto_fit:
            self.scale = max(15.0, min(w, h) / (2.0 * (self.arena + 0.45)))
        cx, cy = w / 2.0 + self.pan.x(), h / 2.0 + self.pan.y()
        if self.follow:
            cx += self.cur[1] * self.scale
            cy += self.cur[0] * self.scale
        p.fillRect(self.rect(), qc('plot'))
        a = self.arena

        # Corral y cuadrícula de 0.5 m
        corral = QRectF(self._to_px(a, a, cx, cy), self._to_px(-a, -a, cx, cy))
        p.fillRect(corral, qc('surface', 140))
        k = -a
        while k <= a + 1e-6:
            major = abs(k - round(k)) < 1e-6
            p.setPen(QPen(qc('border', 170 if major else 80), 1))
            p.drawLine(self._to_px(a, k, cx, cy), self._to_px(-a, k, cx, cy))
            p.drawLine(self._to_px(k, a, cx, cy), self._to_px(k, -a, cx, cy))
            k += 0.5
        p.setPen(QPen(qc('muted', 160), 1.2))
        p.setBrush(Qt.NoBrush)
        p.drawRect(corral)

        # Ejes con marcas cada metro
        p.setPen(QPen(qc('muted', 120), 1))
        p.drawLine(self._to_px(a, 0, cx, cy), self._to_px(-a, 0, cx, cy))
        p.drawLine(self._to_px(0, a, cx, cy), self._to_px(0, -a, cx, cy))
        p.setFont(_ui(8))
        p.setPen(qc('muted'))
        for m in range(1, int(a) + 1):
            p.drawText(self._to_px(m, 0, cx, cy) + QPointF(4, 12), f'{m} m')
            p.drawText(self._to_px(0, m, cx, cy) + QPointF(4, 12), f'{m} m')
        p.drawText(self._to_px(a + 0.16, 0, cx, cy) + QPointF(-3, 4), 'x')
        p.drawText(self._to_px(0, a + 0.16, cx, cy) + QPointF(-3, 4), 'y')

        # Zona segura y origen
        if self.safe_radius > 0:
            r = self.safe_radius * self.scale
            p.setPen(QPen(qc('muted', 150), 1, Qt.DashLine))
            p.drawEllipse(QPointF(cx, cy), r, r)
        p.setPen(QPen(qc('muted'), 1.5))
        o = self._to_px(0, 0, cx, cy)
        p.drawLine(o + QPointF(-5, 0), o + QPointF(5, 0))
        p.drawLine(o + QPointF(0, -5), o + QPointF(0, 5))

        # Trayectoria
        pts = _decimate(self.points)
        if len(pts) > 1:
            path = QPainterPath(self._to_px(pts[0][0], pts[0][1], cx, cy))
            for q in pts[1:]:
                path.lineTo(self._to_px(q[0], q[1], cx, cy))
            p.setPen(QPen(qc('accent', 210), 2, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            p.setBrush(Qt.NoBrush)
            p.drawPath(path)

        # Dron: cruz con cuatro rotores y flecha de rumbo
        d = self._to_px(self.cur[0], self.cur[1], cx, cy)
        p.save()
        p.translate(d)
        p.rotate(-math.degrees(self.cur_yaw))
        arm = 12
        p.setPen(QPen(qc('text'), 2))
        p.drawLine(-arm, -arm, arm, arm)
        p.drawLine(-arm, arm, arm, -arm)
        p.setBrush(qc('plot'))
        for sx, sy in ((-1, -1), (1, -1), (-1, 1), (1, 1)):
            p.drawEllipse(QPointF(sx * arm, sy * arm), 5, 5)
        p.setPen(QPen(qc('accent'), 2))
        p.setBrush(qc('accent'))
        p.drawPolygon(QPolygonF([QPointF(0, -20), QPointF(-5, -10), QPointF(5, -10)]))
        p.restore()

        p.setPen(qc('muted'))
        p.setFont(_mono(9))
        p.drawText(12, 20, f'X {self.cur[0]:+.2f}   Y {self.cur[1]:+.2f}   Z {self.cur[2]:+.2f} m')
        p.drawText(12, 36, f'Rumbo {math.degrees(self.cur_yaw):+.0f}°')


# ─────────────────────────────────────────────────────────────────────────────
#  MAPA 3D (QPainter, sin dependencias)
# ─────────────────────────────────────────────────────────────────────────────
class SpatialMap3DWidget(QWidget):
    """Escena 3D: piso, corral, techo, trayectoria con su sombra, dron y su altura."""

    AZ0, EL0 = math.radians(-60.0), math.radians(25.0)

    def __init__(self, arena=2.0, ceiling=2.0, safe_radius=0.5, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 260)
        self.arena = arena
        self.ceiling = ceiling
        self.safe_radius = safe_radius
        self.points = []
        self.cur = (0.0, 0.0, 0.0)
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.az, self.el = self.AZ0, self.EL0
        self.scale = 80.0
        self.auto_fit = True
        self.follow = False
        self.origin_offset = (0.0, 0.0)
        self._last_mouse = None

    def add_point(self, x, y, z, vx, vy, vz, yaw):
        ax, ay = x - self.origin_offset[0], y - self.origin_offset[1]
        self.cur, self.yaw = (ax, ay, z), yaw
        if not self.points or math.hypot(ax - self.points[-1][0], ay - self.points[-1][1]) > 0.015 \
                or abs(z - self.points[-1][2]) > 0.02:
            self.points.append((ax, ay, z, 0.0, yaw))
            if len(self.points) > 4000:
                self.points.pop(0)
        self.update()

    def set_attitude(self, roll_deg, pitch_deg):
        self.roll, self.pitch = math.radians(roll_deg), math.radians(pitch_deg)
        self.update()

    def reset_map(self, raw_x=None, raw_y=None):
        if raw_x is not None and raw_y is not None:
            self.origin_offset = (raw_x, raw_y)
        self.points.clear()
        self.cur = (0.0, 0.0, self.cur[2])
        self.update()

    def reset_view(self):
        self.az, self.el = self.AZ0, self.EL0
        self.auto_fit = True
        self.follow = False
        self.update()

    def set_follow(self, on):
        self.follow = on
        self.update()

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._last_mouse = e.pos()

    def mouseMoveEvent(self, e):
        if e.buttons() & Qt.LeftButton and self._last_mouse is not None:
            d = e.pos() - self._last_mouse
            self.az += d.x() * 0.008
            self.el = max(math.radians(5.0), min(math.radians(85.0), self.el + d.y() * 0.008))
            self._last_mouse = e.pos()
            self.update()

    def mouseReleaseEvent(self, _):
        self._last_mouse = None

    def wheelEvent(self, e):
        self.auto_fit = False
        self.scale = max(25.0, min(500.0, self.scale * (1.12 if e.angleDelta().y() > 0 else 0.89)))
        self.update()

    # ── Dibujo ──
    def _pt(self, q, target, w, h):
        u, v, _ = project_point((q[0] - target[0], q[1] - target[1], q[2] - target[2]),
                                self.az, self.el, self.scale)
        return QPointF(w / 2.0 + u, h / 2.0 - v)

    def _poly(self, pts, target, w, h):
        return QPolygonF([self._pt(q, target, w, h) for q in pts])

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        if self.auto_fit:
            self.scale = max(25.0, min(w, h * 1.15) / (self.arena * 3.0 + 0.8))
        a, top = self.arena, self.ceiling
        target = self.cur if self.follow else (0.0, 0.0, top / 2.0)
        pt = lambda q: self._pt(q, target, w, h)  # noqa: E731
        p.fillRect(self.rect(), qc('plot'))

        # Piso, cuadrícula de 0.5 m y borde del corral
        floor = [(a, a, 0), (a, -a, 0), (-a, -a, 0), (-a, a, 0)]
        p.setPen(Qt.NoPen)
        p.setBrush(qc('surface', 150))
        p.drawPolygon(self._poly(floor, target, w, h))
        k = -a
        while k <= a + 1e-6:
            major = abs(k - round(k)) < 1e-6
            p.setPen(QPen(qc('border', 170 if major else 80), 1))
            p.drawLine(pt((a, k, 0)), pt((-a, k, 0)))
            p.drawLine(pt((k, a, 0)), pt((k, -a, 0)))
            k += 0.5
        p.setPen(QPen(qc('muted', 160), 1.2))
        p.setBrush(Qt.NoBrush)
        p.drawPolygon(self._poly(floor, target, w, h))

        # Techo y postes: dan la escala de la altura
        ceiling = [(cx, cy, top) for cx, cy, _ in floor]
        p.setPen(QPen(qc('muted', 90), 1, Qt.DashLine))
        p.drawPolygon(self._poly(ceiling, target, w, h))
        p.setPen(QPen(qc('muted', 90), 1))
        for cx, cy, _ in floor:
            p.drawLine(pt((cx, cy, 0)), pt((cx, cy, top)))
        # Marcas de altura sobre el poste que cae más a la derecha en pantalla
        post = max(floor, key=lambda q: project_point((q[0], q[1], 0), self.az, self.el)[0])
        p.setFont(_ui(8))
        for m in range(1, int(top) + 1):
            base = pt((post[0], post[1], m))
            p.setPen(QPen(qc('muted', 160), 1))
            p.drawLine(base, base + QPointF(6, 0))
            p.setPen(qc('muted'))
            p.drawText(base + QPointF(9, 4), f'{m} m')

        # Zona segura y origen en el piso
        if self.safe_radius > 0:
            p.setPen(QPen(qc('muted', 150), 1, Qt.DashLine))
            p.setBrush(Qt.NoBrush)
            p.drawPolygon(self._poly(circle_points(0, 0, 0, self.safe_radius), target, w, h))
        p.setPen(QPen(qc('muted'), 1.5))
        p.drawLine(pt((-0.08, 0, 0)), pt((0.08, 0, 0)))
        p.drawLine(pt((0, -0.08, 0)), pt((0, 0.08, 0)))

        # Trayectoria: sombra en el piso y línea en el aire (más opaca cuanto más alto)
        pts = _decimate(self.points)
        if len(pts) > 1:
            shadow = QPainterPath(pt((pts[0][0], pts[0][1], 0)))
            for q in pts[1:]:
                shadow.lineTo(pt((q[0], q[1], 0)))
            p.setPen(QPen(qc('text', 45), 2, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            p.setBrush(Qt.NoBrush)
            p.drawPath(shadow)
            for q0, q1 in zip(pts, pts[1:]):
                alpha = int(90 + 140 * max(0.0, min(1.0, q1[2] / max(top, 0.1))))
                p.setPen(QPen(qc('accent', alpha), 2, Qt.SolidLine, Qt.RoundCap))
                p.drawLine(pt(q0[:3]), pt(q1[:3]))

        # Altura: línea del dron al piso, anillo en el piso y etiqueta
        x, y, z = self.cur
        ground = pt((x, y, 0))
        drone = pt((x, y, z))
        p.setPen(QPen(qc('accent', 170), 1, Qt.DashLine))
        p.drawLine(drone, ground)
        p.setPen(QPen(qc('accent', 200), 1.2))
        p.setBrush(Qt.NoBrush)
        p.drawPolygon(self._poly(circle_points(x, y, 0, 0.06, 24), target, w, h))
        p.setFont(_mono(9))
        p.setPen(qc('text'))
        mid = (drone + ground) / 2.0
        p.drawText(mid + QPointF(8, 4), f'z {z:.2f} m')

        self._draw_drone(p, target, w, h)
        self._draw_gizmo(p, h)

        p.setPen(qc('muted'))
        p.setFont(_mono(9))
        p.drawText(12, 20, f'X {x:+.2f}   Y {y:+.2f}   Z {z:+.2f} m')
        p.drawText(12, 36, f'Rumbo {math.degrees(self.yaw):+.0f}°')

    def _draw_drone(self, p, target, w, h):
        ox, oy, oz = self.cur
        R, r = 0.11, 0.05                       # semieje del brazo y radio del rotor (m)
        world = lambda q: body_to_world(q, self.roll, self.pitch, self.yaw, (ox, oy, oz))  # noqa: E731
        tips = [(R, R, 0), (R, -R, 0), (-R, -R, 0), (-R, R, 0)]
        items = []
        for tip in tips:
            ring = [world((rx, ry, 0)) for rx, ry, _ in circle_points(tip[0], tip[1], 0, r, 20)]
            depth = project_point(tuple(c - t for c, t in zip(world(tip), target)),
                                  self.az, self.el)[2]
            items.append((depth, tip, ring))
        # Más lejos primero
        items.sort(key=lambda it: -it[0])
        for _, tip, ring in items:
            p.setPen(QPen(qc('text', 200), 1.5))
            p.drawLine(self._pt(world((0, 0, 0)), target, w, h),
                       self._pt(world(tip), target, w, h))
        for _, tip, ring in items:
            p.setPen(QPen(qc('text'), 1.2))
            p.setBrush(qc('plot', 220))
            p.drawPolygon(self._poly(ring, target, w, h))
        # Rumbo: flecha hacia el frente
        nose = self._pt(world((0.20, 0, 0)), target, w, h)
        p.setPen(QPen(qc('accent'), 2))
        p.drawLine(self._pt(world((0, 0, 0)), target, w, h), nose)
        p.setBrush(qc('accent'))
        p.drawEllipse(nose, 2.5, 2.5)

    def _draw_gizmo(self, p, h):
        origin = QPointF(40, h - 36)
        p.setFont(_ui(8))
        for label, vec in (('x', (1, 0, 0)), ('y', (0, 1, 0)), ('z', (0, 0, 1))):
            u, v, _ = project_point(vec, self.az, self.el, 26)
            end = origin + QPointF(u, -v)
            p.setPen(QPen(qc('muted'), 1.3))
            p.drawLine(origin, end)
            p.setPen(qc('text'))
            p.drawText(end + QPointF(3, 4), label)


# ─────────────────────────────────────────────────────────────────────────────
#  HORIZONTE ARTIFICIAL
# ─────────────────────────────────────────────────────────────────────────────
class AttitudeIndicatorWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(170, 170)
        self.roll_deg = self.pitch_deg = self.yaw_deg = 0.0

    def update_attitude(self, roll_deg, pitch_deg, yaw_deg):
        self.roll_deg, self.pitch_deg, self.yaw_deg = roll_deg, pitch_deg, yaw_deg
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        r = min(w, h) / 2.0 - 6
        cx, cy = w / 2.0, h / 2.0

        clip = QPainterPath()
        clip.addEllipse(QPointF(cx, cy), r, r)
        p.setClipPath(clip)
        p.save()
        p.translate(cx, cy)
        p.rotate(-self.roll_deg)
        off = self.pitch_deg * 3.0
        p.fillRect(QRectF(-r * 2, -r * 2 + off, r * 4, r * 2), QColor('#35506b'))
        p.fillRect(QRectF(-r * 2, off, r * 4, r * 2), QColor('#4b4339'))
        p.setPen(QPen(qc('text'), 1.5))
        p.drawLine(int(-r * 1.5), int(off), int(r * 1.5), int(off))
        p.setFont(_ui(7))
        for deg in (-20, -10, 10, 20):
            y = off - deg * 3.0
            half = 13 if abs(deg) == 10 else 20
            p.setPen(QPen(qc('text', 160), 1))
            p.drawLine(int(-half), int(y), int(half), int(y))
            p.drawText(int(half + 3), int(y + 3), str(abs(deg)))
        p.restore()
        p.setClipping(False)

        p.setPen(QPen(qc('border'), 2))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(cx, cy), r, r)
        p.setPen(QPen(qc('accent'), 2.5))
        p.drawLine(int(cx - 28), int(cy), int(cx - 10), int(cy))
        p.drawLine(int(cx + 10), int(cy), int(cx + 28), int(cy))
        p.drawEllipse(QPointF(cx, cy), 3, 3)
        p.setPen(qc('text'))
        p.setFont(_mono(9))
        p.drawText(QRectF(0, cy + r - 26, w, 18), Qt.AlignCenter,
                   f'{int(self.yaw_deg % 360):03d}°')


# ─────────────────────────────────────────────────────────────────────────────
#  MONITOR DEL MANDO
# ─────────────────────────────────────────────────────────────────────────────
class DualSenseWidget(QWidget):
    """L2 (deadman), botones principales, stick izquierdo y cruceta."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(250, 92)
        self.buttons = [0] * 16
        self.axes = [0.0] * 8
        self.deadman = False

    def update_joy(self, buttons, axes, deadman):
        self.buttons, self.axes, self.deadman = list(buttons), list(axes), deadman
        self.update()

    def _btn(self, i):
        return i < len(self.buttons) and self.buttons[i] == 1

    def _ax(self, i):
        return self.axes[i] if i < len(self.axes) else 0.0

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        # L2 como deadman
        on = self.deadman
        p.setPen(QPen(qc('ok') if on else qc('border'), 1.2))
        p.setBrush(qc('ok_dim') if on else Qt.NoBrush)
        p.drawRoundedRect(QRectF(6, 8, 44, 22), 4, 4)
        p.setPen(qc('ok') if on else qc('muted'))
        p.setFont(_ui(9, True))
        p.drawText(QRectF(6, 8, 44, 22), Qt.AlignCenter, 'L2')

        # × ○ △ □
        p.setFont(_ui(10))
        for k, (sym, idx) in enumerate((('×', 0), ('○', 1), ('△', 2), ('□', 3))):
            rect = QRectF(6 + k * 26, 44, 22, 22)
            act = self._btn(idx)
            p.setPen(QPen(qc('accent') if act else qc('border'), 1.2))
            p.setBrush(qc('accent_dim') if act else Qt.NoBrush)
            p.drawEllipse(rect)
            p.setPen(qc('accent') if act else qc('muted'))
            p.drawText(rect, Qt.AlignCenter, sym)
        p.setFont(_ui(8))
        p.setPen(qc('muted'))
        p.drawText(6, 84, '× despegar   ○ △ aterrizar')

        # Stick izquierdo (ejes 0/1) y cruceta (ejes 6/7)
        for cx, label, ax, ay in ((158, 'Stick izq.', 0, 1), (216, 'Cruceta', 6, 7)):
            cy, rad = 36, 20
            p.setPen(QPen(qc('border'), 1.2))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(cx, cy), rad, rad)
            p.setPen(QPen(qc('border'), 1))
            p.drawLine(QPointF(cx - rad, cy), QPointF(cx + rad, cy))
            p.drawLine(QPointF(cx, cy - rad), QPointF(cx, cy + rad))
            # joy: eje 0 positivo = izquierda, eje 1 positivo = arriba
            dot = QPointF(cx - self._ax(ax) * (rad - 4), cy - self._ax(ay) * (rad - 4))
            p.setPen(Qt.NoPen)
            p.setBrush(qc('accent'))
            p.drawEllipse(dot, 4, 4)
            p.setPen(qc('muted'))
            p.drawText(QRectF(cx - 30, 60, 60, 14), Qt.AlignCenter, label)


# ─────────────────────────────────────────────────────────────────────────────
#  VENTANA FLOTANTE DE LA CÁMARA
# ─────────────────────────────────────────────────────────────────────────────
class FloatingCameraWindow(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Bebop 2 — Cámara')
        self.resize(800, 520)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        self.lbl = QLabel('Esperando video...')
        self.lbl.setAlignment(Qt.AlignCenter)
        self.lbl.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.lbl.setMinimumSize(1, 1)
        self.lbl.setObjectName('muted')
        layout.addWidget(self.lbl)

    def set_pixmap(self, pixmap):
        self.lbl.setPixmap(pixmap.scaled(self.lbl.size(), Qt.KeepAspectRatio,
                                         Qt.SmoothTransformation))


# ─────────────────────────────────────────────────────────────────────────────
#  VENTANA PRINCIPAL
# ─────────────────────────────────────────────────────────────────────────────
STATE_TEXT = {0: 'En tierra', 1: 'En el aire', 2: 'Aterrizando', 4: 'Emergencia'}
STATE_CHIP = {0: ('muted', 'surface_alt'), 1: ('ok', 'ok_dim'), 2: ('warn', 'warn_dim'),
              4: ('danger', 'danger_dim')}
VIEWS = ('map', 'camera', 'split')
YOLO_MODES = ('person', 'demo', 'all')


class MuestraOdometriaGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.ros_node = node
        self.setWindowTitle('Bebop 2 — Odometría y visión')
        self.resize(1280, 800)
        self.setMinimumSize(1200, 680)      # por debajo, la vista dividida no cabe
        self.setStyleSheet(STYLESHEET)

        # Estado compartido con los callbacks de ROS (hilo del executor)
        self.telemetry = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                          'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'speed': 0.0, 'total_dist': 0.0}
        self._pose_queue = collections.deque(maxlen=2000)
        self._prev_pos = None
        self._last_pose_time = 0.0
        self._odom_count = 0
        self._last_hz_calc = time.time()
        self._joy = None
        self._deadman = False
        self._demo_state = 0
        self._manual_active = False
        self._shown_state = None
        self._shown_manual = None
        self.bridge = CvBridge()

        # Cámara + YOLO: el callback solo guarda el último frame; un hilo hace la inferencia
        self.yolo_detector = YoloPersonDetector(conf_threshold=0.25, mode='person')
        self.enable_yolo = True
        self.floating_cam_window = None
        self._raw_frame = None
        self._frame_lock = threading.Lock()
        self._frame_event = threading.Event()
        self.latest_frame = None
        self.new_frame_available = False
        self._stop = threading.Event()
        self._last_img_stamp = None
        self.cam_tilt = 0.0
        self.cam_pan = 0.0
        self.cam_frames_count = 0
        self.cam_total_frames = 0
        self.cam_stream_fps = 0.0
        self._last_cam_fps_time = time.time()
        self.last_cam_frame_time = 0.0

        self._build_ui()
        self._init_ros_io()
        self._yolo_thread = threading.Thread(target=self._yolo_loop, name='yolo_worker',
                                             daemon=True)
        self._yolo_thread.start()

        for key, view in (('1', 'map'), ('2', 'camera'), ('3', 'split')):
            QShortcut(QKeySequence(key), self, activated=lambda v=view: self._set_view(v))
        QShortcut(QKeySequence('M'), self, activated=self._toggle_map_mode)

        gp = self.ros_node.get_parameter
        view = gp('gui_view').value
        self._set_view(view if view in VIEWS else 'map')
        self._set_map_mode(gp('gui_map').value.lower() == '3d')

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._refresh_ui)
        self.ui_timer.start(33)

    # ── Construcción de la interfaz ──────────────────────────────────────────
    def _segmented(self, labels, on_change):
        """Botones exclusivos tipo selector. Devuelve (contenedor, botones)."""
        box = QWidget()
        lay = QHBoxLayout(box)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(4)
        group = QButtonGroup(box)
        group.setExclusive(True)
        buttons = []
        for i, text in enumerate(labels):
            b = QPushButton(text)
            b.setCheckable(True)
            b.setObjectName('flat')
            b.clicked.connect(lambda _=False, k=i: on_change(k))
            group.addButton(b)
            lay.addWidget(b)
            buttons.append(b)
        box._group = group
        return box, buttons

    def _build_ui(self):
        gp = self.ros_node.get_parameter
        arena = float(gp('arena_x_max').value)
        ceiling = float(gp('max_z').value)
        safe = float(gp('safe_zone_radius').value)

        root = QWidget()
        self.setCentralWidget(root)
        main = QVBoxLayout(root)
        main.setContentsMargins(10, 10, 10, 10)
        main.setSpacing(8)

        # Encabezado: título, selector de vista, estado
        header = QFrame()
        header.setObjectName('bar')
        hl = QHBoxLayout(header)
        hl.setContentsMargins(12, 6, 12, 6)
        hl.setSpacing(14)
        title = QLabel('Bebop 2')
        title.setObjectName('title')
        hl.addWidget(title)
        seg, self.view_buttons = self._segmented(
            ['Mapa', 'Cámara', 'Dividida'], lambda k: self._set_view(VIEWS[k]))
        hl.addWidget(seg)
        hl.addStretch()
        self.lbl_manual = QLabel('')
        hl.addWidget(self.lbl_manual)
        self.lbl_state = QLabel(STATE_TEXT[0])
        hl.addWidget(self.lbl_state)
        self.lbl_rate = QLabel('Odometría 0.0 Hz')
        self.lbl_rate.setObjectName('muted')
        hl.addWidget(self.lbl_rate)
        main.addWidget(header)

        # Centro: [vistas | panel lateral]
        root_split = QSplitter(Qt.Horizontal)
        main.addWidget(root_split, 1)
        self.view_split = QSplitter(Qt.Horizontal)
        root_split.addWidget(self.view_split)

        # Panel del mapa
        self.map_panel = QWidget()
        self.map_panel.setMinimumWidth(430)
        ml = QVBoxLayout(self.map_panel)
        ml.setContentsMargins(0, 0, 0, 0)
        ml.setSpacing(6)
        self.map2d = SpatialMapWidget(arena, safe)
        self.map3d = SpatialMap3DWidget(arena, ceiling, safe)
        self.map_stack = QStackedWidget()
        self.map_stack.addWidget(self.map2d)
        self.map_stack.addWidget(self.map3d)
        ml.addWidget(self.map_stack, 1)
        tools = QHBoxLayout()
        tools.setSpacing(6)
        seg2, self.map_buttons = self._segmented(['2D', '3D'], lambda k: self._set_map_mode(k == 1))
        tools.addWidget(seg2)
        self.btn_follow = QPushButton('Seguir')
        self.btn_follow.setToolTip('Centrar la vista en el dron')
        self.btn_follow.setCheckable(True)
        self.btn_follow.setObjectName('flat')
        self.btn_follow.toggled.connect(self._on_follow)
        tools.addWidget(self.btn_follow)
        btn_view = QPushButton('Restablecer')
        btn_view.setToolTip('Restablecer la vista (zoom, giro y encuadre)')
        btn_view.setObjectName('flat')
        btn_view.clicked.connect(lambda: self._active_map().reset_view())
        tools.addWidget(btn_view)
        btn_reset = QPushButton('Borrar traza')
        btn_reset.setToolTip('Reiniciar la trayectoria y el recorrido')
        btn_reset.setObjectName('flat')
        btn_reset.clicked.connect(self._reset_trajectory)
        tools.addWidget(btn_reset)
        btn_csv = QPushButton('CSV')
        btn_csv.setToolTip('Exportar la trayectoria a CSV')
        btn_csv.setObjectName('flat')
        btn_csv.clicked.connect(self._export_csv)
        tools.addWidget(btn_csv)
        tools.addStretch()
        ml.addLayout(tools)
        self.view_split.addWidget(self.map_panel)

        # Panel de la cámara
        self.cam_panel = QWidget()
        self.cam_panel.setMinimumWidth(430)
        cl = QVBoxLayout(self.cam_panel)
        cl.setContentsMargins(0, 0, 0, 0)
        cl.setSpacing(6)
        self.lbl_cam_view = QLabel('Esperando video (/bebop/camera/image_raw)')
        self.lbl_cam_view.setAlignment(Qt.AlignCenter)
        self.lbl_cam_view.setObjectName('muted')
        self.lbl_cam_view.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.lbl_cam_view.setMinimumSize(1, 1)
        self.lbl_cam_view.setStyleSheet(
            f"background-color: {THEME['plot']}; border: 1px solid {THEME['border']};"
            f"border-radius: 4px; color: {THEME['muted']};")
        cl.addWidget(self.lbl_cam_view, 1)
        self.lbl_cam_status = QLabel('')
        self.lbl_cam_status.setObjectName('muted')
        cl.addWidget(self.lbl_cam_status)

        yolo_row = QHBoxLayout()
        yolo_row.setSpacing(8)
        self.chk_yolo = QCheckBox('YOLO')
        self.chk_yolo.setChecked(True)
        self.chk_yolo.toggled.connect(lambda on: setattr(self, 'enable_yolo', on))
        yolo_row.addWidget(self.chk_yolo)
        self.combo_mode = QComboBox()
        self.combo_mode.addItems(['Personas', 'Objetos demo', 'Todo (COCO)'])
        self.combo_mode.setToolTip('Qué clases detecta YOLO')
        self.combo_mode.currentIndexChanged.connect(
            lambda i: self.yolo_detector.set_mode(YOLO_MODES[i]))
        yolo_row.addWidget(self.combo_mode)
        self.lbl_conf = QLabel('Conf. 25%')
        self.lbl_conf.setObjectName('muted')
        yolo_row.addWidget(self.lbl_conf)
        self.slider_conf = QSlider(Qt.Horizontal)
        self.slider_conf.setRange(10, 80)
        self.slider_conf.setValue(25)
        self.slider_conf.setMinimumWidth(60)
        self.slider_conf.valueChanged.connect(self._on_conf)
        yolo_row.addWidget(self.slider_conf, 1)
        btn_pop = QPushButton('Ventana')
        btn_pop.setToolTip('Abrir la cámara en una ventana aparte')
        btn_pop.setObjectName('flat')
        btn_pop.clicked.connect(self._popout_camera)
        yolo_row.addWidget(btn_pop)
        cl.addLayout(yolo_row)

        tilt_row = QHBoxLayout()
        tilt_row.setSpacing(8)
        self.lbl_tilt = QLabel('Inclinación +0°')
        self.lbl_tilt.setFixedWidth(110)
        tilt_row.addWidget(self.lbl_tilt)
        self.slider_tilt = QSlider(Qt.Horizontal)
        self.slider_tilt.setRange(-85, 17)
        self.slider_tilt.valueChanged.connect(self._on_tilt)
        tilt_row.addWidget(self.slider_tilt, 1)
        cl.addLayout(tilt_row)
        pan_row = QHBoxLayout()
        pan_row.setSpacing(8)
        self.lbl_pan = QLabel('Giro +0°')
        self.lbl_pan.setFixedWidth(110)
        pan_row.addWidget(self.lbl_pan)
        self.slider_pan = QSlider(Qt.Horizontal)
        self.slider_pan.setRange(-35, 35)
        self.slider_pan.valueChanged.connect(self._on_pan)
        pan_row.addWidget(self.slider_pan, 1)
        b = QPushButton('Centrar')
        b.setObjectName('flat')
        b.clicked.connect(lambda: self.slider_pan.setValue(0))
        pan_row.addWidget(b)
        cl.addLayout(pan_row)
        preset_row = QHBoxLayout()
        preset_row.setSpacing(6)
        for text, tilt in (('Frente', 0), ('Personas', 10), ('Mesa', -15), ('Suelo', -70)):
            b = QPushButton(text)
            b.setToolTip(f'Inclinación {tilt:+d}°')
            b.setObjectName('flat')
            b.clicked.connect(lambda _=False, t=tilt: self.slider_tilt.setValue(t))
            preset_row.addWidget(b)
        preset_row.addStretch()
        cl.addLayout(preset_row)
        self.view_split.addWidget(self.cam_panel)
        self.view_split.setStretchFactor(0, 1)
        self.view_split.setStretchFactor(1, 1)

        # Panel lateral
        side = QWidget()
        side.setMinimumWidth(280)
        side.setMaximumWidth(300)
        sl = QVBoxLayout(side)
        sl.setContentsMargins(8, 0, 0, 0)
        sl.setSpacing(8)
        grp = QGroupBox('Actitud')
        gl = QVBoxLayout(grp)
        gl.setContentsMargins(6, 6, 6, 6)
        self.pfd = AttitudeIndicatorWidget()
        gl.addWidget(self.pfd, 0, Qt.AlignCenter)
        sl.addWidget(grp)

        grp = QGroupBox('Telemetría')
        grid = QGridLayout(grp)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(4)
        self.tel_labels = {}
        for row, (key, name) in enumerate((
                ('x', 'X (frente)'), ('y', 'Y (lateral)'), ('z', 'Z (altura)'),
                ('yaw', 'Rumbo'), ('speed', 'Velocidad'), ('dist', 'Recorrido'))):
            lbl = QLabel(name)
            lbl.setObjectName('muted')
            val = QLabel('—')
            val.setFont(_mono(11))
            val.setAlignment(Qt.AlignRight)
            grid.addWidget(lbl, row, 0)
            grid.addWidget(val, row, 1)
            self.tel_labels[key] = val
        sl.addWidget(grp)

        grp = QGroupBox('Mando')
        gl = QVBoxLayout(grp)
        gl.setContentsMargins(6, 6, 6, 6)
        self.joy_widget = DualSenseWidget()
        gl.addWidget(self.joy_widget)
        sl.addWidget(grp)
        sl.addStretch()
        root_split.addWidget(side)
        root_split.setStretchFactor(0, 1)
        root_split.setStretchFactor(1, 0)

        # Pie: acciones
        footer = QFrame()
        footer.setObjectName('bar')
        fl = QHBoxLayout(footer)
        fl.setContentsMargins(10, 8, 10, 8)
        fl.setSpacing(10)
        btn_takeoff = QPushButton('Despegar (hover a ~1 m)')
        btn_takeoff.setObjectName('primary')
        btn_takeoff.clicked.connect(lambda: self._send_cmd('takeoff'))
        fl.addWidget(btn_takeoff)
        btn_land = QPushButton('Aterrizar')
        btn_land.setObjectName('warn')
        btn_land.clicked.connect(lambda: self._send_cmd('land'))
        fl.addWidget(btn_land)
        fl.addStretch()
        hint = QLabel('Mando: mantener L2 + stick izq. / cruceta para mover · soltar L2 = hover')
        hint.setObjectName('muted')
        fl.addWidget(hint)
        fl.addStretch()
        btn_emerg = QPushButton('Emergencia (corta motores)')
        btn_emerg.setObjectName('danger')
        btn_emerg.clicked.connect(lambda: self._send_cmd('emergency'))
        fl.addWidget(btn_emerg)
        main.addWidget(footer)

    # ── ROS ──────────────────────────────────────────────────────────────────
    def _init_ros_io(self):
        n = self.ros_node
        n.create_subscription(Pose, '/bebop1/pose', self._pose_cb, 10)
        n.create_subscription(Odometry, '/odom_local', self._odom_local_cb, 10)
        n.create_subscription(Odometry, '/bebop/odom', self._bebop_odom_cb, 10)
        # El driver publica en /bebop/camera/image_raw; best-effort recibe de cualquier QoS
        for topic in ('/bebop/camera/image_raw', '/bebop1/camera/image_raw'):
            n.create_subscription(Image, topic, self._camera_cb, qos_profile_sensor_data)
        self.pub_yolo_img = n.create_publisher(Image, '/bebop/camera/image_yolo', 10)
        self.pub_move_cam = n.create_publisher(Vector3, '/bebop/move_camera', 10)
        n.create_subscription(Joy, '/joy', self._joy_cb, 10)
        n.create_subscription(Bool, '/deadman_active', self._deadman_cb, 10)
        n.create_subscription(Int32, '/demo/status', self._state_cb, 10)
        n.create_subscription(Bool, '/demo/manual_active', self._manual_cb, 10)
        self.pub_gui_cmd = n.create_publisher(String, '/demo/command', 10)

        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(n)
        self.ros_thread = threading.Thread(target=self.executor.spin, name='ros_spin',
                                           daemon=True)
        self.ros_thread.start()
        # Sostiene la orientación de la cámara (el Bebop la pierde si no se reenvía)
        n.create_timer(0.2, self._publish_camera_orientation)

    def _pose_cb(self, msg: Pose):
        q = msg.orientation
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        t = self.telemetry
        x, y, z = msg.position.x, msg.position.y, msg.position.z
        if self._prev_pos is not None:
            t['total_dist'] += math.hypot(x - self._prev_pos[0], y - self._prev_pos[1])
        self._prev_pos = (x, y)
        t['x'], t['y'], t['z'], t['yaw'] = x, y, z, math.degrees(yaw)
        self._last_pose_time = time.time()
        self._odom_count += 1
        self._pose_queue.append((x, y, z, t['vx'], t['vy'], t['vz'], yaw))

    def _odom_local_cb(self, msg: Odometry):
        v = msg.twist.twist.linear
        t = self.telemetry
        t['vx'], t['vy'], t['vz'], t['speed'] = v.x, v.y, v.z, math.hypot(v.x, v.y)

    def _bebop_odom_cb(self, msg: Odometry):
        # El bridge publica /bebop1/pose solo con rumbo: roll y pitch salen de la odometría cruda
        q = msg.pose.pose.orientation
        r, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        t = self.telemetry
        t['roll'], t['pitch'] = math.degrees(r), math.degrees(pitch)
        if time.time() - self._last_pose_time > 1.0:    # sin bridge: pose directa de la odometría
            p = msg.pose.pose.position
            t['x'], t['y'], t['z'], t['yaw'] = p.x, p.y, p.z, math.degrees(yaw)
            v = msg.twist.twist.linear
            t['vx'], t['vy'], t['vz'], t['speed'] = v.x, v.y, v.z, math.hypot(v.x, v.y)

    def _camera_cb(self, msg: Image):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp != (0, 0) and stamp == self._last_img_stamp:
            return                                  # el mismo frame por dos tópicos
        self._last_img_stamp = stamp
        now = time.time()
        self.last_cam_frame_time = now
        self.cam_frames_count += 1
        self.cam_total_frames += 1
        if now - self._last_cam_fps_time >= 1.0:
            self.cam_stream_fps = self.cam_frames_count / (now - self._last_cam_fps_time)
            self.cam_frames_count = 0
            self._last_cam_fps_time = now
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            print(f'[GUI] Error convirtiendo la imagen: {e}')
            return
        with self._frame_lock:
            self._raw_frame = (img, msg.header)
        self._frame_event.set()

    def _yolo_loop(self):
        """Hilo trabajador: toma siempre el último frame y descarta los intermedios."""
        while not self._stop.is_set():
            if not self._frame_event.wait(0.5):
                continue
            self._frame_event.clear()
            with self._frame_lock:
                item, self._raw_frame = self._raw_frame, None
            if item is None:
                continue
            img, header = item
            try:
                out = self.yolo_detector.detect(img)[0] if self.enable_yolo else img
                self.latest_frame = out
                self.new_frame_available = True
                msg = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
                msg.header = header
                self.pub_yolo_img.publish(msg)
            except Exception as e:
                if not self._stop.is_set():
                    print(f'[GUI] Error en el hilo de YOLO: {e}')

    def _joy_cb(self, msg: Joy):
        self._joy = (list(msg.buttons), list(msg.axes))

    def _deadman_cb(self, msg: Bool):
        self._deadman = msg.data

    def _state_cb(self, msg: Int32):
        self._demo_state = msg.data

    def _manual_cb(self, msg: Bool):
        self._manual_active = msg.data

    def _publish_camera_orientation(self):
        msg = Vector3()
        msg.x, msg.y, msg.z = float(self.cam_tilt), float(self.cam_pan), 0.0
        self.pub_move_cam.publish(msg)

    # ── Refresco de la interfaz (hilo de Qt) ─────────────────────────────────
    def _refresh_ui(self):
        now = time.time()
        if now - self._last_hz_calc >= 1.0:
            self.lbl_rate.setText(f'Odometría {self._odom_count / (now - self._last_hz_calc):.1f} Hz')
            self._odom_count = 0
            self._last_hz_calc = now

        t = self.telemetry
        while self._pose_queue:
            x, y, z, vx, vy, vz, yaw = self._pose_queue.popleft()
            self.map2d.add_point(x, y, z, vx, vy, vz, yaw)
            self.map3d.add_point(x, y, z, vx, vy, vz, yaw)
        self.map3d.set_attitude(t['roll'], t['pitch'])

        self.tel_labels['x'].setText(f"{t['x']:+.3f} m")
        self.tel_labels['y'].setText(f"{t['y']:+.3f} m")
        self.tel_labels['z'].setText(f"{t['z']:+.3f} m")
        self.tel_labels['yaw'].setText(f"{t['yaw']:+.1f}°")
        self.tel_labels['speed'].setText(f"{t['speed']:.2f} m/s")
        self.tel_labels['dist'].setText(f"{t['total_dist']:.2f} m")
        self.pfd.update_attitude(t['roll'], t['pitch'], t['yaw'])
        if self._joy is not None:
            self.joy_widget.update_joy(self._joy[0], self._joy[1], self._deadman)
        else:
            self.joy_widget.update_joy([], [], self._deadman)

        # Chips de estado (solo se reestilizan al cambiar)
        if self._demo_state != self._shown_state:
            self._shown_state = self._demo_state
            fg, bg = STATE_CHIP.get(self._demo_state, STATE_CHIP[0])
            self.lbl_state.setText(STATE_TEXT.get(self._demo_state, str(self._demo_state)))
            self.lbl_state.setStyleSheet(chip_style(fg, bg))
        manual = (self._demo_state == 1, self._manual_active)
        if manual != self._shown_manual:
            self._shown_manual = manual
            if not manual[0]:
                self.lbl_manual.setText('')
                self.lbl_manual.setStyleSheet('')
            elif manual[1]:
                self.lbl_manual.setText('Manual')
                self.lbl_manual.setStyleSheet(chip_style('accent', 'accent_dim'))
            else:
                self.lbl_manual.setText('Hover')
                self.lbl_manual.setStyleSheet(chip_style('muted', 'surface_alt'))

        self._refresh_camera(now)

    def _refresh_camera(self, now):
        if self.new_frame_available and self.latest_frame is not None:
            self.new_frame_available = False
            try:
                rgb = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                pixmap = QPixmap.fromImage(
                    QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy())
                self.lbl_cam_view.setPixmap(pixmap.scaled(
                    self.lbl_cam_view.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
                if self.floating_cam_window and self.floating_cam_window.isVisible():
                    self.floating_cam_window.set_pixmap(pixmap)
            except Exception as e:
                print(f'[GUI] Error dibujando la cámara: {e}')
        if now - self.last_cam_frame_time < 2.0:
            video = f'Video {self.cam_stream_fps:.0f} fps'
        else:
            video = 'Sin video'
        self.lbl_cam_status.setText(f'{video}   ·   {self.yolo_detector.status_text()}')

    # ── Acciones ─────────────────────────────────────────────────────────────
    def _set_view(self, view):
        k = VIEWS.index(view)
        self.view_buttons[k].setChecked(True)
        self.map_panel.setVisible(view in ('map', 'split'))
        self.cam_panel.setVisible(view in ('camera', 'split'))
        if view == 'split':
            half = max(1, self.view_split.width() // 2)
            self.view_split.setSizes([half, half])

    def _active_map(self):
        return self.map3d if self.map_stack.currentWidget() is self.map3d else self.map2d

    def _set_map_mode(self, is_3d):
        self.map_buttons[1 if is_3d else 0].setChecked(True)
        self.map_stack.setCurrentWidget(self.map3d if is_3d else self.map2d)
        self._active_map().set_follow(self.btn_follow.isChecked())

    def _toggle_map_mode(self):
        self._set_map_mode(self._active_map() is self.map2d)

    def _on_follow(self, on):
        self.map2d.set_follow(on)
        self.map3d.set_follow(on)

    def _on_tilt(self, val):
        self.cam_tilt = float(val)
        self.lbl_tilt.setText(f'Inclinación {val:+d}°')
        self._publish_camera_orientation()

    def _on_pan(self, val):
        self.cam_pan = float(val)
        self.lbl_pan.setText(f'Giro {val:+d}°')
        self._publish_camera_orientation()

    def _on_conf(self, val):
        self.lbl_conf.setText(f'Conf. {val}%')
        self.yolo_detector.set_conf_threshold(val / 100.0)

    def _send_cmd(self, text):
        self.pub_gui_cmd.publish(String(data=text))

    def _reset_trajectory(self):
        self.map2d.reset_map(self.telemetry['x'], self.telemetry['y'])
        self.map3d.reset_map(self.telemetry['x'], self.telemetry['y'])
        self.telemetry['total_dist'] = 0.0

    def _popout_camera(self):
        if not self.floating_cam_window:
            self.floating_cam_window = FloatingCameraWindow()
            self.floating_cam_window.setStyleSheet(STYLESHEET)
        self.floating_cam_window.show()
        self.floating_cam_window.raise_()

    def _export_csv(self):
        path, _ = QFileDialog.getSaveFileName(
            self, 'Guardar trayectoria',
            os.path.join(data_dir('trayectorias'),
                         f"trayectoria_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"),
            'Archivos CSV (*.csv)')
        if not path:
            return
        try:
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Index', 'X_m', 'Y_m', 'Z_m', 'Speed_m_s', 'Yaw_rad'])
                for i, q in enumerate(self.map2d.points):
                    writer.writerow([i] + [round(v, 4) for v in q[:5]])
            QtWidgets.QMessageBox.information(self, 'Exportación', f'Datos guardados en:\n{path}')
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, 'Error al exportar', f'No se pudo guardar:\n{e}')

    # ── Cierre ───────────────────────────────────────────────────────────────
    def shutdown(self):
        if self._stop.is_set():
            return
        self._stop.set()
        self._frame_event.set()
        self.ui_timer.stop()
        if self.floating_cam_window:
            self.floating_cam_window.close()
        try:
            self.executor.shutdown()
            self.ros_node.destroy_node()
        except Exception:
            pass
        self.ros_thread.join(timeout=1.0)
        self._yolo_thread.join(timeout=1.0)

    def closeEvent(self, event):
        self.shutdown()
        event.accept()


def _make_node():
    node = Node('muestra_odometria_gui_node')
    for name, default in (('gui_view', 'map'), ('gui_map', '2d'), ('arena_x_max', 2.0),
                          ('max_z', 2.0), ('safe_zone_radius', 0.5)):
        node.declare_parameter(name, default)
    return node


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(remove_ros_args(args=sys.argv))
    node = _make_node()
    gui = MuestraOdometriaGUI(node)
    gui.show()

    # Python solo procesa señales si el intérprete corre de vez en cuando: el QTimer lo garantiza
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    keepalive = QTimer()
    keepalive.timeout.connect(lambda: None)
    keepalive.start(200)

    code = app.exec_()
    gui.shutdown()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
