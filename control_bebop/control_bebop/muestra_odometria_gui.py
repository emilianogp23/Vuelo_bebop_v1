#!/usr/bin/env python3
"""
Suite de Visualización Profesional de Odometría, Mapeo Espacial y Detección YOLO
para Parrot Bebop 2 (ROS 2 Jazzy).

Características:
1. Mapeo Espacial 2D/3D con estela de trayectoria, gradiente de color por velocidad,
   cuadrícula métrica, orientación en tiempo real y marcador de origen.
2. Horizonte Artificial PFD (Primary Flight Display) giroscópico y compás de rumbo 360°.
3. Cámara en vivo con detección YOLOv8 exclusiva para personas (bounding box táctico,
   confianza y cálculo de distancia estimada con modelo pinhole). Soporte para ventana flotante.
4. Telemetría de alta resolución (X, Y, Z, Roll, Pitch, Yaw, Vx, Vy, Vz, Hz, odómetro).
5. Monitor visual de mando PS5 DualSense.
6. Panel de control y seguridad para Modo Muestra Profesional.
"""

import sys
import os
import math
import time
import csv
from datetime import datetime
import threading

# Asegurar acceso a rob_env donde está instalado ultralytics y torch
for site_pkg in [
    '/home/emiliano/rob_env/lib/python3.12/site-packages',
    os.path.expanduser('~/rob_env/lib/python3.12/site-packages'),
]:
    if os.path.exists(site_pkg) and site_pkg not in sys.path:
        sys.path.insert(0, site_pkg)

import numpy as np
import cv2

# Solución al conflicto entre opencv-python y el sistema PyQt5 en Linux
if 'QT_QPA_PLATFORM_PLUGIN_PATH' in os.environ:
    os.environ.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins'

# PyQt5
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QPointF, QRectF
from PyQt5.QtGui import (
    QColor, QPainter, QPen, QBrush, QFont, QRadialGradient,
    QLinearGradient, QPolygonF, QImage, QPixmap
)
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QFrame, QSplitter, QCheckBox,
    QSlider, QFileDialog, QTabWidget, QGroupBox, QComboBox
)

# ROS 2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import Int32, String, Bool
from cv_bridge import CvBridge

# Detector YOLO local
from .yolo_person_detector import YoloPersonDetector


def euler_from_quaternion(x, y, z, w):
    """Calcula Roll, Pitch, Yaw (en radianes) desde un cuaternión."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = max(-1.0, min(+1.0, +2.0 * (w * y - z * x)))
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw


# ─────────────────────────────────────────────────────────────────────────────
#  WIDGET 1: MAPA ESPACIAL 2D RADAR CON TRAYECTORIA Y ORIENTACIÓN
# ─────────────────────────────────────────────────────────────────────────────
class SpatialMapWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 360)
        self.points = []  # Lista de tuplas: (x, y, z, vx, vy, vz, yaw)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.scale_px_per_m = 90.0  # Pixeles por metro
        self.pan_offset = QPointF(0, 0)
        self.last_mouse_pos = None
        self.auto_center = True
        self.origin_offset = (0.0, 0.0)  # Para resetear origen

        self.setMouseTracking(True)

    def add_point(self, x, y, z, vx, vy, vz, yaw):
        ox, oy = self.origin_offset
        adj_x = x - ox
        adj_y = y - oy
        self.current_x = adj_x
        self.current_y = adj_y
        self.current_z = z
        self.current_yaw = yaw

        # Añadir al historial si hay desplazamiento mínimo
        speed = math.hypot(vx, vy)
        if not self.points:
            self.points.append((adj_x, adj_y, z, speed, yaw))
        else:
            last = self.points[-1]
            dist = math.hypot(adj_x - last[0], adj_y - last[1])
            if dist > 0.015:  # Cada 1.5 cm
                self.points.append((adj_x, adj_y, z, speed, yaw))
                if len(self.points) > 4000:
                    self.points.pop(0)

        self.update()

    def reset_map(self, raw_x=None, raw_y=None):
        if raw_x is not None and raw_y is not None:
            self.origin_offset = (raw_x, raw_y)
        self.points.clear()
        self.current_x = 0.0
        self.current_y = 0.0
        self.pan_offset = QPointF(0, 0)
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.last_mouse_pos = event.pos()
            self.auto_center = False

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton and self.last_mouse_pos:
            delta = event.pos() - self.last_mouse_pos
            self.pan_offset += QPointF(delta.x(), delta.y())
            self.last_mouse_pos = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        self.last_mouse_pos = None

    def wheelEvent(self, event):
        angle = event.angleDelta().y()
        factor = 1.15 if angle > 0 else 0.85
        self.scale_px_per_m = max(15.0, min(600.0, self.scale_px_per_m * factor))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        cx = w / 2.0 + self.pan_offset.x()
        cy = h / 2.0 + self.pan_offset.y()

        # Si auto-center está activo
        if self.auto_center:
            cx = w / 2.0 - (self.current_y * self.scale_px_per_m)
            cy = h / 2.0 - (self.current_x * self.scale_px_per_m)

        # ── Fondo oscuro estilo radar táctico ──
        painter.fillRect(self.rect(), QColor("#0c1017"))

        # Gradiente radial suave en el centro
        grad = QRadialGradient(w/2, h/2, max(w, h)/1.2)
        grad.setColorAt(0.0, QColor(18, 28, 45, 180))
        grad.setColorAt(1.0, QColor(8, 12, 18, 255))
        painter.fillRect(self.rect(), QBrush(grad))

        # ── Cuadrícula métrica ──
        grid_m = 0.5  # Líneas cada 50 cm
        grid_px = grid_m * self.scale_px_per_m

        pen_grid = QPen(QColor(30, 48, 70, 110), 1, Qt.DashLine)
        painter.setPen(pen_grid)

        # Líneas verticales
        start_x = int(-cx / grid_px) - 2
        end_x = int((w - cx) / grid_px) + 2
        for i in range(start_x, end_x):
            gx = cx + i * grid_px
            painter.drawLine(int(gx), 0, int(gx), h)

        # Líneas horizontales
        start_y = int(-cy / grid_px) - 2
        end_y = int((h - cy) / grid_px) + 2
        for j in range(start_y, end_y):
            gy = cy + j * grid_px
            painter.drawLine(0, int(gy), w, int(gy))

        # ── Anillos concéntricos de distancia desde el origen ──
        for r_m in [0.5, 1.0, 1.5, 2.0, 3.0]:
            r_px = r_m * self.scale_px_per_m
            pen_ring = QPen(QColor(0, 180, 220, 40), 1, Qt.DotLine)
            painter.setPen(pen_ring)
            painter.drawEllipse(QPointF(cx, cy), r_px, r_px)

            # Etiqueta de distancia métrica
            painter.setFont(QFont("Monospace", 8))
            painter.setPen(QColor(0, 200, 255, 90))
            painter.drawText(int(cx + r_px + 3), int(cy - 3), f"{r_m:.1f}m")

        # ── Ejes cardinales (X: Frente, Y: Izquierda) ──
        painter.setPen(QPen(QColor(0, 229, 255, 140), 1.5))
        painter.drawLine(int(cx), 0, int(cx), h)  # Eje Y en pantalla (X frente dron)
        painter.drawLine(0, int(cy), w, int(cy))  # Eje X en pantalla (Y lateral dron)

        # ── Marcador de Origen [0, 0] ──
        painter.setPen(QPen(QColor(255, 82, 82), 2))
        painter.setBrush(QBrush(QColor(255, 82, 82, 100)))
        painter.drawEllipse(QPointF(cx, cy), 5, 5)
        painter.setFont(QFont("Ubuntu", 8, QFont.Bold))
        painter.setPen(QColor(255, 82, 82))
        painter.drawText(int(cx + 8), int(cy + 14), "ORIGEN [0,0]")

        # ── Trayectoria con gradiente de color por velocidad ──
        if len(self.points) > 1:
            for idx in range(len(self.points) - 1):
                p1 = self.points[idx]
                p2 = self.points[idx + 1]

                # Convención: X dron va hacia arriba en pantalla (-cy), Y dron hacia izquierda (-cx)
                x1_px = cx - p1[1] * self.scale_px_per_m
                y1_px = cy - p1[0] * self.scale_px_per_m
                x2_px = cx - p2[1] * self.scale_px_per_m
                y2_px = cy - p2[0] * self.scale_px_per_m

                spd = p2[3]  # m/s
                # Normalizar velocidad 0 a 0.8 m/s para mapa de calor
                val = min(1.0, spd / 0.6)
                r = int(255 * val)
                g = int(240 * (1.0 - abs(val - 0.5) * 2))
                b = int(255 * (1.0 - val))

                pen_trail = QPen(QColor(r, max(40, g), max(40, b), 210), 2.5)
                painter.setPen(pen_trail)
                painter.drawLine(QPointF(x1_px, y1_px), QPointF(x2_px, y2_px))

        # ── Posición actual del dron ──
        cur_px_x = cx - self.current_y * self.scale_px_per_m
        cur_px_y = cy - self.current_x * self.scale_px_per_m

        # Estela luminosa alrededor del dron
        glow_grad = QRadialGradient(cur_px_x, cur_px_y, 24)
        glow_grad.setColorAt(0.0, QColor(0, 230, 118, 140))
        glow_grad.setColorAt(1.0, QColor(0, 230, 118, 0))
        painter.fillRect(QRectF(cur_px_x - 24, cur_px_y - 24, 48, 48), QBrush(glow_grad))

        # Dibujo de silueta de dron Bebop (Cruz con rotores)
        painter.save()
        painter.translate(cur_px_x, cur_px_y)
        # Rotar según el ángulo Yaw (en pantalla arriba es -Y)
        painter.rotate(-math.degrees(self.current_yaw))

        # Brazos del dron
        painter.setPen(QPen(QColor(220, 230, 242), 2.5))
        d_arm = 14
        painter.drawLine(-d_arm, -d_arm, d_arm, d_arm)
        painter.drawLine(-d_arm, d_arm, d_arm, -d_arm)

        # 4 Hélices / Rotores
        painter.setPen(QPen(QColor(0, 229, 255), 1.5))
        painter.setBrush(QBrush(QColor(0, 229, 255, 60)))
        r_motor = 5
        for mx, my in [(-d_arm, -d_arm), (d_arm, -d_arm), (-d_arm, d_arm), (d_arm, d_arm)]:
            painter.drawEllipse(QPointF(mx, my), r_motor, r_motor)

        # Flecha de rumbo hacia el frente del dron
        painter.setPen(QPen(QColor(255, 214, 0), 2))
        painter.setBrush(QBrush(QColor(255, 214, 0)))
        arrow = QPolygonF([
            QPointF(0, -22),
            QPointF(-5, -12),
            QPointF(5, -12)
        ])
        painter.drawPolygon(arrow)
        painter.restore()

        # ── Overlay de información en esquina ──
        painter.setFont(QFont("Monospace", 9, QFont.Bold))
        painter.setPen(QColor(0, 229, 255))
        painter.drawText(12, 22, f"POS LOCAL: X={self.current_x:+.3f}m  Y={self.current_y:+.3f}m  Z={self.current_z:+.3f}m")
        painter.setPen(QColor(180, 200, 220))
        painter.drawText(12, 38, f"YAW: {math.degrees(self.current_yaw):+.1f}°  |  ESCALA: {self.scale_px_per_m:.0f} px/m")


# ─────────────────────────────────────────────────────────────────────────────
#  WIDGET 2: HORIZONTE ARTIFICIAL (PFD) Y COMPÁS DE RUMBO AERONÁUTICO
# ─────────────────────────────────────────────────────────────────────────────
class AttitudeIndicatorWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(220, 220)
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0

    def update_attitude(self, roll_deg, pitch_deg, yaw_deg):
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.yaw_deg = yaw_deg
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        r = min(w, h) / 2.0 - 10
        cx, cy = w / 2.0, h / 2.0

        # Máscara circular
        path = QtGui.QPainterPath()
        path.addEllipse(QPointF(cx, cy), r, r)
        painter.setClipPath(path)

        # ── Cielo y Tierra ──
        painter.save()
        painter.translate(cx, cy)
        painter.rotate(-self.roll_deg)

        # Desplazamiento de pitch (3 px por grado)
        pitch_offset = self.pitch_deg * 3.0

        # Cielo azul aeronáutico
        sky_rect = QRectF(-r*2, -r*2 + pitch_offset, r*4, r*2)
        painter.fillRect(sky_rect, QColor("#1e5fa0"))

        # Tierra marrón oscuro
        ground_rect = QRectF(-r*2, pitch_offset, r*4, r*2)
        painter.fillRect(ground_rect, QColor("#5a3c1c"))

        # Línea de horizonte blanca
        painter.setPen(QPen(Qt.white, 2))
        painter.drawLine(int(-r*1.5), int(pitch_offset), int(r*1.5), int(pitch_offset))

        # Escalera de pitch (marcas a ±10°, ±20°)
        painter.setPen(QPen(Qt.white, 1.5))
        painter.setFont(QFont("Monospace", 7))
        for deg in [-20, -10, 10, 20]:
            y_mark = pitch_offset - deg * 3.0
            w_mark = 26 if abs(deg) == 10 else 40
            painter.drawLine(int(-w_mark/2), int(y_mark), int(w_mark/2), int(y_mark))
            painter.drawText(int(w_mark/2 + 3), int(y_mark + 3), f"{abs(deg)}")
        painter.restore()

        # Quitar clip para instrumentos superpuestos
        painter.setClipping(False)

        # ── Borde exterior y escala de Roll ──
        painter.setPen(QPen(QColor(60, 80, 110), 3))
        painter.drawEllipse(QPointF(cx, cy), r, r)

        # Retícula central de la aeronave (Crosshair naranja fijo)
        painter.setPen(QPen(QColor(255, 214, 0), 3))
        painter.drawLine(int(cx - 30), int(cy), int(cx - 10), int(cy))
        painter.drawLine(int(cx + 10), int(cy), int(cx + 30), int(cy))
        painter.drawLine(int(cx), int(cy - 10), int(cx), int(cy - 2))
        painter.drawEllipse(QPointF(cx, cy), 3, 3)

        # ── Brújula / Compás de rumbo en la parte inferior ──
        painter.setFont(QFont("Ubuntu", 9, QFont.Bold))
        painter.setPen(QColor(255, 255, 255))
        head_txt = f"HDG {int((self.yaw_deg) % 360):03d}°"
        painter.drawText(int(cx - 35), int(cy + r - 12), head_txt)


# ─────────────────────────────────────────────────────────────────────────────
#  WIDGET 3: MONITOR VISUAL DEL MANDO PS5 DUALSENSE
# ─────────────────────────────────────────────────────────────────────────────
class DualSenseWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(220, 130)
        self.buttons = [0] * 16
        self.axes = [0.0] * 8
        self.deadman_active = False

    def update_joy(self, buttons, axes, deadman):
        self.buttons = buttons
        self.axes = axes
        self.deadman_active = deadman
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        painter.fillRect(self.rect(), QColor("#121722"))
        w, h = self.width(), self.height()

        # Marco con borde sutil
        painter.setPen(QPen(QColor(40, 56, 80), 1))
        painter.drawRoundedRect(QRectF(2, 2, w - 4, h - 4), 6, 6)

        painter.setFont(QFont("Ubuntu", 8, QFont.Bold))
        painter.setPen(QColor(160, 190, 220))
        painter.drawText(10, 16, "MANDO PS5 DUALSENSE")

        # Estado del Deadman (L2)
        l2_color = QColor(0, 230, 118) if self.deadman_active else QColor(100, 110, 125)
        painter.setPen(QPen(l2_color, 1.5))
        painter.setBrush(QBrush(l2_color if self.deadman_active else QColor(0, 0, 0, 0)))
        painter.drawRoundedRect(QRectF(10, 24, 75, 20), 4, 4)
        painter.setPen(QColor(0, 0, 0) if self.deadman_active else QColor(160, 180, 200))
        painter.drawText(16, 38, "DEADMAN L2")

        # Botones Principales (×, ○, △, □)
        btn_x = len(self.buttons) > 0 and self.buttons[0] == 1
        btn_circle = len(self.buttons) > 1 and self.buttons[1] == 1
        btn_tri = len(self.buttons) > 2 and self.buttons[2] == 1
        btn_sq = len(self.buttons) > 3 and self.buttons[3] == 1

        btn_info = [
            ("× (Armar)", btn_x, QColor(0, 230, 118), 10, 52),
            ("○ (Land)", btn_circle, QColor(255, 171, 0), 105, 52),
            ("△ (Emerg)", btn_tri, QColor(255, 82, 82), 10, 78),
            ("□ (Modo)", btn_sq, QColor(0, 229, 255), 105, 78),
        ]

        for label, active, col, bx, by in btn_info:
            painter.setPen(QPen(col if active else QColor(50, 65, 85), 1.5))
            painter.setBrush(QBrush(col if active else QColor(20, 28, 40)))
            painter.drawRoundedRect(QRectF(bx, by, 85, 20), 4, 4)
            painter.setPen(QColor(0, 0, 0) if active else col)
            painter.drawText(bx + 8, by + 14, label)

        # Sticks analógicos (indicador gráfico miniatura)
        if len(self.axes) >= 4:
            sx1 = 30 + self.axes[0] * 12
            sy1 = 114 - self.axes[1] * 10
            sx2 = 140 + self.axes[3] * 12
            sy2 = 114 - self.axes[4] * 10

            painter.setPen(QPen(QColor(60, 80, 100), 1))
            painter.setBrush(QBrush(QColor(15, 22, 32)))
            painter.drawEllipse(QPointF(30, 114), 12, 12)
            painter.drawEllipse(QPointF(140, 114), 12, 12)

            painter.setBrush(QBrush(QColor(0, 229, 255)))
            painter.drawEllipse(QPointF(sx1, sy1), 4, 4)
            painter.drawEllipse(QPointF(sx2, sy2), 4, 4)


# ─────────────────────────────────────────────────────────────────────────────
#  VENTANA POP-OUT PARA LA CÁMARA YOLO EN PANTALLA SECUNDARIA
# ─────────────────────────────────────────────────────────────────────────────
class FloatingCameraWindow(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Bebop 2 - Transmisión en Vivo con Detección YOLOv8")
        self.resize(800, 600)
        self.setStyleSheet("background-color: #0b0e14;")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        self.lbl_image = QLabel("Esperando señal de video...", self)
        self.lbl_image.setAlignment(Qt.AlignCenter)
        self.lbl_image.setStyleSheet("color: #708090; font-size: 16px; background-color: #05070a;")
        layout.addWidget(self.lbl_image)

    def set_pixmap(self, pixmap):
        scaled = pixmap.scaled(self.lbl_image.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.lbl_image.setPixmap(scaled)


# ─────────────────────────────────────────────────────────────────────────────
#  APLICACIÓN PRINCIPAL / MAIN WINDOW
# ─────────────────────────────────────────────────────────────────────────────
class MuestraOdometriaGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Parrot Bebop 2 — Suite Profesional de Odometría y Visión con IA")
        self.resize(1280, 820)
        self.setMinimumSize(1024, 680)

        # Estado interno de telemetría
        self.telemetry = {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'vx': 0.0, 'vy': 0.0, 'vz': 0.0,
            'speed': 0.0, 'odom_hz': 0.0, 'total_dist': 0.0, 'max_z': 0.0
        }
        self.prev_pos = None
        self.odom_count = 0
        self.last_hz_calc = time.time()
        self.demo_status = "IDLE (MOTORES APAGADOS)"
        self.bridge = CvBridge()

        # Detector YOLO
        self.yolo_detector = YoloPersonDetector(conf_threshold=0.25, mode='person')
        self.enable_yolo = True
        self.floating_cam_window = None
        self.latest_frame = None
        self.new_frame_available = False

        # Estado de orientación de cámara ojo de pez (/bebop/move_camera)
        self.cam_tilt = 0.0
        self.cam_pan = 0.0
        self.cam_frames_count = 0
        self.cam_total_frames = 0
        self.cam_stream_fps = 0.0
        self.last_cam_fps_time = time.time()
        self.last_cam_frame_time = 0.0
        self._last_img_stamp = None

        # Configuración visual moderna oscura
        self._setup_theme()
        self._build_ui()

        # Ros 2 Node & Thread
        self.ros_node = None
        self._init_ros()

        # Timer de refresco UI (30 Hz constante)
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._refresh_ui)
        self.ui_timer.start(33)

    def _setup_theme(self):
        self.setStyleSheet("""
            QMainWindow {
                background-color: #0e121a;
            }
            QWidget {
                color: #c9d1d9;
                font-family: 'Ubuntu', 'Segoe UI', sans-serif;
            }
            QGroupBox {
                border: 1px solid #253346;
                border-radius: 6px;
                margin-top: 10px;
                font-weight: bold;
                color: #00e5ff;
                padding-top: 10px;
                background-color: #131924;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 4px;
            }
            QPushButton {
                background-color: #1a2332;
                border: 1px solid #324765;
                color: #e6edf3;
                border-radius: 4px;
                padding: 7px 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #24344d;
                border-color: #00e5ff;
            }
            QPushButton:pressed {
                background-color: #00e5ff;
                color: #000000;
            }
            QTabWidget::pane {
                border: 1px solid #253346;
                background-color: #111622;
            }
            QTabBar::tab {
                background: #161e2b;
                color: #8b949e;
                padding: 8px 18px;
                border: 1px solid #253346;
                border-bottom: none;
                margin-right: 2px;
                font-weight: bold;
            }
            QTabBar::tab:selected {
                background: #1f2a3c;
                color: #00e5ff;
                border-color: #00e5ff;
            }
        """)

    def _build_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(8)

        # ── 1. Barra Superior / Header de Estado de Misión ──
        header = QFrame()
        header.setStyleSheet("background-color: #151c27; border-radius: 6px; padding: 4px;")
        h_layout = QHBoxLayout(header)
        h_layout.setContentsMargins(12, 4, 12, 4)

        title_lbl = QLabel("🛸 BEBOP 2 — SISTEMA DE TELEMETRÍA Y CONTROL DE ODOMETRÍA")
        title_lbl.setStyleSheet("font-size: 14px; font-weight: bold; color: #00e5ff;")
        h_layout.addWidget(title_lbl)

        h_layout.addStretch()

        self.lbl_demo_state = QLabel("ESTADO: MOTORES APAGADOS")
        self.lbl_demo_state.setStyleSheet(
            "font-size: 12px; font-weight: bold; color: #ff5252; background: #251216; padding: 4px 10px; border-radius: 4px;"
        )
        h_layout.addWidget(self.lbl_demo_state)

        self.lbl_rate = QLabel("FREQ: 0.0 Hz")
        self.lbl_rate.setStyleSheet("font-size: 11px; color: #a0aec0; margin-left: 10px;")
        h_layout.addWidget(self.lbl_rate)

        main_layout.addWidget(header)

        # ── 2. Área Central (Splitter: Izquierda Gráfica, Derecha Telemetría) ──
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter, 1)

        # ── Panel Izquierdo: Pestañas (Mapa Espacial / Cámara YOLO) ──
        self.tabs = QTabWidget()

        # Tab 1: Mapa Espacial y Navegación
        tab_map = QWidget()
        map_layout = QVBoxLayout(tab_map)
        map_layout.setContentsMargins(6, 6, 6, 6)

        self.map_widget = SpatialMapWidget()
        map_layout.addWidget(self.map_widget, 1)

        # Barra de herramientas del mapa
        map_tools = QHBoxLayout()
        btn_center = QPushButton("🎯 Auto-Centrar")
        btn_center.clicked.connect(lambda: setattr(self.map_widget, 'auto_center', True))
        map_tools.addWidget(btn_center)

        btn_reset_map = QPushButton("🔄 Reiniciar Trayectoria")
        btn_reset_map.clicked.connect(self._reset_trajectory)
        map_tools.addWidget(btn_reset_map)

        btn_export = QPushButton("💾 Exportar Datos CSV")
        btn_export.clicked.connect(self._export_csv)
        map_tools.addWidget(btn_export)

        map_tools.addStretch()
        map_layout.addLayout(map_tools)
        self.tabs.addTab(tab_map, "🗺️ Mapa Espacial 2D/3D")

        # Tab 2: Cámara en Vivo con Detección YOLO
        tab_cam = QWidget()
        cam_layout = QVBoxLayout(tab_cam)
        cam_layout.setContentsMargins(6, 6, 6, 6)

        self.lbl_cam_view = QLabel("Esperando flujo de video (/bebop/camera/image_raw)...")
        self.lbl_cam_view.setAlignment(Qt.AlignCenter)
        self.lbl_cam_view.setStyleSheet("background-color: #07090e; border: 1px solid #1a2332; border-radius: 4px;")
        cam_layout.addWidget(self.lbl_cam_view, 1)

        # Controles de visión
        cam_tools = QHBoxLayout()
        self.chk_yolo = QCheckBox("Activar YOLOv8")
        self.chk_yolo.setChecked(True)
        self.chk_yolo.setStyleSheet("font-weight: bold; color: #00e676;")
        self.chk_yolo.toggled.connect(self._toggle_yolo)
        cam_tools.addWidget(self.chk_yolo)

        lbl_mode = QLabel("Filtro:")
        lbl_mode.setStyleSheet("color: #ffd600; font-weight: bold;")
        cam_tools.addWidget(lbl_mode)

        self.combo_mode = QComboBox()
        self.combo_mode.addItems(["Solo Persona", "Modo Demo (Objetos)", "Todo (COCO)"])
        self.combo_mode.setStyleSheet("background-color: #1a2332; color: #ffffff; padding: 3px 8px; border: 1px solid #324765;")
        self.combo_mode.currentIndexChanged.connect(self._on_mode_combo_changed)
        cam_tools.addWidget(self.combo_mode)

        # Slider de umbral de confianza
        self.lbl_conf = QLabel("Confianza: 25%")
        self.lbl_conf.setStyleSheet("color: #00e5ff; font-weight: bold;")
        cam_tools.addWidget(self.lbl_conf)

        self.slider_conf = QSlider(Qt.Horizontal)
        self.slider_conf.setRange(10, 80)
        self.slider_conf.setValue(25)
        self.slider_conf.setFixedWidth(100)
        self.slider_conf.valueChanged.connect(self._on_conf_slider_change)
        cam_tools.addWidget(self.slider_conf)

        btn_popout = QPushButton("🖥️ Desacoplar Ventana")
        btn_popout.clicked.connect(self._popout_camera)
        cam_tools.addWidget(btn_popout)

        cam_tools.addStretch()
        cam_layout.addLayout(cam_tools)

        # ── Control de Gimbal / Cámara Ojo de Pez (/bebop/move_camera) ──
        gimbal_box = QGroupBox("🕹️ Control de Orientación Ojo de Pez (/bebop/move_camera)")
        gimbal_box.setStyleSheet("QGroupBox { font-weight: bold; color: #00e5ff; border: 1px solid #1f2f45; margin-top: 4px; padding-top: 8px; }")
        gimbal_layout = QVBoxLayout(gimbal_box)
        gimbal_layout.setContentsMargins(8, 6, 8, 6)
        gimbal_layout.setSpacing(4)

        # Fila 1: Tilt e Inclinación
        tilt_row = QHBoxLayout()
        self.lbl_tilt_val = QLabel("Inclinación (Tilt):  0.0°")
        self.lbl_tilt_val.setFixedWidth(150)
        self.lbl_tilt_val.setStyleSheet("color: #ffffff; font-weight: bold;")
        tilt_row.addWidget(self.lbl_tilt_val)

        self.slider_tilt = QSlider(Qt.Horizontal)
        self.slider_tilt.setRange(-85, 17)
        self.slider_tilt.setValue(0)
        self.slider_tilt.valueChanged.connect(self._on_tilt_slider_changed)
        tilt_row.addWidget(self.slider_tilt)

        btn_p_front = QPushButton("🎯 Frente (0°)")
        btn_p_front.clicked.connect(lambda: self._set_cam_preset(0, 0))
        tilt_row.addWidget(btn_p_front)

        btn_p_people = QPushButton("👤 Personas (+10°)")
        btn_p_people.clicked.connect(lambda: self._set_cam_preset(10, 0))
        tilt_row.addWidget(btn_p_people)

        btn_p_desk = QPushButton("📐 Mesa (-15°)")
        btn_p_desk.clicked.connect(lambda: self._set_cam_preset(-15, 0))
        tilt_row.addWidget(btn_p_desk)

        btn_p_floor = QPushButton("⬇️ Suelo (-70°)")
        btn_p_floor.clicked.connect(lambda: self._set_cam_preset(-70, 0))
        tilt_row.addWidget(btn_p_floor)

        gimbal_layout.addLayout(tilt_row)

        # Fila 2: Pan y Estado del Flujo
        pan_row = QHBoxLayout()
        self.lbl_pan_val = QLabel("Giro Lateral (Pan):  0.0°")
        self.lbl_pan_val.setFixedWidth(150)
        self.lbl_pan_val.setStyleSheet("color: #ffffff; font-weight: bold;")
        pan_row.addWidget(self.lbl_pan_val)

        self.slider_pan = QSlider(Qt.Horizontal)
        self.slider_pan.setRange(-35, 35)
        self.slider_pan.setValue(0)
        self.slider_pan.valueChanged.connect(self._on_pan_slider_changed)
        pan_row.addWidget(self.slider_pan)

        btn_p_pan0 = QPushButton("🔄 Centrar Pan")
        btn_p_pan0.clicked.connect(lambda: self.slider_pan.setValue(0))
        pan_row.addWidget(btn_p_pan0)

        self.lbl_cam_stream_stat = QLabel("Flujo: Esperando...")
        self.lbl_cam_stream_stat.setStyleSheet("color: #ffd600; font-size: 11px;")
        pan_row.addWidget(self.lbl_cam_stream_stat)

        gimbal_layout.addLayout(pan_row)
        cam_layout.addWidget(gimbal_box)

        self.tabs.addTab(tab_cam, "👁️ Cámara con IA (YOLOv8)")

        splitter.addWidget(self.tabs)

        # ── Panel Derecho: Telemetría, PFD y Mando PS5 ──
        right_panel = QWidget()
        right_panel.setMinimumWidth(340)
        right_panel.setMaximumWidth(420)
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(6, 0, 6, 0)
        right_layout.setSpacing(6)

        # Instrumento PFD (Horizonte Artificial)
        grp_pfd = QGroupBox("Horizonte Artificial & Brújula PFD")
        pfd_layout = QVBoxLayout(grp_pfd)
        pfd_layout.setContentsMargins(6, 6, 6, 6)
        self.pfd_widget = AttitudeIndicatorWidget()
        pfd_layout.addWidget(self.pfd_widget, 0, Qt.AlignCenter)
        right_layout.addWidget(grp_pfd)

        # Tarjetas de Telemetría Numérica
        grp_telemetry = QGroupBox("Telemetría en Tiempo Real")
        grid_tel = QGridLayout(grp_telemetry)
        grid_tel.setSpacing(6)

        self.lbl_val_x = self._create_telem_val("+0.000 m", "#00e676")
        self.lbl_val_y = self._create_telem_val("+0.000 m", "#00e676")
        self.lbl_val_z = self._create_telem_val("+0.000 m", "#00e5ff")

        grid_tel.addWidget(QLabel("X (Frente):"), 0, 0)
        grid_tel.addWidget(self.lbl_val_x, 0, 1)
        grid_tel.addWidget(QLabel("Y (Lateral):"), 1, 0)
        grid_tel.addWidget(self.lbl_val_y, 1, 1)
        grid_tel.addWidget(QLabel("Z (Altura):"), 2, 0)
        grid_tel.addWidget(self.lbl_val_z, 2, 1)

        self.lbl_val_yaw = self._create_telem_val("+0.0°", "#ffd600")
        self.lbl_val_speed = self._create_telem_val("0.00 m/s", "#ff79c6")
        self.lbl_val_dist = self._create_telem_val("0.00 m", "#ff9800")

        grid_tel.addWidget(QLabel("Rumbo (Yaw):"), 3, 0)
        grid_tel.addWidget(self.lbl_val_yaw, 3, 1)
        grid_tel.addWidget(QLabel("Vel. Lineal:"), 4, 0)
        grid_tel.addWidget(self.lbl_val_speed, 4, 1)
        grid_tel.addWidget(QLabel("Dist. Recorrida:"), 5, 0)
        grid_tel.addWidget(self.lbl_val_dist, 5, 1)

        right_layout.addWidget(grp_telemetry)

        # Monitor de Control PS5
        grp_joy = QGroupBox("Estado Mando PS5 DualSense")
        joy_layout = QVBoxLayout(grp_joy)
        joy_layout.setContentsMargins(6, 6, 6, 6)
        self.dualsense_widget = DualSenseWidget()
        joy_layout.addWidget(self.dualsense_widget)
        right_layout.addWidget(grp_joy)

        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        # ── 3. Barra Inferior de Acción y Seguridad ──
        footer = QFrame()
        footer.setStyleSheet("background-color: #151c27; border-radius: 6px; padding: 4px;")
        foot_layout = QHBoxLayout(footer)
        foot_layout.setContentsMargins(10, 6, 10, 6)
        foot_layout.setSpacing(10)

        self.btn_arm = QPushButton("🟢 ACTIVAR MOTORES (MODO MUESTRA)")
        self.btn_arm.setStyleSheet(
            "background-color: #0e3a24; border: 1px solid #00e676; color: #00e676; font-size: 12px; font-weight: bold; padding: 10px;"
        )
        self.btn_arm.clicked.connect(self._cmd_arm)
        foot_layout.addWidget(self.btn_arm)

        self.btn_disarm = QPushButton("🛬 APAGAR MOTORES (LAND)")
        self.btn_disarm.setStyleSheet(
            "background-color: #3d2f09; border: 1px solid #ffd600; color: #ffd600; font-size: 12px; font-weight: bold; padding: 10px;"
        )
        self.btn_disarm.clicked.connect(self._cmd_disarm)
        foot_layout.addWidget(self.btn_disarm)

        foot_layout.addStretch()

        self.btn_emergency = QPushButton("🚨 CORTE TOTAL DE EMERGENCIA")
        self.btn_emergency.setStyleSheet(
            "background-color: #4a1118; border: 2px solid #ff1744; color: #ffffff; font-size: 12px; font-weight: bold; padding: 10px 18px;"
        )
        self.btn_emergency.clicked.connect(self._cmd_emergency)
        foot_layout.addWidget(self.btn_emergency)

        main_layout.addWidget(footer)

    def _create_telem_val(self, default_text, color_hex):
        lbl = QLabel(default_text)
        lbl.setStyleSheet(f"font-family: 'Monospace'; font-size: 13px; font-weight: bold; color: {color_hex};")
        lbl.setAlignment(Qt.AlignRight)
        return lbl

    # ─────────────────────────────────────────────────────────────────────────
    #  COMUNICACIÓN ROS 2
    # ─────────────────────────────────────────────────────────────────────────
    def _init_ros(self):
        if not rclpy.ok():
            rclpy.init()

        self.ros_node = Node('muestra_odometria_gui_node')

        # Subscripciones de Odometría
        self.ros_node.create_subscription(Pose, '/bebop1/pose', self._pose_cb, 10)
        self.ros_node.create_subscription(Odometry, '/odom_local', self._odom_local_cb, 10)
        self.ros_node.create_subscription(Odometry, '/bebop/odom', self._bebop_odom_cb, 10)

        # Video cámara: QoS SensorData (BestEffort) y Fallback (Reliable)
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.ros_node.create_subscription(Image, '/bebop/camera/image_raw', self._camera_cb, cam_qos)
        self.ros_node.create_subscription(Image, '/bebop1/camera/image_raw', self._camera_cb, cam_qos)

        # Fallback Reliable para garantizar recepción con cualquier configuración
        cam_reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.ros_node.create_subscription(Image, '/bebop/camera/image_raw', self._camera_cb, cam_reliable_qos)

        # Publicador de video con Bounding Boxes para rqt_image_view u otros nodos
        self.pub_annotated_img = self.ros_node.create_publisher(Image, '/bebop/camera/image_annotated', 10)
        self.pub_yolo_img = self.ros_node.create_publisher(Image, '/bebop/camera/image_yolo', 10)

        # Publicador de Orientación de Cámara (Gimbal Ojo de Pez)
        self.pub_move_cam = self.ros_node.create_publisher(Vector3, '/bebop/move_camera', 10)

        # Control PS5 y Seguridad
        self.ros_node.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.ros_node.create_subscription(Bool, '/deadman_active', self._deadman_cb, 10)
        self.ros_node.create_subscription(String, '/demo/status_text', self._status_cb, 10)

        # Publisher de Comandos de GUI
        self.pub_gui_cmd = self.ros_node.create_publisher(String, '/demo/command', 10)

        # Executor dedicado en hilo secundario para aislar el spin
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.ros_node)
        self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_thread.start()

        # Timer continuo para SOSTENER la orientación de la cámara de forma permanente (5 Hz)
        self.cam_sustain_timer = self.ros_node.create_timer(0.2, self._publish_camera_orientation)

    def _pose_cb(self, msg: Pose):
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        r, p, yaw = euler_from_quaternion(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        )

        self.telemetry['x'] = x
        self.telemetry['y'] = y
        self.telemetry['z'] = z
        self.telemetry['roll'] = math.degrees(r)
        self.telemetry['pitch'] = math.degrees(p)
        self.telemetry['yaw'] = math.degrees(yaw)

        # Odómetro acumulado
        if self.prev_pos is not None:
            dx = x - self.prev_pos[0]
            dy = y - self.prev_pos[1]
            dist_step = math.hypot(dx, dy)
            self.telemetry['total_dist'] += dist_step
        self.prev_pos = (x, y)

        if z > self.telemetry['max_z']:
            self.telemetry['max_z'] = z

        self.odom_count += 1

        # Enviar al widget del mapa
        self.map_widget.add_point(
            x, y, z, self.telemetry['vx'], self.telemetry['vy'], self.telemetry['vz'], yaw
        )

    def _odom_local_cb(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        self.telemetry['vx'] = vx
        self.telemetry['vy'] = vy
        self.telemetry['vz'] = vz
        self.telemetry['speed'] = math.hypot(vx, vy)

    def _bebop_odom_cb(self, msg: Odometry):
        # Si /bebop1/pose no está activo, usamos /bebop/odom directamente
        if self.odom_count == 0:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            r, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
            self.telemetry['x'] = p.x
            self.telemetry['y'] = p.y
            self.telemetry['z'] = p.z
            self.telemetry['yaw'] = math.degrees(yaw)
            self.telemetry['pitch'] = math.degrees(pitch)
            self.telemetry['roll'] = math.degrees(r)
            self.telemetry['vx'] = msg.twist.twist.linear.x
            self.telemetry['vy'] = msg.twist.twist.linear.y
            self.telemetry['vz'] = msg.twist.twist.linear.z
            self.telemetry['speed'] = math.hypot(p.x, p.y)

    def _camera_cb(self, msg: Image):
        # Evitar duplicados por suscripciones QoS
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp != (0, 0) and stamp == self._last_img_stamp:
            return
        self._last_img_stamp = stamp

        now = time.time()
        self.last_cam_frame_time = now
        self.cam_frames_count += 1
        self.cam_total_frames += 1

        dt = now - self.last_cam_fps_time
        if dt >= 1.0:
            self.cam_stream_fps = self.cam_frames_count / dt
            self.cam_frames_count = 0
            self.last_cam_fps_time = now

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if cv_img is None:
                return

            # Inferencia YOLO de personas y objetos según configuración
            if self.enable_yolo and self.yolo_detector:
                annotated_img, _ = self.yolo_detector.detect(cv_img)
            else:
                annotated_img = cv_img

            # Publicar a tópicos ROS 2 para que rqt_image_view pueda visualizarlo
            try:
                annot_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
                annot_msg.header = msg.header
                self.pub_annotated_img.publish(annot_msg)
                self.pub_yolo_img.publish(annot_msg)
            except Exception as e_pub:
                pass

            # Guardar para el hilo principal de Qt (100% thread-safe)
            self.latest_frame = annotated_img
            self.new_frame_available = True

        except Exception as e:
            print(f"[MuestraOdometriaGUI _camera_cb Error]: {e}")

    def _joy_cb(self, msg: Joy):
        deadman = getattr(self, '_last_deadman', False)
        self.dualsense_widget.update_joy(msg.buttons, msg.axes, deadman)

        # Control de Orientación de Cámara con Cruceta / D-Pad PS5
        dpad_x = msg.axes[6] if len(msg.axes) > 6 else 0.0
        dpad_y = msg.axes[7] if len(msg.axes) > 7 else 0.0

        dpad_changed = False
        if dpad_y > 0.5 and getattr(self, '_prev_dpad_y', 0.0) <= 0.5:
            self.cam_tilt = min(17.0, self.cam_tilt + 5.0)
            dpad_changed = True
        elif dpad_y < -0.5 and getattr(self, '_prev_dpad_y', 0.0) >= -0.5:
            self.cam_tilt = max(-85.0, self.cam_tilt - 5.0)
            dpad_changed = True

        if dpad_x > 0.5 and getattr(self, '_prev_dpad_x', 0.0) <= 0.5:
            self.cam_pan = min(35.0, self.cam_pan + 5.0)
            dpad_changed = True
        elif dpad_x < -0.5 and getattr(self, '_prev_dpad_x', 0.0) >= -0.5:
            self.cam_pan = max(-35.0, self.cam_pan - 5.0)
            dpad_changed = True

        self._prev_dpad_x = dpad_x
        self._prev_dpad_y = dpad_y

        if dpad_changed:
            self.slider_tilt.blockSignals(True)
            self.slider_tilt.setValue(int(self.cam_tilt))
            self.slider_tilt.blockSignals(False)
            self.slider_pan.blockSignals(True)
            self.slider_pan.setValue(int(self.cam_pan))
            self.slider_pan.blockSignals(False)
            self.lbl_tilt_val.setText(f"Inclinación (Tilt): {self.cam_tilt:+.1f}°")
            self.lbl_pan_val.setText(f"Giro Lateral (Pan): {self.cam_pan:+.1f}°")

    def _deadman_cb(self, msg: Bool):
        self._last_deadman = msg.data

    def _status_cb(self, msg: String):
        self.demo_status = msg.data

    # ─────────────────────────────────────────────────────────────────────────
    #  ACTUALIZACIÓN CÍCLICA DE INTERFAZ (UI LOOP)
    # ─────────────────────────────────────────────────────────────────────────
    def _refresh_ui(self):
        # 1. Cálculo de frecuencia de odometría
        now = time.time()
        dt = now - self.last_hz_calc
        if dt >= 1.0:
            hz = self.odom_count / dt
            self.lbl_rate.setText(f"ODOM: {hz:.1f} Hz")
            self.odom_count = 0
            self.last_hz_calc = now

        # 2. Actualizar etiquetas de telemetría
        t = self.telemetry
        self.lbl_val_x.setText(f"{t['x']:+.3f} m")
        self.lbl_val_y.setText(f"{t['y']:+.3f} m")
        self.lbl_val_z.setText(f"{t['z']:+.3f} m")
        self.lbl_val_yaw.setText(f"{t['yaw']:+.1f}°")
        self.lbl_val_speed.setText(f"{t['speed']:.2f} m/s")
        self.lbl_val_dist.setText(f"{t['total_dist']:.2f} m")

        # 3. Estado de la misión
        self.lbl_demo_state.setText(self.demo_status)
        if "ACTIVO" in self.demo_status:
            self.lbl_demo_state.setStyleSheet(
                "font-size: 12px; font-weight: bold; color: #00e676; background: #0e3a24; padding: 4px 10px; border-radius: 4px;"
            )
        elif "EMERGENCIA" in self.demo_status:
            self.lbl_demo_state.setStyleSheet(
                "font-size: 12px; font-weight: bold; color: #ff1744; background: #5c0f16; padding: 4px 10px; border-radius: 4px;"
            )
        else:
            self.lbl_demo_state.setStyleSheet(
                "font-size: 12px; font-weight: bold; color: #ffd600; background: #3d2f09; padding: 4px 10px; border-radius: 4px;"
            )

        # 4. Actualizar PFD
        self.pfd_widget.update_attitude(t['roll'], t['pitch'], t['yaw'])

        # 5. Renderizar cámara en vivo en el hilo principal de Qt (100% thread-safe)
        if self.new_frame_available and self.latest_frame is not None:
            self.new_frame_available = False
            try:
                rgb_img = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_img.shape
                bytes_per_line = ch * w
                q_img = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888).copy()
                pixmap = QPixmap.fromImage(q_img)

                scaled = pixmap.scaled(
                    self.lbl_cam_view.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
                )
                self.lbl_cam_view.setPixmap(scaled)

                if self.floating_cam_window and self.floating_cam_window.isVisible():
                    self.floating_cam_window.set_pixmap(pixmap)
            except Exception as e:
                print(f"[UI Camera Render Error]: {e}")

        # 6. Actualizar indicador de señal de cámara ojo de pez
        if now - self.last_cam_frame_time < 2.0:
            self.lbl_cam_stream_stat.setText(
                f"🟢 ACTIVO: {self.cam_stream_fps:.1f} FPS | Total: {self.cam_total_frames} frames"
            )
            self.lbl_cam_stream_stat.setStyleSheet("color: #00e676; font-weight: bold; font-size: 11px;")
        else:
            self.lbl_cam_stream_stat.setText(
                "⚠️ Sin flujo (/bebop/camera/image_raw) — Esperando frames"
            )
            self.lbl_cam_stream_stat.setStyleSheet("color: #ff5252; font-weight: bold; font-size: 11px;")

    # ─────────────────────────────────────────────────────────────────────────
    #  ACCIONES DE USUARIO
    # ─────────────────────────────────────────────────────────────────────────
    def _on_mode_combo_changed(self, index):
        modes = ['person', 'demo', 'all']
        if 0 <= index < len(modes) and self.yolo_detector:
            self.yolo_detector.set_mode(modes[index])

    def _on_tilt_slider_changed(self, val):
        self.cam_tilt = float(val)
        self.lbl_tilt_val.setText(f"Inclinación (Tilt): {self.cam_tilt:+.1f}°")
        self._publish_camera_orientation()

    def _on_pan_slider_changed(self, val):
        self.cam_pan = float(val)
        self.lbl_pan_val.setText(f"Giro Lateral (Pan): {self.cam_pan:+.1f}°")
        self._publish_camera_orientation()

    def _set_cam_preset(self, tilt, pan):
        self.slider_tilt.setValue(int(tilt))
        self.slider_pan.setValue(int(pan))
        self.cam_tilt = float(tilt)
        self.cam_pan = float(pan)
        self.lbl_tilt_val.setText(f"Inclinación (Tilt): {self.cam_tilt:+.1f}°")
        self.lbl_pan_val.setText(f"Giro Lateral (Pan): {self.cam_pan:+.1f}°")
        self._publish_camera_orientation()

    def _publish_camera_orientation(self):
        if hasattr(self, 'pub_move_cam') and self.pub_move_cam:
            msg = Vector3()
            msg.x = float(self.cam_tilt)
            msg.y = float(self.cam_pan)
            msg.z = 0.0
            self.pub_move_cam.publish(msg)

    def _cmd_arm(self):
        msg = String()
        msg.data = "takeoff"
        self.pub_gui_cmd.publish(msg)

    def _cmd_disarm(self):
        msg = String()
        msg.data = "land"
        self.pub_gui_cmd.publish(msg)

    def _cmd_emergency(self):
        msg = String()
        msg.data = "emergency"
        self.pub_gui_cmd.publish(msg)

    def _reset_trajectory(self):
        self.map_widget.reset_map(self.telemetry['x'], self.telemetry['y'])
        self.telemetry['total_dist'] = 0.0

    def _toggle_yolo(self, checked):
        self.enable_yolo = checked

    def _toggle_all_classes(self, checked):
        if self.yolo_detector:
            self.yolo_detector.detect_all_classes = checked

    def _on_conf_slider_change(self, val):
        self.lbl_conf.setText(f"Confianza: {val}%")
        if self.yolo_detector:
            self.yolo_detector.conf_threshold = val / 100.0

    def _popout_camera(self):
        if not self.floating_cam_window:
            self.floating_cam_window = FloatingCameraWindow()
        self.floating_cam_window.show()
        self.floating_cam_window.raise_()

    def _export_csv(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Guardar Trayectoria Odometría",
            os.path.expanduser(f"~/ros2_ws/src/control_bebop/trayectoria_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"),
            "Archivos CSV (*.csv)"
        )
        if not path:
            return

        try:
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Index', 'X_m', 'Y_m', 'Z_m', 'Speed_m_s', 'Yaw_rad'])
                for i, p in enumerate(self.map_widget.points):
                    writer.writerow([i, round(p[0], 4), round(p[1], 4), round(p[2], 4), round(p[3], 4), round(p[4], 4)])
            QtWidgets.QMessageBox.information(self, "Exportación Exitosa", f"Datos guardados en:\n{path}")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error al Exportar", f"No se pudo guardar el archivo:\n{e}")

    def closeEvent(self, event):
        if self.floating_cam_window:
            self.floating_cam_window.close()
        if hasattr(self, 'executor') and self.executor:
            self.executor.shutdown()
        if self.ros_node:
            self.ros_node.destroy_node()
        event.accept()


def main(args=None):
    app = QApplication(sys.argv)
    gui = MuestraOdometriaGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
