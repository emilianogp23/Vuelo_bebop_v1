#!/usr/bin/env python3
"""
Módulo de Detección de Personas con YOLOv8 para Parrot Bebop 2.

Basado en la implementación de VeranoUG2026-Optimizador_Trayectoria/yolo_detector.py:
- Filtra EXCLUSIVAMENTE la clase 'person' (ID 0).
- Dibuja bounding boxes estilizados con etiquetas de confianza.
- Estima la distancia a la persona utilizando el modelo de cámara Pinhole:
      dist = (altura_real * altura_img) / (altura_bbox * 2 * tan(fov / 2))
- Mide FPS de inferencia en tiempo real.
"""

import sys
import os
import math
import time
import cv2
import numpy as np

# Asegurar que el entorno virtual 'rob_env' donde está instalado ultralytics y torch esté en sys.path
for site_pkg in [
    '/home/emiliano/rob_env/lib/python3.12/site-packages',
    os.path.expanduser('~/rob_env/lib/python3.12/site-packages'),
]:
    if os.path.exists(site_pkg) and site_pkg not in sys.path:
        sys.path.insert(0, site_pkg)

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False


# Alturas estimadas reales (metros) para el modelo Pinhole
OBJECT_HEIGHTS = {
    'person': 1.70,
    'bottle': 0.25,
    'cup': 0.12,
    'cell phone': 0.15,
    'laptop': 0.25,
    'chair': 0.85,
    'backpack': 0.45,
    'sports ball': 0.22,
    'book': 0.25,
    'remote': 0.18,
    'tv': 0.60,
    'keyboard': 0.15,
    'mouse': 0.10,
}

DEMO_CLASSES = set(OBJECT_HEIGHTS.keys())

CLASS_NAMES_ES = {
    'person': 'PERSONA',
    'bottle': 'BOTELLA',
    'cup': 'TAZA',
    'cell phone': 'CELULAR',
    'laptop': 'LAPTOP',
    'chair': 'SILLA',
    'backpack': 'MOCHILA',
    'sports ball': 'PELOTA',
    'book': 'LIBRO',
    'remote': 'CONTROL',
    'tv': 'PANTALLA',
    'keyboard': 'TECLADO',
    'mouse': 'RATON',
}


class YoloPersonDetector:
    def __init__(self, model_path=None, conf_threshold=0.25, fov=0.87, real_person_height=1.70, mode='person'):
        """
        Inicializa el detector YOLOv8 para personas y objetos.
        :param model_path: Ruta a yolov8n.pt o None para búsqueda automática.
        :param conf_threshold: Umbral de confianza mínimo [0.0 - 1.0].
        :param fov: Campo de visión vertical/horizontal de la cámara en radianes (~50° = 0.87 rad).
        :param real_person_height: Altura media estimada de una persona en metros (1.70 m).
        :param mode: 'person' (solo personas), 'demo' (personas + objetos cotidianos para testing), 'all' (todo).
        """
        self.conf_threshold = conf_threshold
        self.fov = fov
        self.real_person_height = real_person_height
        self.mode = mode  # 'person', 'demo', 'all'
        self.detect_all_classes = (mode == 'all')
        self.model = None
        self.fps = 0.0
        self._prev_time = time.time()

        if not ULTRALYTICS_AVAILABLE:
            print("[YoloPersonDetector] ADVERTENCIA: 'ultralytics' no está disponible en este entorno.")
            return

        # Búsqueda de pesos si no se proporciona ruta absoluta
        candidate_paths = []
        if model_path:
            candidate_paths.append(model_path)

        candidate_paths.extend([
            "/home/emiliano/ros2_ws/src/VeranoUG2026-Optimizador_Trayectoria/controllers/Controlador_principal/yolov8n.pt",
            "/home/emiliano/ros2_ws/src/VeranoUG2026-Optimizador_Trayectoria/yolov8n.pt",
            os.path.expanduser("~/ros2_ws/src/control_bebop/config/yolov8n.pt"),
            "yolov8n.pt",
        ])

        resolved_path = None
        for p in candidate_paths:
            if os.path.exists(p):
                resolved_path = os.path.abspath(p)
                break

        if resolved_path is None:
            resolved_path = "yolov8n.pt"  # Ultralytics intentará descargarlo si no existe

        print(f"[YoloPersonDetector] Cargando modelo YOLO desde: {resolved_path}")
        try:
            self.model = YOLO(resolved_path)
            print("[YoloPersonDetector] Modelo YOLOv8 cargado exitosamente.")
        except Exception as e:
            print(f"[YoloPersonDetector] Error al cargar modelo YOLO: {e}")
            self.model = None

    def set_mode(self, mode: str):
        """Cambia el modo de filtrado: 'person', 'demo', 'all'."""
        self.mode = mode
        self.detect_all_classes = (mode == 'all')

    def set_conf_threshold(self, threshold: float):
        """Ajusta el umbral mínimo de confianza."""
        self.conf_threshold = max(0.05, min(0.95, threshold))

    def detect(self, frame_bgr):
        """
        Ejecuta la inferencia sobre una imagen en formato BGR de OpenCV.
        :param frame_bgr: Imagen en formato numpy array BGR.
        :return: (frame_annotated, detections)
                 detections es una lista de diccionarios:
                 [{'class': str, 'bbox': (x1, y1, x2, y2), 'conf': float, 'dist_m': float, 'center': (cx, cy)}]
        """
        if frame_bgr is None:
            return frame_bgr, []
        if self.model is None:
            out_err = frame_bgr.copy()
            cv2.rectangle(out_err, (10, 10), (520, 50), (0, 0, 180), -1)
            cv2.putText(out_err, "⚠️ ERROR: YOLOv8 no pudo inicializarse", (15, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            return out_err, []

        now = time.time()
        dt = now - self._prev_time
        if dt > 0.001:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)
        self._prev_time = now

        img_h, img_w = frame_bgr.shape[:2]
        out_img = frame_bgr.copy()
        detections = []

        try:
            results = self.model(frame_bgr, conf=self.conf_threshold, verbose=False)
        except Exception as e:
            print(f"[YoloPersonDetector] Error en inferencia: {e}")
            return out_img, []

        tan_half_fov = math.tan(self.fov / 2.0)

        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                cls_id = int(box.cls[0].cpu().numpy())
                class_name = self.model.names.get(cls_id, "")

                # FILTRADO SEGÚN MODO
                if self.mode == 'person':
                    if class_name != "person" and cls_id != 0:
                        continue
                elif self.mode == 'demo':
                    if class_name not in DEMO_CLASSES:
                        continue
                # Si mode == 'all', acepta cualquier clase

                conf = float(box.conf[0].cpu().numpy())
                if conf < self.conf_threshold:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                bbox_h = max(1.0, float(y2 - y1))
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # Cálculo de distancia estimada con modelo pinhole
                real_h = OBJECT_HEIGHTS.get(class_name, self.real_person_height)
                dist_m = (real_h * img_h) / (bbox_h * 2.0 * tan_half_fov)

                detections.append({
                    'class': class_name,
                    'bbox': (x1, y1, x2, y2),
                    'conf': conf,
                    'dist_m': dist_m,
                    'center': (cx, cy)
                })

                # ── Dibujo HUD estilo profesional ──
                is_person = (class_name == "person" or cls_id == 0)
                if is_person:
                    color_box = (0, 240, 100)      # Verde Tech para persona
                    color_text_bg = (10, 30, 20)
                elif class_name in ('bottle', 'cup', 'cell phone'):
                    color_box = (0, 200, 255)      # Ámbar/Cian para objetos cercanos
                    color_text_bg = (30, 25, 10)
                else:
                    color_box = (255, 120, 0)      # Azul cobalto para otros
                    color_text_bg = (20, 15, 30)

                # Bounding Box principal
                cv2.rectangle(out_img, (x1, y1), (x2, y2), color_box, 2)

                # Esquinas reforzadas (estilo visor táctico)
                corner_len = min(20, int((x2 - x1) * 0.2), int((y2 - y1) * 0.2))
                if corner_len > 4:
                    cv2.line(out_img, (x1, y1), (x1 + corner_len, y1), (255, 255, 255), 3)
                    cv2.line(out_img, (x1, y1), (x1, y1 + corner_len), (255, 255, 255), 3)
                    cv2.line(out_img, (x2, y1), (x2 - corner_len, y1), (255, 255, 255), 3)
                    cv2.line(out_img, (x2, y1), (x2, y1 + corner_len), (255, 255, 255), 3)
                    cv2.line(out_img, (x1, y2), (x1 + corner_len, y2), (255, 255, 255), 3)
                    cv2.line(out_img, (x1, y2), (x1, y2 - corner_len), (255, 255, 255), 3)
                    cv2.line(out_img, (x2, y2), (x2 - corner_len, y2), (255, 255, 255), 3)
                    cv2.line(out_img, (x2, y2), (x2, y2 - corner_len), (255, 255, 255), 3)

                # Punto central del objetivo
                cv2.circle(out_img, (cx, cy), 4, (0, 0, 255), -1)
                cv2.circle(out_img, (cx, cy), 8, (0, 240, 100), 1)

                # Texto de identificación y telemetría
                tag_name = CLASS_NAMES_ES.get(class_name, class_name.upper())
                tag = f"{tag_name}  {conf*100:.0f}% | {dist_m:.2f} m"
                font = cv2.FONT_HERSHEY_DUPLEX
                font_scale = 0.55
                font_thick = 1
                (tw, th), _ = cv2.getTextSize(tag, font, font_scale, font_thick)

                # Fondo para legibilidad
                y_tag_bottom = max(y1, th + 8)
                cv2.rectangle(out_img, (x1, y_tag_bottom - th - 6), (x1 + tw + 10, y_tag_bottom), color_text_bg, -1)
                cv2.rectangle(out_img, (x1, y_tag_bottom - th - 6), (x1 + tw + 10, y_tag_bottom), color_box, 1)
                cv2.putText(out_img, tag, (x1 + 5, y_tag_bottom - 4), font, font_scale, (255, 255, 255), font_thick, cv2.LINE_AA)

        # ── Overlay de estado superior siempre visible con fondo oscuro ──
        count = len(detections)
        mode_str = "SOLO PERSONAS" if self.mode == 'person' else ("DEMO OBJETOS" if self.mode == 'demo' else "TODO COCO")
        status_txt = f"YOLOv8 [{mode_str}]: {count} DETECTADO(S) | FPS: {self.fps:.1f} | UMBRAL: {int(self.conf_threshold*100)}%"
        cv2.rectangle(out_img, (8, 6), (620, 36), (12, 18, 28), -1)
        border_col = (0, 255, 120) if count > 0 else (60, 80, 110)
        cv2.rectangle(out_img, (8, 6), (620, 36), border_col, 1)
        text_col = (0, 255, 120) if count > 0 else (0, 229, 255)
        cv2.putText(out_img, status_txt, (14, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.52, text_col, 2, cv2.LINE_AA)

        return out_img, detections

