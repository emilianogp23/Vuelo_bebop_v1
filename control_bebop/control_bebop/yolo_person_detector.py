#!/usr/bin/env python3
"""
Módulo de detección con YOLOv8 para Parrot Bebop 2.

- Por defecto solo la clase 'person' (modos 'demo' y 'all' para pruebas).
- Estima la distancia con el modelo pinhole:
      dist = (altura_real * altura_img) / (altura_bbox * 2 * tan(fov / 2))
- Carga el modelo en un hilo y NUNCA intenta descargarlo: el Bebop crea su propio
  Wi-Fi sin internet, así que una descarga fallaría. Si faltan los pesos, `status`
  lo dice (config/yolov8n.pt del paquete).
- `status` / `device` / `fps` los lee la GUI para mostrar el estado de YOLO.
"""

import math
import os
import threading
import time

import cv2

from control_bebop.paths import add_user_venvs, config_file, workspace_dir

# Entorno virtual ~/rob_env (ultralytics, torch) de la versión de Python actual
add_user_venvs(('rob_env',))

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

# Los tipos de OpenCV (Hershey) solo dibujan ASCII: sin acentos ni eñes
CLASS_NAMES_ES = {
    'person': 'persona',
    'bottle': 'botella',
    'cup': 'taza',
    'cell phone': 'celular',
    'laptop': 'laptop',
    'chair': 'silla',
    'backpack': 'mochila',
    'sports ball': 'pelota',
    'book': 'libro',
    'remote': 'control',
    'tv': 'pantalla',
    'keyboard': 'teclado',
    'mouse': 'raton',
}

BOX_COLOR = (214, 168, 76)      # BGR: azul suave, un solo color para todas las clases
LABEL_BG = (38, 33, 29)
LABEL_FG = (240, 236, 232)

STATUS_LOADING = 'cargando'
STATUS_READY = 'listo'
STATUS_ERROR = 'error'


def find_weights(model_path=None):
    """Ruta a los pesos locales o None. No descarga nada."""
    candidates = []
    if model_path:
        candidates.append(model_path)
    pkg_model = config_file('yolov8n.pt')
    if pkg_model:
        candidates.append(pkg_model)
    ws = workspace_dir()
    if ws:
        opt = os.path.join(ws, 'src', 'VeranoUG2026-Optimizador_Trayectoria')
        candidates.extend([
            os.path.join(opt, 'controllers', 'Controlador_principal', 'yolov8n.pt'),
            os.path.join(opt, 'yolov8n.pt'),
        ])
    candidates.append('yolov8n.pt')
    return next((os.path.abspath(p) for p in candidates if os.path.isfile(p)), None)


class YoloPersonDetector:
    def __init__(self, model_path=None, conf_threshold=0.25, fov=0.87,
                 real_person_height=1.70, mode='person', autoload=True):
        """
        :param model_path: ruta a yolov8n.pt o None para búsqueda automática.
        :param conf_threshold: umbral de confianza mínimo [0.0 - 1.0].
        :param fov: campo de visión vertical de la cámara en radianes (~50° = 0.87 rad).
        :param real_person_height: altura media de una persona en metros.
        :param mode: 'person', 'demo' (personas + objetos cotidianos) o 'all'.
        :param autoload: carga el modelo en un hilo al construir.
        """
        self.conf_threshold = conf_threshold
        self.fov = fov
        self.real_person_height = real_person_height
        self.mode = mode
        self.detect_all_classes = (mode == 'all')
        self.model = None
        self.fps = 0.0
        self.device = 'cpu'
        self.imgsz = 320
        self._opts = {}
        self.status = STATUS_LOADING
        self.error = ''
        self._model_path = model_path
        self._lock = threading.Lock()

        if autoload:
            threading.Thread(target=self.load, name='yolo_loader', daemon=True).start()

    @property
    def ready(self):
        return self.model is not None

    def load(self):
        """Carga el modelo (se llama desde un hilo; también se puede llamar a mano)."""
        if not ULTRALYTICS_AVAILABLE:
            self._fail("'ultralytics' no está instalado en este entorno")
            return
        weights = find_weights(self._model_path)
        if weights is None:
            self._fail('no se encontró yolov8n.pt (copiarlo a control_bebop/config/)')
            return
        try:
            import torch
            use_cuda = torch.cuda.is_available()
            self.device = 'cuda:0' if use_cuda else 'cpu'
            self.imgsz = 640 if use_cuda else 320
            print(f'[YOLO] Cargando {weights} en {self.device}')
            model = YOLO(weights)
            # Calentamiento: la primera inferencia es lenta y bloquearía la primera imagen
            self._opts = {'imgsz': self.imgsz, 'device': self.device, 'verbose': False}
            if use_cuda:
                self._opts['half'] = True   # solo con GPU; ultralytics avisa si se pasa half=False
            model(_blank(self.imgsz), **self._opts)
            self.model = model
            self.status = STATUS_READY
            print('[YOLO] Modelo listo')
        except Exception as e:  # pesos corruptos, torch sin soporte, etc.
            self._fail(str(e))

    def _fail(self, msg):
        self.model = None
        self.status = STATUS_ERROR
        self.error = msg
        print(f'[YOLO] ERROR: {msg}')

    def status_text(self):
        if self.status == STATUS_READY:
            return f'YOLO listo | {self.device} | {self.fps:.0f} fps'
        if self.status == STATUS_LOADING:
            return 'YOLO cargando...'
        return f'YOLO sin modelo: {self.error}'

    def set_mode(self, mode: str):
        """Cambia el modo de filtrado: 'person', 'demo', 'all'."""
        self.mode = mode
        self.detect_all_classes = (mode == 'all')

    def set_conf_threshold(self, threshold: float):
        self.conf_threshold = max(0.05, min(0.95, threshold))

    def detect(self, frame_bgr):
        """
        Inferencia sobre una imagen BGR de OpenCV.
        :return: (frame_anotado, detecciones). Sin modelo listo devuelve el frame tal cual.
                 detecciones: [{'class', 'bbox', 'conf', 'dist_m', 'center'}]
        """
        if frame_bgr is None or self.model is None:
            return frame_bgr, []

        img_h = frame_bgr.shape[0]
        out_img = frame_bgr.copy()
        detections = []

        t0 = time.time()
        try:
            results = self.model(frame_bgr, conf=self.conf_threshold, **self._opts)
        except Exception as e:
            print(f'[YOLO] Error en inferencia: {e}')
            return out_img, []
        dt = time.time() - t0
        if dt > 0.001:
            self.fps = 0.8 * self.fps + 0.2 * (1.0 / dt) if self.fps else 1.0 / dt

        tan_half_fov = math.tan(self.fov / 2.0)

        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                cls_id = int(box.cls[0].cpu().numpy())
                class_name = self.model.names.get(cls_id, '')

                if self.mode == 'person':
                    if class_name != 'person' and cls_id != 0:
                        continue
                elif self.mode == 'demo':
                    if class_name not in DEMO_CLASSES:
                        continue

                conf = float(box.conf[0].cpu().numpy())
                if conf < self.conf_threshold:
                    continue

                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].cpu().numpy())
                bbox_h = max(1.0, float(y2 - y1))
                real_h = OBJECT_HEIGHTS.get(class_name, self.real_person_height)
                dist_m = (real_h * img_h) / (bbox_h * 2.0 * tan_half_fov)

                detections.append({
                    'class': class_name,
                    'bbox': (x1, y1, x2, y2),
                    'conf': conf,
                    'dist_m': dist_m,
                    'center': (int((x1 + x2) / 2), int((y1 + y2) / 2)),
                })

                cv2.rectangle(out_img, (x1, y1), (x2, y2), BOX_COLOR, 2, cv2.LINE_AA)
                tag = (f'{CLASS_NAMES_ES.get(class_name, class_name)} '
                       f'{conf * 100:.0f}% | {dist_m:.1f} m')
                (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                top = max(y1, th + 10)
                cv2.rectangle(out_img, (x1, top - th - 8), (x1 + tw + 10, top), LABEL_BG, -1)
                cv2.putText(out_img, tag, (x1 + 5, top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            LABEL_FG, 1, cv2.LINE_AA)

        return out_img, detections


def _blank(size):
    import numpy as np
    return np.zeros((size, size, 3), dtype=np.uint8)
