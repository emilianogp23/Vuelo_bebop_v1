"""
Ritmo de cuadros para grabar video a una frecuencia fija.

La cámara no publica exactamente a la frecuencia del video: el Bebop real
manda ~30 Hz con saltos y Gazebo baja de 15 Hz cuando la simulación corre
más lenta que el tiempo real. Si cada cuadro recibido se escribe una sola
vez, el AVI dura más o menos que el vuelo.

FramePacer decide cuántas veces escribir cada cuadro según su marca de
tiempo: lo repite si faltan cuadros y lo descarta si sobran. Así la duración
del video coincide con el tiempo de las marcas (tiempo simulado en Gazebo).

Este módulo no importa rclpy ni cv2, así que se prueba sin ROS.
"""

import math


class FramePacer:
    def __init__(self, fps, max_gap=1.0):
        """fps: frecuencia del video. max_gap: hueco (s) a partir del cual
        no se rellena con cuadros repetidos y se reancla el tiempo."""
        if fps <= 0.0:
            raise ValueError('fps debe ser positivo')
        self.fps = float(fps)
        self.max_gap = float(max_gap)
        self.reset()

    def reset(self):
        self.t0 = None
        self.last = None
        self.written = 0

    def copies(self, stamp):
        """Veces que hay que escribir el cuadro con marca `stamp` (s)."""
        if self.t0 is None:
            self.t0 = stamp
        elif stamp < self.last or stamp - self.last > self.max_gap:
            # Reloj reiniciado o pausa larga: seguir sin rellenar el hueco
            self.t0 = stamp - self.written / self.fps
        self.last = stamp
        target = int(math.floor((stamp - self.t0) * self.fps + 0.5)) + 1
        n = max(0, target - self.written)
        self.written += n
        return n
