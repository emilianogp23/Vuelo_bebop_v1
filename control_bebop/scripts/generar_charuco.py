#!/usr/bin/env python3
"""
Genera el PNG imprimible del tablero ChArUco para calibrar la cámara del Bebop.

    python3 scripts/generar_charuco.py [--salida charuco.png] [--cuadro 0.035] [--dpi 300]

Por defecto: 7 x 5 cuadros de 3.5 cm (24.5 x 17.5 cm, cabe en hoja carta
horizontal), marcadores de 2.6 cm, DICT_5X5_100. Imprimir al 100 % en papel
mate, pegar en una placa rígida y plana, y MEDIR el lado de un cuadro con
regla: ese valor va en square_length del nodo calibrar_camara.
"""
import argparse
import os
import sys

import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control_bebop.calibracion import Charuco  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--salida', default='charuco.png')
    ap.add_argument('--cuadros-x', type=int, default=7)
    ap.add_argument('--cuadros-y', type=int, default=5, help='impar (ver calibracion.py)')
    ap.add_argument('--cuadro', type=float, default=0.035, help='lado del cuadro en m')
    ap.add_argument('--marcador', type=float, default=0.026, help='lado del ArUco en m')
    ap.add_argument('--diccionario', default='DICT_5X5_100')
    ap.add_argument('--dpi', type=int, default=300)
    a = ap.parse_args()

    ch = Charuco(a.cuadros_x, a.cuadros_y, a.cuadro, a.marcador, a.diccionario)
    px = a.cuadro / 0.0254 * a.dpi            # pixeles por cuadro
    margin = int(round(0.01 / 0.0254 * a.dpi))  # 1 cm blanco alrededor
    img = ch.image(int(round(px * a.cuadros_x)) + 2 * margin,
                   int(round(px * a.cuadros_y)) + 2 * margin, margin)
    cv2.imwrite(a.salida, img)
    print(f'{a.salida}: {a.cuadros_x}x{a.cuadros_y} cuadros de {a.cuadro * 100:.1f} cm, '
          f'{a.diccionario}, {img.shape[1]}x{img.shape[0]} px a {a.dpi} dpi. '
          'Imprimir al 100 % y medir el cuadro.')
    print(f'ros2 run control_bebop calibrar_camara --ros-args -p squares_x:={a.cuadros_x} '
          f'-p squares_y:={a.cuadros_y} -p square_length:=<medido> '
          f'-p marker_length:={a.marcador} -p dictionary:={a.diccionario}')


if __name__ == '__main__':
    main()
