#!/usr/bin/env python3
"""
Genera PNG imprimibles de los ArUco de config/aruco_map.yaml.

    python3 scripts/generar_marcadores.py [--mapa config/aruco_map.yaml] [--salida marcadores] [--dpi 300]

Cada PNG tiene el cuadro negro del tamaño del mapa, margen blanco de un
módulo y el id escrito abajo. Imprimir al 100 % (sin "ajustar a página") y
medir el cuadro negro con regla antes de pegarlo; en papel mate.
"""
import argparse
import os

import cv2
import numpy as np
import yaml


def marker_png(dictionary, marker_id, size_m, dpi):
    px = int(round(size_m / 0.0254 * dpi))
    draw = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker  # OpenCV < 4.7
    img = draw(dictionary, marker_id, px)
    # 1 cm de margen blanco: el de 17 cm cabe en hoja carta (21.6 cm de ancho)
    margin = int(round(1.0 / 2.54 * dpi))
    out = np.full((px + 2 * margin + 60, px + 2 * margin), 255, np.uint8)
    out[margin:margin + px, margin:margin + px] = img
    cv2.putText(out, f'id {marker_id}  {size_m * 100:.1f} cm', (margin, px + margin + 45),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 2)
    return out


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mapa', default=os.path.join(here, '..', 'config', 'aruco_map.yaml'))
    ap.add_argument('--salida', default='marcadores')
    ap.add_argument('--dpi', type=int, default=300)
    args = ap.parse_args()

    with open(args.mapa) as f:
        data = yaml.safe_load(f)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, data['dictionary']))
    markers = [(m['id'], m['size']) for m in data['markers']]
    os.makedirs(args.salida, exist_ok=True)
    for marker_id, size in markers:
        path = os.path.join(args.salida, f'aruco_{marker_id:02d}.png')
        cv2.imwrite(path, marker_png(dictionary, int(marker_id), float(size), args.dpi),
                    [cv2.IMWRITE_PNG_COMPRESSION, 9])
    print(f'{len(markers)} marcadores en {args.salida}/ a {args.dpi} dpi')


if __name__ == '__main__':
    main()
