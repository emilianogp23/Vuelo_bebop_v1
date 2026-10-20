#!/usr/bin/env python3
"""
Calibración de la cámara del techo (C920). Escribe config/ceiling_camera.yaml.

    ros2 run control_bebop calibrar_camara_techo intrinsecos --tablero 9x6 --cuadro 0.025
        Ventana en vivo: ESPACIO guarda una vista del tablero (mover el tablero por
        todo el cuadro y a distintas inclinaciones), C calibra con >= 15 vistas, Q sale.

    ros2 run control_bebop calibrar_camara_techo extrinsecos
        Con los ArUco del piso (y de las vallas) del mapa a la vista y el dron
        fuera: calcula dónde está la cámara en el marco corral.

Correr intrínsecos primero y repetir extrínsecos cada vez que se mueva la cámara.
"""
import argparse
import sys

import cv2
import numpy as np
import yaml

from control_bebop.ceiling_tracker import load_camera_config, open_camera
from control_bebop.paths import config_file, source_config_path
from control_bebop.vision import (camera_in_world, detect, load_marker_map,
                                  make_detector, solve_pnp)

HEADER = """# Cámara del techo (Logitech C920). Lo escribe calibrar_camara_techo.
# focus: 0 = infinito en la C920; autofoco siempre apagado.
# exposure: -1 = automática; un valor fijo evita cambios con la luz.
"""


def save(cfg):
    path = source_config_path('ceiling_camera.yaml')
    with open(path, 'w') as f:
        f.write(HEADER)
        yaml.safe_dump(cfg, f, sort_keys=False, default_flow_style=None)
    print(f'Guardado: {path}')


def intrinsics(cfg, cols, rows, square):
    cap = open_camera(cfg)
    pattern = (cols, rows)
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    obj_pts, img_pts, size = [], [], None
    while True:
        ok, frame = cap.read()
        if not ok:
            print('No se puede leer la cámara')
            return 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, pattern)
        view = frame.copy()
        if found:
            cv2.drawChessboardCorners(view, pattern, corners, found)
        cv2.putText(view, f'vistas: {len(obj_pts)}  ESPACIO guardar  C calibrar  Q salir',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow('intrinsecos', view)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' ') and found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                       (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
            obj_pts.append(objp)
            img_pts.append(corners)
        elif key == ord('c') and len(obj_pts) >= 15:
            err, K, D, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
            print(f'Error de reproyección: {err:.3f} px (objetivo < 0.5)')
            cfg.update(width=size[0], height=size[1], camera_matrix=K.flatten().tolist(),
                       dist_coeffs=D.flatten().tolist(), intrinsics_calibrated=True)
            # Los extrínsecos dependen de los intrínsecos: hay que repetirlos
            cfg['extrinsics'] = {'calibrated': False}
            save(cfg)
            return 0
        elif key == ord('q'):
            return 1


def extrinsics(cfg, frames=30):
    dict_name, corners_map, _ = load_marker_map(config_file('aruco_map.yaml'))
    detector = make_detector(dict_name)
    K = np.array(cfg['camera_matrix'], dtype=np.float64).reshape(3, 3)
    D = np.array(cfg['dist_coeffs'], dtype=np.float64)
    cap = open_camera(cfg)
    best = None
    for _ in range(frames):
        ok, frame = cap.read()
        if not ok:
            continue
        found = {i: c for i, c in detect(detector, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)).items()
                 if i in corners_map}
        if len(found) < 2:
            continue
        obj = np.vstack([corners_map[i] for i in found])
        sol = solve_pnp(obj, np.vstack(list(found.values())), K, D)
        if sol and (best is None or (len(found), -sol[2]) > (best[0], -best[1][2])):
            best = (len(found), sol, sorted(found))
    if best is None:
        print('Se necesitan al menos 2 marcadores del mapa a la vista.')
        return 1
    n, (rvec, tvec, err), ids = best
    _, pos = camera_in_world(rvec, tvec)
    print(f'Marcadores usados: {ids}  error {err:.2f} px')
    print(f'Cámara en el corral: x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f} m '
          '(z debería ser ~2.74)')
    cfg['extrinsics'] = {'calibrated': True, 'rvec': rvec.tolist(), 'tvec': tvec.tolist(),
                         'reproj_px': round(err, 3), 'markers': ids}
    save(cfg)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('modo', choices=['intrinsecos', 'extrinsecos'])
    ap.add_argument('--tablero', default='9x6', help='esquinas internas, columnas x filas')
    ap.add_argument('--cuadro', type=float, default=0.025, help='lado del cuadro (m)')
    args = ap.parse_args(argv)
    cfg = load_camera_config(config_file('ceiling_camera.yaml'))
    if args.modo == 'intrinsecos':
        cols, rows = (int(v) for v in args.tablero.lower().split('x'))
        return intrinsics(cfg, cols, rows, args.cuadro)
    return extrinsics(cfg)


if __name__ == '__main__':
    sys.exit(main())
