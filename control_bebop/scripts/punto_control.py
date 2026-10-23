#!/usr/bin/env python3
"""
Prueba de puntos de control de la localización visual (guía §6.5).

Con el driver y aruco_localizer corriendo, se sostiene el dron quieto sobre un
punto medido del corral. Promedia /vision/onboard/pose unos segundos, lo compara
con el punto real, dice qué marcadores ve la cámara y agrega una fila al CSV.

    /usr/bin/python3 scripts/punto_control.py X Y Z YAW_DEG [--nombre P1]

X, Y, Z en metros en el marco corral (z = altura del centro del dron sobre el
piso), YAW_DEG hacia dónde mira el dron (+x = 0, +y = 90).
"""
import argparse
import csv
import math
import os
import sys
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from control_bebop.frames import normalize_angle, yaw_from_quaternion  # noqa: E402
from control_bebop.paths import config_file  # noqa: E402
from control_bebop.vision import detect, load_marker_map, make_detector  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    for n in ('x', 'y', 'z', 'yaw_deg'):
        ap.add_argument(n, type=float)
    ap.add_argument('--nombre', default='')
    ap.add_argument('--segundos', type=float, default=3.0)
    ap.add_argument('--csv', default='puntos_control.csv')
    a = ap.parse_args()

    dict_name, marker_map = load_marker_map(config_file('aruco_map.yaml'))
    detector = make_detector(dict_name)
    bridge = CvBridge()
    poses, seen, frames = [], {}, [0]

    def pose_cb(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        poses.append((p.x, p.y, p.z, yaw_from_quaternion(q.x, q.y, q.z, q.w)))

    def image_cb(m):
        frames[0] += 1
        for i in detect(detector, bridge.imgmsg_to_cv2(m, desired_encoding='mono8')):
            if i in marker_map:
                seen[i] = seen.get(i, 0) + 1

    rclpy.init()
    node = rclpy.create_node('punto_control')
    node.create_subscription(PoseWithCovarianceStamped, '/vision/onboard/pose', pose_cb, 10)
    node.create_subscription(Image, '/bebop/camera/image_raw', image_cb, qos_profile_sensor_data)
    print(f'Midiendo {a.segundos:.0f} s, no muevas el dron...')
    end = time.time() + a.segundos
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    # ids vistos en al menos la mitad de los cuadros (descarta detecciones sueltas)
    ids = sorted(i for i, c in seen.items() if c >= frames[0] / 2)
    print(f'Cuadros {frames[0]} | marcadores vistos: {ids or "ninguno"}'
          + ('  ⚠️ incluye 3 o 5 (sin re-medir)' if {3, 5} & set(ids) else ''))
    if not poses:
        print('Sin poses de aruco_localizer: es un hueco (o el nodo no corre).')
        row = [a.nombre, a.x, a.y, a.z, a.yaw_deg, 0] + [''] * 8 + [' '.join(map(str, ids))]
    else:
        P = np.array(poses)
        mx, my, mz = P[:, :3].mean(axis=0)
        myaw = math.atan2(np.sin(P[:, 3]).mean(), np.cos(P[:, 3]).mean())
        exy = math.hypot(mx - a.x, my - a.y)
        dyaw = math.degrees(normalize_angle(myaw - math.radians(a.yaw_deg)))
        std = P[:, :2].std(axis=0).max()
        ok = exy < 0.05 and abs(dyaw) < 2.0
        print(f'Poses {len(P)} | medido x {mx:.3f} y {my:.3f} z {mz:.3f} yaw {math.degrees(myaw):.1f}°'
              f' | ruido {std * 100:.1f} cm')
        print(f'Error xy {exy * 100:.1f} cm (dx {(mx - a.x) * 100:+.1f}, dy {(my - a.y) * 100:+.1f})'
              f' | dz {(mz - a.z) * 100:+.1f} cm | dyaw {dyaw:+.1f}°  → {"OK" if ok else "FALLA"}'
              ' (criterio < 5 cm y < 2°)')
        row = [a.nombre, a.x, a.y, a.z, a.yaw_deg, len(P), round(mx, 3), round(my, 3), round(mz, 3),
               round(math.degrees(myaw), 1), round(exy * 100, 1), round((mz - a.z) * 100, 1),
               round(dyaw, 1), 'OK' if ok else 'FALLA', ' '.join(map(str, ids))]

    new = not os.path.exists(a.csv)
    with open(a.csv, 'a', newline='') as f:
        w = csv.writer(f)
        if new:
            w.writerow(['nombre', 'x', 'y', 'z', 'yaw_deg', 'poses', 'mx', 'my', 'mz', 'myaw_deg',
                        'err_xy_cm', 'dz_cm', 'dyaw_deg', 'resultado', 'ids'])
        w.writerow(row)
    print(f'Guardado en {os.path.abspath(a.csv)}')


if __name__ == '__main__':
    main()
