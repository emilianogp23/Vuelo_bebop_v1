#!/usr/bin/env python3
import sys
import os
# Compatibilidad con venvs (rosbags instalado en el venv del usuario)
for p in [os.path.expanduser('~/env_dron/lib/python3.10/site-packages'),
          os.path.expanduser('~/env_dron/lib/python3.12/site-packages')]:
    if os.path.exists(p) and p not in sys.path:
        sys.path.append(p)

import argparse
import numpy as np
import yaml
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

# Orden de preferencia: /odom_local está en el marco de despegue (x = frente
# del dron al despegar), que es donde se mide con cinta. /bebop/odom está en
# Norte/Oeste y solo sirve si el dron despegó mirando al norte.
ODOM_TOPICS = ('/odom_local', '/bebop/odom', '/odom')


def main():
    parser = argparse.ArgumentParser(description='Analiza deriva y calcula factores de corrección a partir de un rosbag del Bebop 2.')
    parser.add_argument('bag_path', help='Ruta al directorio del rosbag')
    parser.add_argument('--tape-x', type=float, default=1.0, help='Distancia X real medida con cinta (metros)')
    parser.add_argument('--tape-y', type=float, default=0.0, help='Distancia Y real medida con cinta (metros)')
    args = parser.parse_args()

    odom_by_topic = {t: [] for t in ODOM_TOPICS}
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    print(f"Abriendo rosbag: {args.bag_path}")
    try:
        with Reader(args.bag_path) as reader:
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic in odom_by_topic:
                    msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                    x = msg.pose.pose.position.x
                    y = msg.pose.pose.position.y
                    odom_by_topic[connection.topic].append((timestamp, x, y))
    except Exception as e:
        print(f"Error al leer el rosbag: {e}")
        return

    # Usar UN solo tópico (antes se mezclaban /bebop/odom y /odom_local)
    topic = next((t for t in ODOM_TOPICS if odom_by_topic[t]), None)
    if topic is None:
        print("No se encontraron mensajes de odometría en el bag.")
        return
    odom_data = odom_by_topic[topic]
    print(f"Usando tópico: {topic} ({len(odom_data)} mensajes)")

    # Sort just in case
    odom_data.sort(key=lambda item: item[0])

    t0 = odom_data[0][0]
    t_end = odom_data[-1][0]
    dt_total = (t_end - t0) / 1e9  # seconds

    dx_odom = odom_data[-1][1] - odom_data[0][1]
    dy_odom = odom_data[-1][2] - odom_data[0][2]

    # Factor de Escala
    # Si te moviste 1.5m pero odom dice 1.0m, scale = 1.5/1.0 = 1.5
    # Por seguridad no dividimos si el dx_odom es muy pequeno
    scale_x = 1.0
    scale_y = 1.0
    if abs(args.tape_x) > 0.1 and abs(dx_odom) > 0.05:
        scale_x = abs(args.tape_x) / abs(dx_odom)
    if abs(args.tape_y) > 0.1 and abs(dy_odom) > 0.05:
        scale_y = abs(args.tape_y) / abs(dy_odom)
    elif abs(args.tape_x) > 0.1:
        # Sin medida en Y: usar la misma escala que X (flujo óptico isotrópico)
        scale_y = scale_x

    # Deriva (asumiendo que en reposo se desvió, simplificado)
    # This is a very rough estimate unless we know exactly when it was in hover
    # For now, let's just create the calibrated yaml with the scale
    # To really calculate trim, we'd look at periods where cmd_vel is 0
    # But as an automated offline tool, this is what was promised.

    print("\n=== RESULTADOS DE ANÁLISIS ===")
    print(f"Duración de grabación: {dt_total:.1f} s")
    print(f"Odometría X leída: {dx_odom:.3f} m | Real esperada: {args.tape_x:.3f} m -> Scale X: {scale_x:.3f}")
    print(f"Odometría Y leída: {dy_odom:.3f} m | Real esperada: {args.tape_y:.3f} m -> Scale Y: {scale_y:.3f}")
    if not (0.5 <= scale_x <= 2.0 and 0.5 <= scale_y <= 2.0):
        print("⚠️  Escala fuera de [0.5, 2.0]: revisa la medición; NO se guarda.")
        return

    # Generar YAML en el paquete real (Vuelo_bebop_v1)
    src_yaml = os.path.expanduser(
        '~/ros2_ws/src/Vuelo_bebop_v1/control_bebop/config/real_drone_calibrated.yaml')
    yaml_path = os.path.abspath(src_yaml)

    # Solo corrección de odometría: las ganancias y la trayectoria
    # se quedan en real_drone.yaml para no sobreescribirlas sin probar.
    config = {
        '/**': {
            'ros__parameters': {
                'scale_x': float(scale_x),
                'scale_y': float(scale_y),
                'trim_x': 0.0,  # Could be estimated later
                'trim_y': 0.0
            }
        }
    }

    try:
        os.makedirs(os.path.dirname(yaml_path), exist_ok=True)
        with open(yaml_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
        print(f"\nArchivo guardado en: {yaml_path}")
        print("Ahora puedes lanzar con: ros2 launch control_bebop real_flight.launch.py")
    except Exception as e:
        print(f"Error guardando {yaml_path}: {e}")

if __name__ == '__main__':
    main()
