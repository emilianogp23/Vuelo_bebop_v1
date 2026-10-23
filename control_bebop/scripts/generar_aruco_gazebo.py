#!/usr/bin/env python3
"""
Pone los ArUco de config/aruco_map_sim.yaml en el mundo de Gazebo (bebop1.sdf).

Crea en control_bebop/resource/meshes/texturas_lab/materials/textures/:
    aruco_NN.png   marcador con un módulo de margen blanco
y reescribe el modelo "aruco_marcadores" de bebop_gz/worlds/bebop1.sdf (entre
los comentarios ARUCO BEGIN/END): una placa delgada por marcador.

Uso:  python3 scripts/generar_aruco_gazebo.py
Después: colcon build --packages-select bebop_gz control_bebop
"""
import math
import os
import re

import cv2
import numpy as np
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP = os.path.join(PKG, 'config', 'aruco_map_sim.yaml')
TEX = os.path.join(PKG, 'resource', 'meshes', 'texturas_lab', 'materials', 'textures')
WORLD = os.path.join(PKG, '..', 'bebop_ros', 'bebop_gz', 'worlds', 'bebop1.sdf')
PX = 600          # lado de la textura; múltiplo de 6 módulos (4x4 + borde) + 2 de margen
THICK = 0.002


def marker_texture(dictionary, marker_id):
    draw = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    module = PX // 8
    img = np.full((PX, PX), 255, np.uint8)
    img[module:PX - module, module:PX - module] = draw(dictionary, marker_id, PX - 2 * module)
    return img


def visual(m):
    side = m['size'] * 8 / 6   # placa con el margen blanco
    x, y, z = m['center']
    if 'normal_yaw_deg' in m:
        # Placa vertical; su cara +x mira hacia normal_yaw
        yaw = math.radians(m['normal_yaw_deg'])
        x -= THICK / 2 * math.cos(yaw)
        y -= THICK / 2 * math.sin(yaw)
        size, rpy = f'{THICK} {side:.4f} {side:.4f}', f'0 0 {yaw:.6f}'
    else:
        size, rpy = f'{side:.4f} {side:.4f} {THICK}', f'0 0 {math.radians(m["floor_yaw_deg"]):.6f}'
        z -= THICK / 2
    tex = f'model://texturas_lab/materials/textures/aruco_{m["id"]:02d}.png'
    return f'''      <link name="aruco_{m['id']:02d}">
        <pose>{x:.4f} {y:.4f} {z:.4f} {rpy}</pose>
        <visual name="visual">
          <geometry><box><size>{size}</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular><pbr><metal><albedo_map>{tex}</albedo_map><roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr></material>
        </visual>
      </link>'''


def main():
    with open(MAP) as f:
        data = yaml.safe_load(f)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, data['dictionary']))
    os.makedirs(TEX, exist_ok=True)
    for m in data['markers']:
        cv2.imwrite(os.path.join(TEX, f'aruco_{m["id"]:02d}.png'), marker_texture(dictionary, m['id']))

    block = ('    <!-- ARUCO BEGIN: generado por control_bebop/scripts/generar_aruco_gazebo.py -->\n'
             '    <model name="aruco_marcadores">\n      <static>true</static>\n'
             + '\n'.join(visual(m) for m in data['markers'])
             + '\n    </model>\n    <!-- ARUCO END -->\n')
    with open(WORLD) as f:
        world = f.read()
    if '<!-- ARUCO BEGIN' in world:
        world = re.sub(r'    <!-- ARUCO BEGIN.*?<!-- ARUCO END -->\n', lambda _: block, world, flags=re.S)
    else:
        world = world.replace('  </world>', block + '\n  </world>')
    with open(WORLD, 'w') as f:
        f.write(world)
    print(f'{len(data["markers"])} marcadores → {TEX} y {os.path.normpath(WORLD)}')


if __name__ == '__main__':
    main()
