#!/usr/bin/env python3
"""
Vista 3D en tiempo real del vuelo (matplotlib).

Dibuja el corral, los ArUco del mapa, la zona segura, el objeto y la órbita
objetivo, y en vivo:
    /bebop1/pose         trayectoria que usa el control (con su rumbo)
    /vision/fused_pose   trayectoria corregida por visión (modo sombra)
    /vision/onboard/pose mediciones de la cámara del Bebop (puntos)
    /vision/ceiling/pose mediciones de la cámara del techo (puntos)
    /goal                setpoint actual
    /flight_state, /vision/lock en el título

Uso:
    ros2 run control_bebop plot_3d --ros-args --params-file config/real_drone.yaml
Con el ratón se gira la vista; la tecla R borra las trayectorias.
"""
import math
import threading
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

from control_bebop.frames import yaw_from_quaternion
from control_bebop.paths import config_file
from control_bebop.vision import load_marker_map

STATE_NAMES = {0: 'IDLE', 1: 'AUTOMATIC', 2: 'TAKING_OFF', 3: 'LANDING', 4: 'EMERGENCY',
               5: 'HOVER', 6: 'MANUAL', 7: 'RECOVERY', 8: 'RETURN_HOME'}


def _yaw(q):
    return yaw_from_quaternion(q.x, q.y, q.z, q.w)


class Plot3DNode(Node):
    def __init__(self):
        super().__init__('plot_3d')
        self.declare_parameter('map_file', '')
        self.declare_parameter('arena_x_max', 2.0)
        self.declare_parameter('corral_height', 2.5)
        self.declare_parameter('target_x', 1.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('circle_radius', 0.5)
        self.declare_parameter('circle_height', 1.2)     # solo para dibujar la órbita
        self.declare_parameter('object_height', 0.95)    # caja 0.70 + bote 0.25
        self.declare_parameter('object_size', 0.30)
        self.declare_parameter('safe_zone_radius', 0.5)
        self.declare_parameter('min_z_outside_safe', 1.0)
        self.declare_parameter('trail_seconds', 120.0)
        self.declare_parameter('rate', 5.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.p = {n: g(n) for n in (
            'arena_x_max', 'corral_height', 'target_x', 'target_y', 'circle_radius',
            'circle_height', 'object_height', 'object_size', 'safe_zone_radius',
            'min_z_outside_safe', 'trail_seconds', 'rate')}
        path = g('map_file') or config_file('aruco_map.yaml')
        try:
            _, self.markers, _ = load_marker_map(path)
        except (OSError, TypeError):
            self.markers = {}
            self.get_logger().warn(f'Sin mapa de marcadores ({path})')

        n = int(self.p['trail_seconds'] * 15)
        self.lock = threading.Lock()
        self.pose = deque(maxlen=n)        # (t, x, y, z)
        self.fused = deque(maxlen=n)
        self.onboard = deque(maxlen=300)
        self.ceiling = deque(maxlen=300)
        self.yaw = 0.0
        self.goal = None
        self.state = None
        self.vision_lock = None

        self.create_subscription(Pose, '/bebop1/pose', self.pose_cb, 10)
        self.create_subscription(PoseStamped, '/vision/fused_pose', self.fused_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/vision/onboard/pose',
                                 lambda m: self.meas_cb(m, self.onboard), 10)
        self.create_subscription(PoseWithCovarianceStamped, '/vision/ceiling/pose',
                                 lambda m: self.meas_cb(m, self.ceiling), 10)
        self.create_subscription(Pose, '/goal', self.goal_cb, 10)
        self.create_subscription(Int32, '/flight_state', self.state_cb, 10)
        self.create_subscription(Bool, '/vision/lock', self.lock_cb, 10)

    def pose_cb(self, m):
        with self.lock:
            self.pose.append((time.time(), m.position.x, m.position.y, m.position.z))
            self.yaw = _yaw(m.orientation)

    def fused_cb(self, m):
        p = m.pose.position
        with self.lock:
            self.fused.append((time.time(), p.x, p.y, p.z))

    def meas_cb(self, m, buf):
        p = m.pose.pose.position
        with self.lock:
            buf.append((time.time(), p.x, p.y, p.z))

    def goal_cb(self, m):
        self.goal = (m.position.x, m.position.y, m.position.z)

    def state_cb(self, m):
        self.state = m.data

    def lock_cb(self, m):
        self.vision_lock = m.data

    def clear(self):
        with self.lock:
            for b in (self.pose, self.fused, self.onboard, self.ceiling):
                b.clear()

    def snapshot(self):
        """Copia de los datos recientes (dentro de trail_seconds)."""
        t_min = time.time() - self.p['trail_seconds']
        with self.lock:
            return {k: [s[1:] for s in getattr(self, k) if s[0] >= t_min]
                    for k in ('pose', 'fused', 'onboard', 'ceiling')}, self.yaw


def _xyz(points):
    return ([p[0] for p in points], [p[1] for p in points], [p[2] for p in points]) \
        if points else ([], [], [])


def draw_static(ax, p, markers):
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    a, h = p['arena_x_max'], p['corral_height']
    # Corral
    for z in (0.0, h):
        ax.plot([-a, a, a, -a, -a], [-a, -a, a, a, -a], [z] * 5, color='0.6', lw=1)
    for x, y in ((-a, -a), (a, -a), (a, a), (-a, a)):
        ax.plot([x, x], [y, y], [0, h], color='0.6', lw=1)
    # Marcadores
    if markers:
        ax.add_collection3d(Poly3DCollection([c.tolist() for c in markers.values()],
                                             facecolors='k', edgecolors='k', alpha=0.8))
        for i, c in markers.items():
            cx, cy, cz = c.mean(axis=0)
            ax.text(cx, cy, cz + 0.12, str(i), fontsize=6, color='0.3')
    ang = [i * 2 * math.pi / 72 for i in range(73)]
    # Zona segura y nivel mínimo fuera de ella
    r = p['safe_zone_radius']
    ax.plot([r * math.cos(t) for t in ang], [r * math.sin(t) for t in ang], [0] * 73,
            color='tab:green', lw=2, label=f'Zona segura (r={r:.2f} m)')
    # Objeto (caja + bote) como prisma
    s, ox, oy, oh = p['object_size'] / 2, p['target_x'], p['target_y'], p['object_height']
    sq = [(ox - s, oy - s), (ox + s, oy - s), (ox + s, oy + s), (ox - s, oy + s)]
    faces = [[(x, y, 0) for x, y in sq], [(x, y, oh) for x, y in sq]]
    faces += [[(*sq[i], 0), (*sq[(i + 1) % 4], 0), (*sq[(i + 1) % 4], oh), (*sq[i], oh)]
              for i in range(4)]
    ax.add_collection3d(Poly3DCollection(faces, facecolors='tan', edgecolors='sienna', alpha=0.5))
    # Órbita objetivo
    cr, ch = p['circle_radius'], p['circle_height']
    ax.plot([ox + cr * math.cos(t) for t in ang], [oy + cr * math.sin(t) for t in ang],
            [ch] * 73, 'g--', lw=1, label=f'Órbita (r={cr:.2f} m)')
    ax.plot([0], [0], [0], 'k^', ms=8)
    ax.set_xlim(-a, a)
    ax.set_ylim(-a, a)
    ax.set_zlim(0, h)
    ax.set_box_aspect((2 * a, 2 * a, h))
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')


def main(args=None):
    rclpy.init(args=args)
    node = Plot3DNode()
    def spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass
    threading.Thread(target=spin, daemon=True).start()

    import matplotlib.pyplot as plt

    fig = plt.figure('Vuelo 3D', figsize=(9, 8))
    ax = fig.add_subplot(projection='3d')
    draw_static(ax, node.p, node.markers)
    path_l, = ax.plot([], [], [], 'm-', lw=2, label='/bebop1/pose (control)')
    fused_l, = ax.plot([], [], [], 'c-', lw=1.5, label='Corregida por visión (sombra)')
    onboard_s = ax.scatter([], [], [], s=6, c='tab:blue', label='Visión a bordo')
    ceiling_s = ax.scatter([], [], [], s=6, c='tab:orange', label='Cámara del techo')
    dot, = ax.plot([], [], [], 'mo', ms=8)
    goal_d, = ax.plot([], [], [], 'rx', ms=10, mew=2, label='Goal')
    head_l, = ax.plot([], [], [], 'r-', lw=2)
    ax.legend(loc='upper left', fontsize=8)
    fig.canvas.mpl_connect('key_press_event', lambda e: node.clear() if e.key == 'r' else None)

    period = 1.0 / node.p['rate']
    try:
        while plt.fignum_exists(fig.number) and rclpy.ok():
            data, yaw = node.snapshot()
            path_l.set_data_3d(*_xyz(data['pose']))
            fused_l.set_data_3d(*_xyz(data['fused']))
            onboard_s._offsets3d = _xyz(data['onboard'])
            ceiling_s._offsets3d = _xyz(data['ceiling'])
            if data['pose']:
                x, y, z = data['pose'][-1]
                dot.set_data_3d([x], [y], [z])
                head_l.set_data_3d([x, x + 0.3 * math.cos(yaw)], [y, y + 0.3 * math.sin(yaw)], [z, z])
                low = (z < node.p['min_z_outside_safe']
                       and math.hypot(x, y) >= node.p['safe_zone_radius'])
                pos = f'({x:+.2f}, {y:+.2f}, {z:.2f}) m  {math.degrees(yaw):+.0f}°'
            else:
                low, pos = False, 'sin /bebop1/pose'
            if node.goal:
                goal_d.set_data_3d(*([v] for v in node.goal))
            lock = {None: 'sin datos', True: 'LOCK', False: 'sin lock'}[node.vision_lock]
            ax.set_title(f'{STATE_NAMES.get(node.state, "—")}  |  {pos}  |  visión: {lock}'
                         + ('  |  ⚠ BAJO FUERA DE ZONA SEGURA' if low else ''),
                         color='red' if low else 'black', fontsize=10)
            plt.pause(period)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
