#!/usr/bin/env python3
"""
Recreación 3D de un vuelo grabado (bag de main_flight / real_flight).

Dibuja el corral, los ArUco, el objeto y la órbita (igual que plot_3d) y anima:
    odometría sola       deriva sin corregir: /bebop1/pose en modo sombra,
                         /vision/odom_pose con la visión al mando
    corregida con ArUco  /vision/fused_pose en modo sombra, /bebop1/pose al mando
    trayectoria propuesta /goal (setpoints del planificador)
    real                 /bebop1/pose_gt (solo simulación)
    mediciones ArUco     /vision/onboard/pose (puntos)
A la derecha, error XY y altura contra el tiempo. La referencia del error es la
pose real en simulación y /goal en el dron real.

Uso (después de `source /opt/ros/<distro>/setup.bash`):
    python3 scripts/replay_3d.py bags/vuelo_sim_YYYYMMDD_HHMMSS
    python3 scripts/replay_3d.py <bag> --static            # solo la gráfica final
    python3 scripts/replay_3d.py <bag> --save vuelo.mp4 --speed 2
    python3 scripts/replay_3d.py <bag> --params config/real_drone.yaml --desde 30 --hasta 90
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)

from analisis_bag import STATES, load, zoh  # noqa: E402
from control_bebop.paths import config_file  # noqa: E402
from control_bebop.plot_3d import PARAMS, draw_static  # noqa: E402
from control_bebop.vision import load_marker_map  # noqa: E402

TOPICS = {'/bebop1/pose', '/bebop1/pose_gt', '/goal', '/flight_state', '/vision/onboard/pose',
          '/vision/fused_pose', '/vision/odom_pose', '/vision/lock'}
DT = 0.05   # s entre cuadros de la animación


def read_params(path):
    """PARAMS de plot_3d con los valores del yaml (secciones /**, trajectory_planner, plot_3d)."""
    p = dict(PARAMS)
    with open(path) as f:
        sections = yaml.safe_load(f) or {}
    for name in sorted(sections, key=lambda n: n == 'plot_3d'):   # plot_3d manda
        params = (sections[name] or {}).get('ros__parameters', {})
        p.update({k: v for k, v in params.items() if k in PARAMS})
    return p


def series(d):
    """{etiqueta: (datos, estilo)} según el modo de visión con que se grabó."""
    shadow = '/vision/fused_pose' in d
    vision = shadow or '/vision/onboard/pose' in d or '/vision/lock' in d
    if shadow:
        odom, corr = d['/bebop1/pose'], d['/vision/fused_pose']
    elif vision:
        odom, corr = d.get('/vision/odom_pose'), d['/bebop1/pose']
        if odom is None:
            print('Aviso: bag sin /vision/odom_pose (grabado antes de publicarla): '
                  'no hay odometría sola con la visión al mando')
    else:
        odom, corr = d['/bebop1/pose'], None
    out = {
        'Odometría sola': (odom, dict(color='tab:orange', lw=1.6)),
        'Corregida con ArUco': (corr, dict(color='tab:blue', lw=2.0)),
        'Trayectoria propuesta (/goal)': (d.get('/goal'), dict(color='tab:green', lw=1.0, alpha=0.8)),
        'Real (Gazebo)': (d.get('/bebop1/pose_gt'), dict(color='k', lw=1.4)),
    }
    return {k: v for k, v in out.items() if v[0] is not None}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='Carpeta del bag o archivo .db3')
    ap.add_argument('--params', help='yaml de configuración (por defecto sim.yaml o real_drone.yaml)')
    ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--static', action='store_true', help='Solo la gráfica final')
    ap.add_argument('--save', help='Guardar la animación (.mp4 o .gif)')
    ap.add_argument('--desde', type=float, help='s del bag (por defecto: despegue)')
    ap.add_argument('--hasta', type=float, help='s del bag (por defecto: aterrizaje)')
    a = ap.parse_args()

    d = load(a.bag, TOPICS)
    if '/bebop1/pose' not in d or '/flight_state' not in d:
        sys.exit('El bag no tiene /bebop1/pose o /flight_state.')
    sim = '/bebop1/pose_gt' in d
    p = read_params(a.params or os.path.join(PKG, 'config', 'sim.yaml' if sim else 'real_drone.yaml'))
    try:
        _, markers = load_marker_map(config_file(p['map_file'] or 'aruco_map.yaml'))
    except (OSError, TypeError):
        markers = {}

    st = d['/flight_state']
    air = st[st[:, 1] != 0, 0]
    t0 = a.desde if a.desde is not None else (air[0] - 1.0 if len(air) else st[0, 0])
    t1 = a.hasta if a.hasta is not None else (air[-1] + 1.0 if len(air) else st[-1, 0])
    tg = np.arange(t0, t1, DT)
    if len(tg) < 2:
        sys.exit(f'Ventana vacía: {t0:.1f}–{t1:.1f} s')

    # Todas las series al mismo reloj (la pose real de Gazebo llega a ~1 kHz)
    sers = {k: (np.column_stack([zoh(s, tg, c) for c in (1, 2, 3, 4)]), sty)
            for k, (s, sty) in series(d).items()}
    state = zoh(st, tg, 1).astype(int)
    ref_name = 'Real (Gazebo)' if sim else 'Trayectoria propuesta (/goal)'
    ref = sers.get(ref_name, (None,))[0]
    meas = d.get('/vision/onboard/pose')
    if meas is not None:
        meas = meas[(meas[:, 0] >= t0) & (meas[:, 0] <= t1)]

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig = plt.figure('Vuelo 3D (bag)', figsize=(15, 8))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.5, 1])
    ax = fig.add_subplot(gs[:, 0], projection='3d')
    axe = fig.add_subplot(gs[0, 1])
    axz = fig.add_subplot(gs[1, 1], sharex=axe)
    draw_static(ax, p, markers)

    lines, dots = {}, {}
    for k, (s, sty) in sers.items():
        lines[k], = ax.plot([], [], [], label=k, **sty)
        dots[k], = ax.plot([], [], [], 'o', color=sty['color'], ms=6)
        axz.plot(tg, s[:, 2], label=k, **sty)
        if ref is not None and k != ref_name:
            e = np.hypot(s[:, 0] - ref[:, 0], s[:, 1] - ref[:, 1])
            axe.plot(tg, e, label=f'{k} (máx {e.max():.2f}, final {e[-1]:.2f} m)', **sty)
    head, = ax.plot([], [], [], 'r-', lw=2)   # rumbo de la cámara
    meas_s = ax.scatter([], [], [], s=5, c='tab:purple', label='Mediciones ArUco')
    ax.legend(loc='upper left', fontsize=7)
    axe.set_ylabel(f'Error XY vs {"real" if sim else "/goal"} (m)')
    axe.grid(True)
    axe.legend(fontsize=7)
    axz.set_ylabel('Z (m)')
    axz.set_xlabel('t del bag (s)')
    axz.grid(True)
    cursors = [axe.axvline(t0, color='r', lw=0.8), axz.axvline(t0, color='r', lw=0.8)]
    main_k = ref_name if sim else 'Corregida con ArUco'
    main_k = main_k if main_k in sers else next(iter(sers))
    title = ax.set_title('')

    def draw(k):
        for name, (s, _) in sers.items():
            lines[name].set_data_3d(s[:k, 0], s[:k, 1], s[:k, 2])
            dots[name].set_data_3d(s[k - 1:k, 0], s[k - 1:k, 1], s[k - 1:k, 2])
        x, y, z, yaw = sers[main_k][0][k - 1]
        head.set_data_3d([x, x + 0.35 * math.cos(yaw)], [y, y + 0.35 * math.sin(yaw)], [z, z])
        if meas is not None:
            m = meas[meas[:, 0] <= tg[k - 1]]
            meas_s._offsets3d = (m[:, 1], m[:, 2], m[:, 3])
        for c in cursors:
            c.set_xdata([tg[k - 1]])
        title.set_text(f't={tg[k - 1]:.1f} s  {STATES.get(state[k - 1], state[k - 1])}  '
                       f'rumbo ({main_k}) {math.degrees(yaw):+.0f}°')

    if a.static:
        draw(len(tg))
        plt.show()
        return
    step = max(1, round(a.speed))   # cuadros de DT que avanza cada dibujo
    frames = list(range(step, len(tg), step)) + [len(tg)]
    anim = FuncAnimation(fig, draw, frames=frames, interval=1000 * DT * step / a.speed,
                         repeat=False)
    if a.save:
        anim.save(a.save, fps=round(1 / (DT * step / a.speed)))
        print('Guardado en', a.save)
    else:
        plt.show()


if __name__ == '__main__':
    main()
