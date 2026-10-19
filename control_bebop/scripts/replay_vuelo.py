#!/usr/bin/env python3
"""
Reproduce un vuelo grabado (CSV *_datos.csv generado desde el bag) en una
gráfica animada: vista superior XY con flecha de yaw, altura Z vs tiempo,
yaw vs tiempo y estado de vuelo.

Uso:
    python3 replay_vuelo.py <archivo.csv> [--speed 1.0] [--save salida.mp4|gif]
                                          [--static] [--min-z 0.5]
"""
import argparse
import csv
import math
import bisect

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

STATES = {0: 'IDLE', 1: 'AUTOMATIC', 2: 'TAKING_OFF', 3: 'LANDING',
          4: 'EMERGENCY', 5: 'HOVER', 6: 'MANUAL', 7: 'RECOVERY',
          8: 'RETURN_HOME'}


def load_db3(path):
    """Lee directamente un bag .db3 (requiere haber hecho source de ROS 2)."""
    import glob
    import os
    import sqlite3
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import Pose
    from std_msgs.msg import Int32

    if not os.path.exists(path):
        # Intentar buscar como prefijo de directorio en bags/ o en el cwd
        matches = glob.glob(path + '*')
        if matches:
            path = matches[0]
        else:
            raise FileNotFoundError(f"No se encontró el archivo o carpeta: {path}")

    if os.path.isdir(path):
        db_files = glob.glob(os.path.join(path, '*.db3'))
        if not db_files:
            raise FileNotFoundError(f"No se encontró ningún archivo .db3 dentro de {path}")
        path = db_files[0]
    elif not path.endswith('.db3'):
        # Si pasaron un path sin extensión pero apunta a archivo o directorio
        db_files = glob.glob(path + '*.db3')
        if db_files:
            path = db_files[0]

    if os.path.getsize(path) == 0:
        raise ValueError(f"El archivo {path} está vacío (0 bytes).")

    con = sqlite3.connect(path)
    topics = {i: n for i, n in con.execute('SELECT id, name FROM topics')}
    out = {'pose': [], 'goal': [], 'state': []}
    t0 = None
    for tid, ts, blob in con.execute(
            'SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp'):
        name = topics[tid]
        if name not in ('/bebop1/pose', '/goal', '/flight_state'):
            continue
        if t0 is None:
            t0 = ts
        t = (ts - t0) * 1e-9
        if name == '/flight_state':
            out['state'].append((t, deserialize_message(blob, Int32).data))
            continue
        m = deserialize_message(blob, Pose)
        q = m.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        key = 'pose' if name == '/bebop1/pose' else 'goal'
        out[key].append((t, m.position.x, m.position.y, m.position.z, yaw))
    arr = lambda L, n: [np.array(c) for c in zip(*L)] if L else \
        [np.array([]) for _ in range(n)]
    return {'pose': arr(out['pose'], 5), 'odom': [np.array([])] * 5,
            'goal': arr(out['goal'], 5)[:4],
            'state': arr(out['state'], 2)}


def load(path):
    if not path.endswith('.csv'):
        return load_db3(path)
    rows = list(csv.DictReader(open(path)))
    def col(topic, keys):
        out = [[] for _ in range(len(keys) + 1)]
        for r in rows:
            if r['Topico'] != topic:
                continue
            try:
                vals = [float(r['Tiempo_s'])] + [float(r[k]) for k in keys]
            except ValueError:
                continue
            for o, v in zip(out, vals):
                o.append(v)
        return [np.array(o) for o in out]
    return {
        'pose': col('local_pose', ['X', 'Y', 'Z', 'Yaw']),
        'odom': col('/bebop/odom', ['X', 'Y', 'Z', 'Yaw']),
        'goal': col('goal', ['X', 'Y', 'Z']),
        'state': col('state', ['Estado_vuelo']),
        'cmd': col('/bebop/cmd_vel', ['Vx', 'Vy', 'Vz', 'V_yaw']),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--save', default=None)
    ap.add_argument('--static', action='store_true',
                    help='Solo gráfica final, sin animación')
    ap.add_argument('--min-z', type=float, default=0.5,
                    help='Umbral de altura peligrosa en vuelo (m)')
    a = ap.parse_args()

    d = load(a.csv)
    t, x, y, z, yaw = d['pose']
    if len(t) == 0:   # respaldo: odometría cruda
        t, x, y, z, yaw = d['odom']
    yaw = np.unwrap(yaw)
    gt, gx, gy, gz = d['goal']
    st_t, st = d['state']
    st = st.astype(int)

    # Altura mínima mientras el dron está realmente volando
    def state_at(tt):
        i = bisect.bisect_right(st_t, tt) - 1
        return st[i] if i >= 0 else 0
    flying = np.array([state_at(tt) in (1, 5, 6) for tt in t])
    if flying.any():
        zf = z[flying]
        print(f'Z en vuelo (AUTOMATIC/HOVER/MANUAL): min={zf.min():.2f} '
              f'max={zf.max():.2f} m; bajo {a.min_z} m: '
              f'{np.mean(zf < a.min_z) * 100:.1f}% del tiempo')
        if len(gz):
            print(f'Z de /goal: min={gz.min():.2f} max={gz.max():.2f}')
        # Mayor descenso estando en AUTOMATIC
        auto = np.array([state_at(tt) == 1 for tt in t])
        if auto.any():
            print(f'En AUTOMATIC: Z min={z[auto].min():.2f} '
                  f'max={z[auto].max():.2f}')

    fig = plt.figure(figsize=(13, 7))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.1, 1])
    axx = fig.add_subplot(gs[:, 0])
    axz = fig.add_subplot(gs[0, 1])
    axy = fig.add_subplot(gs[1, 1], sharex=axz)
    axs = fig.add_subplot(gs[2, 1], sharex=axz)

    pad = 0.3
    axx.set_xlim(min(x.min(), gx.min() if len(gx) else 0) - pad,
                 max(x.max(), gx.max() if len(gx) else 0) + pad)
    axx.set_ylim(min(y.min(), gy.min() if len(gy) else 0) - pad,
                 max(y.max(), gy.max() if len(gy) else 0) + pad)
    axx.set_aspect('equal'); axx.grid(True)
    axx.set_xlabel('X (m)'); axx.set_ylabel('Y (m)')
    axx.set_title('Vista superior (posición estimada por odometría)')
    if len(gx):
        axx.plot(gx, gy, 'g.', ms=2, alpha=.4, label='Setpoints /goal')
    axx.plot(0, 0, 'k^', ms=10, label='Despegue')
    path_l, = axx.plot([], [], 'm-', lw=1.8, label='Trayectoria (odometría)')
    dot, = axx.plot([], [], 'mo', ms=8)
    arrow = [None]
    axx.legend(loc='upper left', fontsize=8)

    axz.plot(t, z, 'b-', alpha=.25)
    if len(gz):
        axz.plot(gt, gz, 'g--', alpha=.5, label='Z goal')
    axz.axhline(a.min_z, color='r', ls=':', label=f'{a.min_z} m')
    zl, = axz.plot([], [], 'b-', lw=1.8, label='Z de /bebop1/pose')
    axz.set_ylabel('Z (m)'); axz.grid(True); axz.legend(fontsize=7)
    axy.plot(t, np.degrees(yaw), 'k-', alpha=.25)
    yl, = axy.plot([], [], 'k-', lw=1.8)
    axy.set_ylabel('Yaw (°)'); axy.grid(True)
    axs.step(st_t, st, 'r-', where='post', alpha=.25)
    sl, = axs.step([], [], 'r-', where='post')
    axs.set_yticks(list(STATES)); axs.set_yticklabels(
        [f'{k} {v[:6]}' for k, v in STATES.items()], fontsize=6)
    axs.set_ylabel('Estado'); axs.set_xlabel('t (s)'); axs.grid(True)
    ttl = fig.suptitle('')

    def draw(k):
        path_l.set_data(x[:k], y[:k]); dot.set_data([x[k - 1]], [y[k - 1]])
        if arrow[0] is not None:
            arrow[0].remove()
        L = 0.15
        arrow[0] = axx.arrow(x[k - 1], y[k - 1], L * math.cos(yaw[k - 1]),
                             L * math.sin(yaw[k - 1]), head_width=0.06,
                             color='orange', zorder=5)
        zl.set_data(t[:k], z[:k]); yl.set_data(t[:k], np.degrees(yaw[:k]))
        n = bisect.bisect_right(st_t, t[k - 1])
        sl.set_data(st_t[:n], st[:n])
        s = state_at(t[k - 1])
        ttl.set_text(f't={t[k-1]:.1f}s  Z={z[k-1]:.2f} m  '
                     f'yaw={math.degrees(yaw[k-1]):.0f}°  '
                     f'estado={STATES.get(s, s)}')

    if a.static:
        draw(len(t))
        plt.show()
        return

    dt_mean = (t[-1] - t[0]) / max(1, len(t) - 1)
    interval = 40  # ms
    step = max(1, int(round(a.speed * interval / 1000.0 / dt_mean)))
    frames = list(range(step, len(t) + step, step))
    frames = [min(f, len(t)) for f in frames]
    anim = FuncAnimation(fig, draw, frames=frames, interval=interval,
                         repeat=False)
    if a.save:
        anim.save(a.save, fps=25)
        print('Guardado en', a.save)
    else:
        plt.show()


if __name__ == '__main__':
    main()
