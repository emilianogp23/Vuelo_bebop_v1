#!/usr/bin/env python3
"""
Diagnóstico rápido de un vuelo real grabado con real_flight.launch.py.

Reporta:
    - Transiciones de /flight_state.
    - Error de seguimiento pose vs goal en AUTOMATIC (máximo, RMS) y el
      sobrepaso en la fase de aproximación (primeros segundos de AUTOMATIC).
    - Mensajes con gaz != 0 en AUTOMATIC (con z_mode native debe ser 0).
    - Comando de yaw máximo en AUTOMATIC (saturación de la órbita lookat).
    - Tiempo con el goal detenido en AUTOMATIC (órbita frenada por deriva) y
      avisos de /trajectory_stalled.
    - z de odometría al final (en el piso): muestra la deriva de la z integrada,
      y la altura medida /bebop/altitude si el bag la tiene.
    - XY final de odometría: si se aterrizó en el origen, es el error acumulado.
    - Identificación del modelo pitch → velocidad:  dv/dt = a·u(t-d) - b·v

Uso (después de `source /opt/ros/humble/setup.bash`):
    python3 analisis_bag.py bags/vuelo_real_YYYYMMDD_HHMMSS
    python3 analisis_bag.py <carpeta_o_.db3> --approach 20
"""
import argparse
import glob
import math
import os
import sqlite3

import numpy as np

STATES = {0: 'IDLE', 1: 'AUTOMATIC', 2: 'TAKING_OFF', 3: 'LANDING',
          4: 'EMERGENCY', 5: 'HOVER', 6: 'MANUAL', 7: 'RECOVERY',
          8: 'RETURN_HOME'}
AUTOMATIC = 1


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load(path):
    """Devuelve {tópico: [(t, valores...)]} con t en segundos desde el inicio."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    if os.path.isdir(path):
        dbs = sorted(glob.glob(os.path.join(path, '*.db3')))
        if not dbs:
            raise FileNotFoundError(f'No hay .db3 en {path}')
        path = dbs[0]
    con = sqlite3.connect(path)
    topics = {r[0]: (r[1], get_message(r[2]))
              for r in con.execute('SELECT id, name, type FROM topics')}
    data = {}
    t0 = None
    for tid, ts, raw in con.execute(
            'SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp'):
        name, mtype = topics[tid]
        t = ts * 1e-9
        t0 = t if t0 is None else t0
        t -= t0
        m = deserialize_message(raw, mtype)
        if name in ('/bebop1/pose', '/goal'):
            row = (t, m.position.x, m.position.y, m.position.z, _yaw(m.orientation))
        elif name == '/odom_local':
            p = m.pose.pose
            row = (t, p.position.x, p.position.y, p.position.z,
                   _yaw(p.orientation), m.twist.twist.linear.x, m.twist.twist.linear.y)
        elif name == '/bebop/odom':
            p = m.pose.pose
            row = (t, p.position.x, p.position.y, p.position.z, _yaw(p.orientation))
        elif name in ('/bebop/cmd_vel', '/bridge/cmd_vel_raw', '/goal_vel'):
            row = (t, m.linear.x, m.linear.y, m.linear.z, m.angular.z)
        elif name in ('/flight_state', '/bebop/altitude', '/trajectory_stalled'):
            row = (t, float(m.data))
        else:
            continue
        data.setdefault(name, []).append(row)
    return {k: np.array(v) for k, v in data.items()}


def state_intervals(states):
    """Lista de (t_ini, t_fin, estado)."""
    out = []
    cur = None
    t_ini = 0.0
    for t, s in states:
        s = int(s)
        if s != cur:
            if cur is not None:
                out.append((t_ini, t, cur))
            cur, t_ini = s, t
    if cur is not None:
        out.append((t_ini, states[-1, 0], cur))
    return out


def zoh(series, tg, col):
    idx = np.searchsorted(series[:, 0], tg, side='right') - 1
    return series[np.clip(idx, 0, len(series) - 1), col]


def identify(odom, cmd, t_a, t_b, ts=0.2):
    """Ajuste por mínimos cuadrados de dv/dt = a·u(t-d) - b·v para pitch."""
    tg = np.arange(t_a, t_b, ts)
    if len(tg) < 20:
        return None
    v = zoh(odom, tg, 5)   # vx en marco cuerpo
    u = zoh(cmd, tg, 1)    # pitch enviado al driver
    best = None
    for k in range(0, 8):
        y = (v[k + 1:] - v[k:-1]) / ts
        x = np.column_stack([u[:len(v) - k - 1], v[k:-1]])
        coef = np.linalg.lstsq(x, y, rcond=None)[0]
        pred = x @ coef
        r2 = 1.0 - ((y - pred) ** 2).sum() / max(((y - y.mean()) ** 2).sum(), 1e-9)
        if best is None or r2 > best[3]:
            best = (k * ts, coef[0], -coef[1], r2)
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='Carpeta del bag o archivo .db3')
    ap.add_argument('--approach', type=float, default=20.0,
                    help='s de AUTOMATIC usados para medir el sobrepaso de aproximación')
    args = ap.parse_args()

    d = load(args.bag)
    if '/flight_state' not in d or '/bebop1/pose' not in d:
        print('El bag no tiene /flight_state o /bebop1/pose.')
        return

    print('== Estados')
    ivs = state_intervals(d['/flight_state'])
    for a, b, s in ivs:
        print(f'  {a:6.1f}–{b:6.1f} s  {STATES.get(s, s)}')

    pose = d['/bebop1/pose']
    autos = [(a, b) for a, b, s in ivs if s == AUTOMATIC]
    if autos and '/goal' in d:
        print('\n== AUTOMATIC')
        goal = d['/goal']
        for a, b in autos:
            m = (pose[:, 0] >= a) & (pose[:, 0] <= b)
            tg = pose[m, 0]
            gx, gy = zoh(goal, tg, 1), zoh(goal, tg, 2)
            err = np.hypot(pose[m, 1] - gx, pose[m, 2] - gy)
            print(f'  tramo {a:.1f}–{b:.1f} s: error XY máx {err.max():.2f} m, '
                  f'RMS {math.sqrt(np.mean(err ** 2)):.2f} m')
            ma = m & (pose[:, 0] <= a + args.approach)
            if ma.any():
                gfin = zoh(goal, np.array([a + args.approach]), 1)[0]
                over = (pose[ma, 1] - gfin).max()
                print(f'  aproximación ({args.approach:.0f} s): x máx − goal final = '
                      f'{over:+.2f} m (objetivo < 0.10)')
            if '/bebop/cmd_vel' in d:
                c = d['/bebop/cmd_vel']
                mc = (c[:, 0] >= a) & (c[:, 0] <= b)
                gaz = int((np.abs(c[mc, 3]) > 1e-3).sum())
                print(f'  mensajes con gaz != 0: {gaz} (con z_mode native debe ser 0)')
                print(f'  |yaw| máx enviado: {np.abs(c[mc, 4]).max():.2f}'
                      ' (cerca del límite = órbita saturada)')
            mg = (goal[:, 0] >= a) & (goal[:, 0] <= b)
            if mg.sum() > 2:
                g = goal[mg]
                dt = np.diff(g[:, 0])
                v = np.hypot(np.diff(g[:, 1]), np.diff(g[:, 2])) / np.maximum(dt, 1e-3)
                w = np.abs(np.diff(np.unwrap(g[:, 4]))) / np.maximum(dt, 1e-3)
                still = dt[(v < 0.005) & (w < 0.005)].sum()
                print(f'  goal detenido: {still:.1f} s de {b - a:.1f} s '
                      '(incluye el settling; mucho más = órbita frenada por deriva)')
            if '/trajectory_stalled' in d:
                ts = d['/trajectory_stalled']
                n = int(((ts[:, 0] >= a) & (ts[:, 0] <= b + 0.5)).sum())
                if n:
                    print(f'  órbita atascada → HOVER: {n} vez/veces')

    print('\n== Altura')
    if '/bebop/odom' in d:
        oz = d['/bebop/odom'][:, 3]
        print(f'  z integrada del driver: máx {oz.max() - oz[0]:.2f} m | final '
              f'{oz[-1] - oz[0]:+.2f} m (en el piso debería ser ~0: lo demás es deriva)')
    if '/bebop/altitude' in d:
        alt = d['/bebop/altitude']
        print(f'  /bebop/altitude (medida): máx {alt[:, 1].max():.2f} m | '
              f'final {alt[-1, 1]:+.2f} m')
    else:
        print('  Sin /bebop/altitude: driver sin el parche de altura (patches/)')
    print(f'  z de /bebop1/pose: máx {pose[:, 3].max():.2f} m | final {pose[-1, 3]:+.2f} m')

    print('\n== Posición final (odometría)')
    print(f'  x {pose[-1, 1]:+.2f} m, y {pose[-1, 2]:+.2f} m, distancia al despegue '
          f'{math.hypot(pose[-1, 1], pose[-1, 2]):.2f} m')
    print('  Si aterrizó en el punto de despegue, esto es el error acumulado de la odometría.')

    if autos and '/odom_local' in d and '/bebop/cmd_vel' in d:
        a, b = max(autos, key=lambda iv: iv[1] - iv[0])
        res = identify(d['/odom_local'], d['/bebop/cmd_vel'], a, b)
        if res:
            dly, ga, gb, r2 = res
            vss = ga / gb if gb > 1e-3 else float('inf')
            print('\n== Modelo pitch → velocidad (tramo AUTOMATIC más largo)')
            print(f'  retardo {dly:.1f} s | a {ga:.2f} m/s² por unidad | b {gb:.2f} 1/s | '
                  f'v régimen {vss:.2f} m/s por unidad | R² {r2:.2f}')
            print(f'  feedforward de régimen sugerido ≈ b/a = {gb / ga:.2f}'
                  if ga > 1e-3 else '')


if __name__ == '__main__':
    main()
