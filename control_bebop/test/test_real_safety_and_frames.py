"""Pruebas sin ROS de la geocerca, los marcos, la altura y el PID."""
import math
import os

import pytest

from control_bebop.frames import TakeoffFrame, rotate2d, select_altitude
from control_bebop.safety import GeofenceParams, apply_geofence


# ── Geocerca ──────────────────────────────────────────────────
def test_ceiling_forces_descent_when_z_enabled():
    p = GeofenceParams(max_altitude=2.0)
    (_, _, gaz, _), events = apply_geofence((0.0, 0.0, 0.3, 0.0), (0, 0, 1.95, 0), p)
    assert gaz < 0.0
    assert 'techo_forzar_bajar' in events


def test_z_disabled_leaves_gaz_untouched():
    # Con la z integrada a la deriva (p. ej. 1.95 m estando a 1 m real) no
    # debe forzarse a bajar ni bloquearse nada en altura.
    p = GeofenceParams(max_altitude=2.0, z_enabled=False)
    for z, gaz_in in ((1.95, 0.3), (1.80, 0.2), (0.10, -0.2), (1.0, 0.0)):
        (_, _, gaz, _), events = apply_geofence((0.0, 0.0, gaz_in, 0.0), (0, 0, z, 0), p)
        assert gaz == gaz_in
        assert not any(e.startswith(('techo', 'piso')) for e in events)


def test_z_disabled_keeps_xy_limits():
    p = GeofenceParams(half_size=2.0, z_enabled=False)
    (pitch, _, _, _), events = apply_geofence((0.3, 0.0, 0.0, 0.0), (1.6, 0, 1.0, 0), p)
    assert 0.0 <= pitch < 0.1
    assert 'bloqueo_x' in events
    (pitch, _, _, _), events = apply_geofence((0.3, 0.0, 0.0, 0.0), (1.8, 0, 1.0, 0), p)
    assert pitch < 0.0
    assert 'empuje_x' in events


# ── Marcos ────────────────────────────────────────────────────
def test_takeoff_frame_origin_and_yaw():
    f = TakeoffFrame()
    f.set_origin(1.0, 2.0, 0.0, math.pi / 2)
    # 1 m al "norte" del origen con yaw0 = 90°: en el marco de despegue es -y
    x, y, z, yaw = f.pose_to_local(2.0, 2.0, 0.5, math.pi / 2)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(-1.0)
    assert z == pytest.approx(0.5)
    assert yaw == pytest.approx(0.0)


def test_body_velocity_rotation_matches_takeoff_frame():
    # Avanzar a 0.2 m/s con el frente girado 90° = moverse en +y local
    wx, wy = rotate2d(0.2, 0.0, math.pi / 2)
    assert wx == pytest.approx(0.0, abs=1e-9)
    assert wy == pytest.approx(0.2)
    f = TakeoffFrame(scale=(0.8, 0.8, 1.0))
    bx, by, _ = f.body_vel_to_local(0.2, 0.0, 0.0, math.pi / 2)
    assert (bx, by) == pytest.approx((wx * 0.8, wy * 0.8), abs=1e-9)


# ── PID: término D ────────────────────────────────────────────
def _pid():
    mod = pytest.importorskip('control_bebop.control_PID')
    pid = mod.PID()
    pid.kp_pitch, pid.kd_pitch, pid.ki_pitch = 0.25, 0.60, 0.0
    return pid


def test_pid_d_uses_measured_velocity_error():
    pid = _pid()
    # Sin error de posición, yendo a 0.10 m/s cuando se pide 0.05 m/s → frenar
    cmd = pid.control_pitch(1.0, 1.0, 0.1, vel=0.10, vel_ref=0.05)
    assert cmd == pytest.approx(0.60 * (0.05 - 0.10))
    # Siguiendo la referencia exacta → sin término D
    cmd = pid.control_pitch(1.0, 1.0, 0.1, vel=0.05, vel_ref=0.05)
    assert cmd == pytest.approx(0.0)


def test_pid_fallback_differentiates_position():
    pid = _pid()
    pid.d_alpha = 0.0
    pid.control_pitch(0.0, 0.0, 0.1)          # primera muestra: D = 0
    cmd = pid.control_pitch(0.0, 0.01, 0.1)   # se movió 1 cm en 0.1 s
    assert cmd == pytest.approx(0.25 * -0.01 + 0.60 * -0.1)


def test_pid_integral_can_compensate_drift():
    # Vuelo del 16 oct: con ki 0.03 e int_limit 0.5 el integral saturaba en
    # 0.015 y no compensaba la deriva. Con real_drone.yaml debe llegar a ~0.10.
    yaml = pytest.importorskip('yaml')
    cfg_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'real_drone.yaml')
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)['/**']['ros__parameters']
    pid = _pid()
    pid.ki_pitch = cfg['ki_xy']
    pid.int_limit_xy = cfg['int_limit_xy']
    for _ in range(300):                      # 30 s con 0.3 m de error
        cmd = pid.control_pitch(0.3, 0.0, 0.1, vel=0.0, vel_ref=0.0)
    assert cmd - 0.25 * 0.3 == pytest.approx(cfg['ki_xy'] * cfg['int_limit_xy'])
    assert cfg['ki_xy'] * cfg['int_limit_xy'] >= 0.09


# ── Altura: /bebop/altitude contra z integrada ────────────────
def test_select_altitude_uses_recent_measurement():
    z, measured = select_altitude(1.6, 1.02, 0.2, 1.0)
    assert (z, measured) == (pytest.approx(1.02), True)


def test_select_altitude_falls_back_when_stale():
    z, measured = select_altitude(1.6, 1.02, 1.5, 1.0)
    assert (z, measured) == (pytest.approx(1.6), False)


def test_select_altitude_falls_back_without_measurement():
    assert select_altitude(0.7, None, None, 1.0) == (pytest.approx(0.7), False)



# ── Zona segura (robots en el piso) ───────────────────────────
def test_low_outside_safe_zone_climbs_and_blocks_outward():
    p = GeofenceParams(safe_zone_radius=0.5, min_z_outside_safe=1.0)
    # En x=0.6 a 0.8 m de altura, mirando a +x y avanzando hacia afuera
    (pitch, roll, gaz, _), events = apply_geofence((0.2, 0.1, -0.1, 0.0), (0.6, 0, 0.8, 0), p)
    assert pitch == pytest.approx(0.0, abs=1e-9)
    assert roll == pytest.approx(0.1)          # tangencial: permitido
    assert gaz > 0.0
    assert {'zona_segura_bloqueo', 'zona_segura_subir'} <= set(events)
    # Hacia adentro sí se deja
    (pitch, _, _, _), _ = apply_geofence((-0.2, 0.0, 0.0, 0.0), (0.6, 0, 0.8, 0), p)
    assert pitch == pytest.approx(-0.2)


def test_safe_zone_allows_low_inside_and_high_outside():
    p = GeofenceParams(safe_zone_radius=0.5, min_z_outside_safe=1.0)
    for pose in ((0.2, 0.0, 0.3, 0.0), (1.0, 0.0, 1.2, 0.0)):
        cmd, events = apply_geofence((0.2, 0.0, -0.1, 0.0), pose, p)
        assert cmd[0] == pytest.approx(0.2)
        assert not any(e.startswith('zona_segura') for e in events)


# ── Geocerca con velocidad y CommandGate ──────────────────────
def test_geofence_brakes_early_when_moving_out_fast():
    p = GeofenceParams()
    # En x=1.2 a 0.3 m/s hacia afuera: permitido 0.5/2.5 = 0.2 m/s → frena
    (pitch, _, _, _), events = apply_geofence((0.2, 0.0, 0.0, 0.0), (1.2, 0, 1.2, 0), p,
                                              vel=(0.3, 0.0))
    assert pitch < 0.0 and 'bloqueo_x' in events
    # Sin velocidad, en el mismo punto, no se toca el comando
    (pitch, _, _, _), events = apply_geofence((0.2, 0.0, 0.0, 0.0), (1.2, 0, 1.2, 0), p)
    assert pitch == pytest.approx(0.2) and not events


def test_geofence_lets_orbit_pass_near_the_edge():
    # Órbita de r 0.5 en (1, 0): llega a x = 1.5 casi sin velocidad radial
    p = GeofenceParams()
    (pitch, _, _, _), _ = apply_geofence((0.08, 0.0, 0.0, 0.0), (1.5, 0, 1.1, 0), p,
                                         vel=(0.0, 0.05))
    assert pitch == pytest.approx(0.08)


def test_geofence_push_grows_with_depth_and_is_capped():
    p = GeofenceParams()
    pushes = [-apply_geofence((0.0,) * 4, (x, 0, 1.2, 0), p)[0][0] for x in (1.75, 2.0, 3.0)]
    assert pushes[0] < pushes[1] < pushes[2] == pytest.approx(p.max_push)


def test_geofence_lets_drone_come_back_inside():
    p = GeofenceParams()
    (pitch, _, _, _), _ = apply_geofence((-0.2, 0.0, 0.0, 0.0), (1.6, 0, 1.2, 0), p,
                                         vel=(-0.1, 0.0))
    assert pitch == pytest.approx(-0.2)


def test_geofence_stops_identified_bebop_models():
    # dv/dt = a·u(t−d) − b·v, rangos de los bags del 13–16 oct; piloto empujando afuera
    import itertools
    p = GeofenceParams()
    dt, worst = 0.05, 0.0
    for a, b, d in itertools.product((0.8, 1.9), (0.25, 1.1), (0.2, 0.9)):
        for v0, pilot in ((0.3, 0.4), (0.5, 0.4), (0.6, 0.14), (0.3, 0.0)):
            x, v, queue = 1.0, v0, [0.0] * int(round(d / dt) + 1)
            for _ in range(int(30 / dt)):
                u = apply_geofence((pilot, 0.0, 0.0, 0.0), (x, 0.0, 1.2, 0.0), p, (v, 0.0))[0][0]
                queue.append(u)
                v += (a * queue.pop(0) - b * v) * dt
                x += v * dt
                worst = max(worst, x)
    assert worst < p.half_size + 0.05   # la valla está a ~2.45 m


def test_command_gate_fences_hover_only_in_the_air():
    from control_bebop.safety import CommandGate
    gate = CommandGate(GeofenceParams(), timeout=0.3)
    edge = (1.8, 0.0, 1.2, 0.0)
    # En tierra: ceros sin geocerca (no se mandan comandos a un dron apagado)
    assert gate.output(10.0, edge, (0.0, 0.0)) == ((0.0, 0.0, 0.0, 0.0), [])
    gate.takeoff()
    # Sin comando reciente (watchdog) la geocerca empuja igual hacia dentro
    (pitch, _, _, _), events = gate.output(10.0, edge, (0.0, 0.0))
    assert pitch < 0.0 and 'empuje_x' in events
    gate.command((0.3, 0.0, 0.0, 0.0), 10.0)
    assert gate.output(10.1, edge, (0.0, 0.0))[0][0] < 0.0
    # Aterrizando fuera de la zona segura no se fuerza a subir
    gate.land()
    assert gate.output(10.2, (1.0, 0.0, 0.5, 0.0), (0.0, 0.0))[0][2] == 0.0


def test_command_gate_ignores_min_altitude_without_measured_z():
    from control_bebop.safety import CommandGate
    gate = CommandGate(GeofenceParams(min_z_outside_safe=1.0))
    gate.takeoff()
    low_outside = (1.0, 0.0, 0.5, 0.0)
    assert gate.output(0.0, low_outside, (0.0, 0.0), z_measured=True)[0][2] > 0.0
    assert gate.output(0.0, low_outside, (0.0, 0.0), z_measured=False)[0][2] == 0.0
