"""Pruebas sin ROS de la geocerca, los marcos y el término D del PID."""
import math

import pytest

from control_bebop.frames import TakeoffFrame, rotate2d
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
    assert pitch == 0.0
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
