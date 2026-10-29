"""Pruebas del modo muestra: control manual y geometría del mapa 3D (sin ROS en marcha)."""
import math

import pytest
from geometry_msgs.msg import Twist

from control_bebop.demo_safety_manager import compute_nudge
from control_bebop.muestra_odometria_gui import body_to_world, project_point


def _twist(x=0.0, y=0.0, z=0.0, yaw=0.0):
    t = Twist()
    t.linear.x, t.linear.y, t.linear.z, t.angular.z = x, y, z, yaw
    return t


# ── Control manual ────────────────────────────────────────────
def test_nudge_full_stick_reaches_limit_on_every_axis():
    # Stick a fondo (0.5 en /joy_cmd) → exactamente el tope, incluida la altura
    cmd = compute_nudge(_twist(0.5, -0.5, 0.5, -0.5), 0.25, 0.5)
    assert (cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z) == (0.25, -0.25, 0.25, -0.25)


def test_nudge_is_proportional_below_full_scale():
    cmd = compute_nudge(_twist(x=0.25), 0.25, 0.5)
    assert cmd.linear.x == pytest.approx(0.125)


def test_nudge_never_exceeds_limit():
    cmd = compute_nudge(_twist(5.0, -5.0, 5.0, 5.0), 0.25, 0.5)
    assert max(abs(cmd.linear.x), abs(cmd.linear.y), abs(cmd.linear.z), abs(cmd.angular.z)) == 0.25


def test_nudge_zero_stays_zero():
    cmd = compute_nudge(_twist(), 0.25, 0.5)
    assert (cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z) == (0.0, 0.0, 0.0, 0.0)


# ── Geometría 3D ──────────────────────────────────────────────
def test_project_origin_is_center():
    assert project_point((0, 0, 0), math.radians(-60), math.radians(25), 80)[:2] == (0.0, 0.0)


def test_project_z_goes_up_on_screen():
    _, v, _ = project_point((0, 0, 1), math.radians(-60), math.radians(25), 100)
    assert v == pytest.approx(100 * math.cos(math.radians(25)))
    assert v > 0


def test_project_top_down_maps_forward_to_screen_up():
    # Vista desde arriba con az = -90°: x del dron hacia arriba, y hacia la izquierda
    az, el = math.radians(-90), math.radians(90)
    _, v_fwd, _ = project_point((1, 0, 0), az, el)
    u_left, _, _ = project_point((0, 1, 0), az, el)
    assert v_fwd == pytest.approx(1.0)
    assert u_left == pytest.approx(-1.0)


def test_project_depth_increases_away_from_camera():
    az, el = math.radians(-90), math.radians(25)    # x adelante = alejándose
    near = project_point((-1, 0, 0), az, el)[2]
    far = project_point((1, 0, 0), az, el)[2]
    assert far > near


def test_body_to_world_yaw_turns_nose_left():
    x, y, z = body_to_world((1, 0, 0), 0.0, 0.0, math.pi / 2, (2.0, 3.0, 1.0))
    assert (x, y, z) == pytest.approx((2.0, 4.0, 1.0))


def test_body_to_world_level_keeps_height():
    assert body_to_world((0.2, 0.2, 0), 0, 0, 0.7, (0, 0, 1.0))[2] == pytest.approx(1.0)


def test_body_to_world_positive_pitch_drops_the_nose():
    assert body_to_world((1, 0, 0), 0.0, math.radians(10), 0.0)[2] < 0


# ── YOLO ──────────────────────────────────────────────────────
def test_yolo_without_weights_reports_error_and_never_downloads(monkeypatch):
    import numpy as np
    from control_bebop import yolo_person_detector as ypd

    monkeypatch.setattr(ypd, 'find_weights', lambda _=None: None)
    det = ypd.YoloPersonDetector(autoload=False)
    assert det.status == ypd.STATUS_LOADING
    det.load()
    assert det.status == ypd.STATUS_ERROR
    assert 'yolov8n.pt' in det.error or 'ultralytics' in det.error
    assert not det.ready
    frame = np.zeros((48, 64, 3), dtype=np.uint8)
    out, dets = det.detect(frame)      # sin modelo devuelve el frame tal cual
    assert out is frame and dets == []


def test_yolo_packaged_weights_are_found():
    from control_bebop.yolo_person_detector import find_weights
    path = find_weights()
    assert path is not None and path.endswith('yolov8n.pt')
