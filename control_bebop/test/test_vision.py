"""Pruebas sin ROS de la localización visual (PnP sintético y fusión)."""
import math
import os

import cv2
import numpy as np
import pytest

from control_bebop.vision import (OdomHistory, OffsetFilter, camera_in_world,
                                  ceiling_drone_pose, drone_marker_corners_local,
                                  load_marker_map, marker_corners,
                                  onboard_drone_pose, solve_pnp)

# Intrínsecos del Bebop (ros2_bebop_driver/config/bebop2_camera_calib.yaml)
K = np.array([[537.29, 0, 427.33], [0, 527.0, 240.23], [0, 0, 1]])
D = np.zeros(5)
MAP = os.path.join(os.path.dirname(__file__), '..', 'config', 'aruco_map.yaml')


def camera_rt(pos, yaw, tilt):
    """rvec, tvec (mundo → cámara) de una cámara sin roll."""
    f = np.array([math.cos(tilt) * math.cos(yaw), math.cos(tilt) * math.sin(yaw), math.sin(tilt)])
    r = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
    d = np.cross(f, r)
    R_cw = np.column_stack([r, d, f]).T
    rvec, _ = cv2.Rodrigues(R_cw)
    return rvec, -R_cw @ np.asarray(pos, dtype=float)


def project(points, rvec, tvec, k=K, noise=0.0, seed=0):
    img, _ = cv2.projectPoints(np.asarray(points, dtype=float), rvec, tvec, k, D)
    img = img.reshape(-1, 2)
    return img + np.random.default_rng(seed).normal(0, noise, img.shape)


def visible(corners, rvec, tvec, w=856, h=480):
    R, _ = cv2.Rodrigues(rvec)
    pc = (R @ corners.T).T + tvec
    img = project(corners, rvec, tvec)
    return bool(np.all(pc[:, 2] > 0.2) and np.all((img >= 0) & (img < [w, h])))


# ── Mapa ──────────────────────────────────────────────────────
def test_wall_marker_faces_viewer():
    # Marcador en la valla x=+2 mirando hacia -x: visto desde el origen, TL
    # queda arriba a la izquierda (y positiva = izquierda del observador)
    c = marker_corners([2.0, 0.0, 0.35], 0.25, normal_yaw=math.pi)
    tl, tr, br, bl = c
    assert tl[1] > tr[1] and tl[2] > bl[2]
    assert np.linalg.norm(tr - tl) == pytest.approx(0.25)


def test_map_file_loads_with_unique_ids():
    name, corners, drone = load_marker_map(MAP)
    assert name.startswith('DICT_')
    assert len([i for i in corners if i < 30]) == 8   # uno por tabla
    assert drone['id'] not in corners


# ── A bordo ───────────────────────────────────────────────────
@pytest.mark.parametrize('pos,yaw', [((0.5, 0.3, 1.2), 0.5), ((-0.4, 0.8, 1.5), 2.5),
                                     ((1.0, -0.5, 1.2), -1.8)])
def test_onboard_pose_from_wall_markers(pos, yaw):
    _, corners, _ = load_marker_map(MAP)
    tilt = math.radians(-28)
    rvec, tvec = camera_rt(pos, yaw, tilt)
    seen = [c for c in corners.values() if visible(c, rvec, tvec)]
    assert len(seen) >= 1
    obj = np.vstack(seen)
    sol = solve_pnp(obj, project(obj, rvec, tvec, noise=0.5), K, D)
    assert sol is not None
    R_wc, cam = camera_in_world(sol[0], sol[1])
    x, y, z, est_yaw = onboard_drone_pose(R_wc, cam, cam_offset_x=0.0)
    assert (x, y, z) == pytest.approx(pos, abs=0.05)
    assert math.degrees(abs(math.remainder(est_yaw - yaw, 2 * math.pi))) < 2.0


def test_onboard_offset_moves_to_drone_center():
    R_wc = np.column_stack([[0, -1, 0], [0, 0, -1], [1, 0, 0]]).astype(float)
    x, y, _, yaw = onboard_drone_pose(R_wc, np.array([1.1, 0.0, 1.0]), cam_offset_x=0.1)
    assert (x, y, yaw) == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)


# ── Techo ─────────────────────────────────────────────────────
def test_ceiling_pose_of_drone_marker():
    cam_pos = (0.2, -0.1, 2.74)
    # Cámara mirando hacia abajo
    f, r = np.array([0, 0, -1.0]), np.array([0, -1.0, 0])
    R_wc = np.column_stack([r, np.cross(f, r), f])
    K_c = np.array([[930.0, 0, 640], [0, 930.0, 360], [0, 0, 1]])
    drone = (0.6, 0.3, 1.3, math.radians(40))
    local = drone_marker_corners_local(0.10)
    c, s = math.cos(drone[3]), math.sin(drone[3])
    # Borde superior del marcador (+y local) hacia el frente del dron
    R_wm = np.array([[s, c, 0], [-c, s, 0], [0, 0, 1.0]])
    world = (R_wm @ local.T).T + drone[:3]
    rvec, _ = cv2.Rodrigues(R_wc.T)
    tvec = -R_wc.T @ np.array(cam_pos)
    img = project(world, rvec, tvec, k=K_c, noise=0.3)
    (x, y, z, yaw), err, dist = ceiling_drone_pose(img, 0.10, K_c, D, R_wc, np.array(cam_pos))
    assert (x, y, z) == pytest.approx(drone[:3], abs=0.03)
    assert math.degrees(abs(yaw - drone[3])) < 2.0
    assert dist == pytest.approx(1.55, abs=0.05)


# ── Fusión ────────────────────────────────────────────────────
def test_odom_history_interpolates():
    h = OdomHistory()
    h.add(0.0, 0.0, 0.0, 3.0)
    h.add(1.0, 1.0, 2.0, -3.0)
    x, y, yaw = h.at(0.5)
    assert (x, y) == pytest.approx((0.5, 1.0))
    assert abs(abs(yaw) - math.pi) < 0.02   # interpola por el lado corto
    assert h.at(5.0) is None


def test_offset_filter_corrects_drift():
    f = OffsetFilter()
    true_offset = (0.4, -0.3, math.radians(10))
    for i in range(50):
        t = i * 0.1
        odom = (0.5 * math.cos(t), 0.5 * math.sin(t), t)
        rx, ry = (odom[0] * math.cos(true_offset[2]) - odom[1] * math.sin(true_offset[2]),
                  odom[0] * math.sin(true_offset[2]) + odom[1] * math.cos(true_offset[2]))
        meas = (rx + true_offset[0], ry + true_offset[1], odom[2] + true_offset[2])
        f.predict(t)
        f.update(meas, odom, 0.03, math.radians(2))
    assert f.offset[:2] == pytest.approx(true_offset[:2], abs=0.01)
    assert f.offset[2] == pytest.approx(true_offset[2], abs=0.01)


def test_offset_filter_rejects_outlier_then_relocalizes():
    f = OffsetFilter(max_rejects=3)
    assert f.update((1.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.03, 0.03) == 'init'
    assert f.update((2.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.03, 0.03) == 'rejected'
    assert f.offset[0] == pytest.approx(0.0)
    assert f.update((2.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.03, 0.03) == 'rejected'
    assert f.update((2.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.03, 0.03) == 'reinit'
    assert f.apply(1.0, 0.0, 0.0)[0] == pytest.approx(2.0)


def test_offset_filter_weights_by_sigma():
    # La medición precisa pesa más que la ruidosa
    a, b = OffsetFilter(), OffsetFilter()
    for f in (a, b):
        f.update((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.05, 0.05)
    a.update((0.2, 0.0, 0.0), (0.0, 0.0, 0.0), 0.02, 0.05)
    b.update((0.2, 0.0, 0.0), (0.0, 0.0, 0.0), 0.20, 0.05)
    assert a.offset[0] > b.offset[0] > 0.0


def test_detector_finds_generated_marker():
    from control_bebop.vision import detect, make_detector
    draw = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    img = np.full((300, 300), 255, np.uint8)
    img[50:250, 50:250] = draw(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 7, 200)
    found = detect(make_detector('DICT_4X4_50'), img)
    assert list(found) == [7]
    assert found[7][0] == pytest.approx((50, 50), abs=1.0)


def test_single_marker_ambiguity_resolved_by_level_camera():
    # Un solo marcador visto de frente y algo de lado: IPPE da dos soluciones
    c = marker_corners([2.0, 1.0, 0.30], 0.17, normal_yaw=math.pi)
    for pos, yaw in (((0.5, 0.6, 1.3), 0.25), ((-0.5, -0.2, 1.2), 0.5)):
        rvec, tvec = camera_rt(pos, yaw, math.radians(-28))
        sol = solve_pnp(c, project(c, rvec, tvec, noise=0.3), K, D, level_camera=True)
        assert sol is not None
        R_wc, cam = camera_in_world(sol[0], sol[1])
        x, y, z, est_yaw = onboard_drone_pose(R_wc, cam)
        assert (x, y, z) == pytest.approx(pos, abs=0.10)
        assert math.degrees(abs(est_yaw - yaw)) < 3.0
