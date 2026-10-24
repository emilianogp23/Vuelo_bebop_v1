"""Pruebas sin ROS de la localización visual (PnP sintético y fusión)."""
import math
import os

import cv2
import numpy as np
import pytest

from control_bebop.vision import (OdomHistory, OffsetFilter, VisionFusion, camera_in_world,
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
    name, corners = load_marker_map(MAP)
    assert name.startswith('DICT_')
    assert len([i for i in corners if i < 30]) == 8   # uno por tabla


@pytest.mark.parametrize('name', ['aruco_map.yaml', 'aruco_map_sim.yaml'])
def test_wall_markers_face_inside(name):
    # Cara hacia afuera = esquinas en espejo: el PnP falla o da la pose reflejada
    _, corners = load_marker_map(os.path.join(os.path.dirname(MAP), name))
    for i, c in corners.items():
        if i < 30:
            # derecha × abajo apunta hacia la valla, lejos de quien lo mira
            into_wall = np.cross(c[1] - c[0], c[3] - c[0])
            assert into_wall @ c.mean(axis=0) > 0, f'id {i} mira hacia afuera'


# ── A bordo ───────────────────────────────────────────────────
@pytest.mark.parametrize('pos,yaw', [((0.5, 0.3, 1.2), 0.5), ((-0.4, 0.8, 1.5), 2.5),
                                     ((1.0, -0.5, 1.2), -1.8)])
def test_onboard_pose_from_wall_markers(pos, yaw):
    _, corners = load_marker_map(MAP)
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


def test_offset_filter_rejects_outliers_without_jumping():
    # Una racha de mediciones malas (PnP espejo) no mueve la pose
    f = OffsetFilter()
    assert f.update((1.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.03, 0.03) == 'init'
    for _ in range(20):
        assert f.update((3.0, 0.0, 0.8), (1.0, 0.0, 0.0), 0.10, 0.03) == 'rejected'
    assert f.apply(1.0, 0.0, 0.0) == pytest.approx((1.0, 0.0, 0.0))


def test_offset_filter_weights_by_sigma():
    # La medición precisa pesa más que la ruidosa
    a, b = OffsetFilter(), OffsetFilter()
    for f in (a, b):
        f.update((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.05, 0.05)
    a.update((0.1, 0.0, 0.0), (0.0, 0.0, 0.0), 0.02, 0.05)
    b.update((0.1, 0.0, 0.0), (0.0, 0.0, 0.0), 0.20, 0.05)
    assert a.offset[0] > b.offset[0] > 0.0


def test_offset_filter_gate_opens_without_markers():
    # 0.4 m de error: recién corregido se rechaza; tras 60 s sin marcadores se acepta
    f = OffsetFilter()
    f.reset(0.05, 0.05)
    f.predict(0.0)
    assert f.update((0.4, 0.0, 0.0), (0.0, 0.0, 0.0), 0.03, 0.03) == 'rejected'
    f.predict(60.0)
    assert f.update((0.4, 0.0, 0.0), (0.0, 0.0, 0.0), 0.03, 0.03) == 'ok'
    assert f.offset[0] > 0.35


def test_offset_filter_gate_opens_with_odometry_motion():
    # 0.8 m de error tras 3 m de traslado en 10 s: solo con q_xy se rechaza; con q_move entra
    for q_move, esperado in ((0.0, 'rejected'), (0.02, 'ok')):
        v = VisionFusion(q_move=q_move)
        v.takeoff(reanchor=True)
        for i in range(11):
            v.odom(i, 0.3 * i, 0.0, 0.0)
        assert v.vision(10.0, (2.2, 0.0, 0.0), 0.03, 0.03) == esperado


def test_offset_filter_applies_correction_gradually():
    f = OffsetFilter(rate_xy=0.1)
    f.reset(0.05, 0.05)
    f.predict(0.0)
    f.predict(60.0)
    f.update((0.4, 0.0, 0.0), (0.0, 0.0, 0.0), 0.03, 0.03)
    assert f.apply(0.0, 0.0, 0.0)[0] == pytest.approx(0.0)   # sin escalón
    f.predict(61.0)
    assert f.apply(0.0, 0.0, 0.0)[0] == pytest.approx(0.1)
    f.predict(70.0)
    assert f.apply(0.0, 0.0, 0.0)[0] == pytest.approx(f.offset[0])


def test_fusion_starts_at_takeoff_and_loses_lock_without_markers():
    v = VisionFusion(max_sigma=0.20, init_sigma_xy=0.05, q_xy=0.0025)
    assert v.odom(0.0, 0.3, 0.0, 0.0) is None and not v.locked
    v.takeoff(reanchor=False)
    assert v.odom(0.1, 0.3, 0.0, 0.0) == pytest.approx((0.3, 0.0, 0.0))
    assert v.locked
    v.odom(14.0, 0.3, 0.0, 0.0)    # sqrt(0.0025 + 0.0025·14) ≈ 0.19 m
    assert v.locked
    v.odom(16.0, 0.3, 0.0, 0.0)
    assert not v.locked
    assert v.vision(16.0, (0.3, 0.0, 0.0), 0.03, 0.03) == 'ok'
    assert v.locked
    assert v.vision(30.0, (0.3, 0.0, 0.0), 0.03, 0.03) == 'sin_odom'


def test_fusion_takeoff_keeps_visual_offset_unless_reanchored():
    v = VisionFusion()
    v.odom(0.0, 0.0, 0.0, 0.0)
    assert v.vision(0.0, (0.2, 0.0, 0.0), 0.03, 0.03) == 'init'
    v.takeoff(reanchor=False)
    assert v.filter.offset[0] == pytest.approx(0.2)
    v.takeoff(reanchor=True)
    assert v.filter.offset[0] == pytest.approx(0.0)


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


def test_single_floor_marker_off_origin_plane():
    # Piso de la simulación (z 0.003): el IPPE de OpenCV fallaba si el plano no
    # pasa por el origen (≈50 px de error) y los ArUco del piso se descartaban
    c = marker_corners([0.5, 0.5, 0.003], 0.17, floor_yaw=0.0)
    pos, yaw = (1.4, 0.25, 1.0), math.radians(-172)
    rvec, tvec = camera_rt(pos, yaw, math.radians(-28))
    sol = solve_pnp(c, project(c, rvec, tvec, noise=0.3), K, D, level_camera=True)
    assert sol is not None and sol[2] < 2.0
    x, y, z, _ = onboard_drone_pose(*camera_in_world(sol[0], sol[1]))
    assert (x, y, z) == pytest.approx(pos, abs=0.05)


def test_fusion_rejects_measurement_with_wrong_height():
    # Un PnP espejo suele dar también una z imposible: la altura sí se conoce
    v = VisionFusion(max_dz=0.3)
    v.takeoff(reanchor=False)
    v.odom(0.0, 0.5, 0.0, 0.0)
    assert v.vision(0.0, (0.5, 0.0, 0.0), 0.03, 0.03, z_err=0.45) == 'rejected_z'
    assert v.vision(0.0, (0.5, 0.0, 0.0), 0.03, 0.03, z_err=0.10) == 'ok'
