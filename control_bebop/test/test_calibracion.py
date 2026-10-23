"""Pruebas sin ROS de la calibración ChArUco (imagen sintética y vistas proyectadas)."""
import math

import cv2
import numpy as np
import pytest
import yaml

from control_bebop.calibracion import (Charuco, calibrate, camera_info_yaml, coverage,
                                       is_new_view, view_features)

K = np.array([[537.29, 0, 427.33], [0, 527.0, 240.23], [0, 0, 1]])
SIZE = (856, 480)


def test_detects_every_corner_in_printed_board():
    ch = Charuco()
    img = ch.image(1400, 1000, margin_px=60)
    corners, ids = ch.detect(img)
    assert ids is not None and len(ids) == ch.n_corners == 6 * 4


def _views():
    """Esquinas del tablero proyectadas con K desde poses variadas e inclinadas."""
    ch = Charuco()
    center = ch.corners3d.mean(axis=0)
    rng = np.random.default_rng(1)
    views = []
    for _ in range(20):
        rvec = np.array([math.pi + rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6),
                         rng.uniform(-0.3, 0.3)])
        R, _ = cv2.Rodrigues(rvec)
        tvec = np.array([rng.uniform(-0.1, 0.1), rng.uniform(-0.05, 0.05),
                         rng.uniform(0.35, 0.6)]) - R @ center
        img, _ = cv2.projectPoints(ch.corners3d, rvec, tvec, K, None)
        img = img.reshape(-1, 2) + rng.normal(0, 0.2, (ch.n_corners, 2))
        inside = np.all((img >= 0) & (img < SIZE), axis=1)
        views.append((img[inside].astype(np.float32), np.flatnonzero(inside)))
    return ch, views


def test_calibrate_recovers_intrinsics():
    ch, views = _views()
    rms, k, d, errors = calibrate(ch, views, SIZE)
    assert rms < 0.5
    assert k[0, 0] == pytest.approx(K[0, 0], rel=0.01)
    assert k[1, 1] == pytest.approx(K[1, 1], rel=0.01)
    assert k[:2, 2] == pytest.approx(K[:2, 2], abs=3.0)
    assert len(errors) == len(views) and max(errors) < 1.0


def test_view_selection_asks_for_variety():
    ch, views = _views()
    f0 = view_features(ch, *views[0], SIZE)
    assert not is_new_view(f0, [f0])
    assert is_new_view(f0, [f0 + [0.3, 0, 0, 0, 0]])
    cov = coverage([view_features(ch, c, i, SIZE) for c, i in views])
    assert cov['tilt_max_deg'] > 20.0


def test_camera_info_yaml_matches_driver_format():
    text = camera_info_yaml(K, np.zeros(5), SIZE, comment='rms 0.3 px')
    data = yaml.safe_load(text)
    assert (data['image_width'], data['image_height']) == SIZE
    assert data['camera_name'] == 'bebop_front'
    assert data['camera_matrix']['data'][0] == pytest.approx(537.29)
    assert len(data['distortion_coefficients']['data']) == 5
    assert len(data['projection_matrix']['data']) == 12
