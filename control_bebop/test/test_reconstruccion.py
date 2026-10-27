"""Pruebas de control_bebop.reconstruccion que no necesitan COLMAP ni OpenCV."""
import numpy as np

from control_bebop import reconstruccion as rec


def test_select_frames_one_per_window_sharpest():
    scores = [1, 5, 2, 9, 3, 4, 8, 7]
    # 4 ventanas de 2 frames: se toma el más nítido de cada una
    assert rec.select_frames(scores, 4, blur_ratio=0) == [1, 3, 5, 6]


def test_select_frames_drops_blurry_windows():
    scores = [100, 90, 2, 3, 110, 95]
    # mediana 92.5; la ventana del medio (máx 3) queda bajo 0.35·mediana
    assert rec.select_frames(scores, 3) == [0, 4]


def test_select_frames_more_targets_than_frames():
    assert rec.select_frames([5, 6, 7], 10, blur_ratio=0) == [0, 1, 2]
    assert rec.select_frames([], 10) == []


def test_select_frames_spread_over_whole_video():
    keep = rec.select_frames([1.0] * 1000, 10, blur_ratio=0)
    assert len(keep) == 10
    assert keep[0] < 100 and keep[-1] >= 900


def test_orbit_pairs_close_the_loop():
    names = [f'f{i:02d}.png' for i in range(20)]
    pairs = rec.orbit_pairs(names, overlap=3)
    assert ('f00.png', 'f01.png') in pairs
    assert ('f00.png', 'f19.png') in pairs      # cierre del lazo
    assert ('f00.png', 'f06.png') in pairs       # 2·overlap
    assert len(pairs) == len(set(pairs))
    assert all(a < b for a, b in pairs)
    # mucho menos que todos contra todos (190)
    assert len(pairs) < 120


def test_count_model(tmp_path):
    (tmp_path / 'images.txt').write_text(
        '# comentario\n'
        '1 1 0 0 0 0 0 0 1 a.png\n10 20 -1\n'
        '2 1 0 0 0 0 0 0 1 b.png\n\n')
    (tmp_path / 'points3D.txt').write_text('# c\n1 0 0 0 1 2 3 0.1\n2 0 0 0 1 2 3 0.1\n')
    assert rec.count_model(str(tmp_path)) == (2, 2)


def test_estimate_normals_plane_oriented_to_camera():
    import numpy as np
    rng = np.random.default_rng(0)
    pts = np.c_[rng.random((400, 2)), np.zeros(400)]   # plano z = 0
    view = np.tile([0.3, 0.0, 1.0], (400, 1))           # cámaras arriba
    normals = rec.estimate_normals(pts, view, radius=0.1)
    assert np.all(normals[:, 2] > 0.99)


def test_sparse_normals_ply(tmp_path):
    (tmp_path / 'images.txt').write_text(
        '1 1 0 0 0 0 0 5 1 a.png\n\n2 1 0 0 0 1 0 5 1 b.png\n\n')
    (tmp_path / 'points3D.txt').write_text(
        ''.join(f'{i} {i * 0.01} 0 0 10 20 30 0.1 1 0 2 0\n' for i in range(30)))
    out = tmp_path / 'n.ply'
    assert rec.sparse_normals_ply(str(tmp_path), str(out)) == 30
    text = out.read_text().splitlines()
    assert 'element vertex 30' in text and len(text) == 13 + 30


def test_select_frames_by_angle_even_sectors_despite_pause():
    # 100 frames: la órbita se pausa en 0° (50 frames) y el resto recorre el círculo
    ang = np.concatenate([np.zeros(50), np.linspace(0, 2 * np.pi, 50, endpoint=False)])
    scores = np.ones(100)
    keep = rec.select_frames_by_angle(scores, ang, 10, blur_ratio=0)
    sectors = {int(np.floor(ang[i] / (2 * np.pi) * 10)) for i in keep}
    assert len(keep) == 10 and sectors == set(range(10))


def test_select_frames_by_angle_skips_nan_and_empty_sectors():
    ang = np.array([0.1, 0.2, np.nan, np.nan, 3.2])
    keep = rec.select_frames_by_angle([1, 5, 9, 9, 2], ang, 4, blur_ratio=0)
    assert keep == [1, 4]


def test_fit_circle_ignores_straight_approach():
    th = np.linspace(0, 2 * np.pi, 80, endpoint=False)
    x = np.concatenate([np.linspace(0, 0.1, 15), 0.7 + 0.8 * np.cos(th)])
    y = np.concatenate([np.zeros(15), -0.1 + 0.8 * np.sin(th)])
    cx, cy, r = rec.fit_circle(x, y)
    assert abs(cx - 0.7) < 0.03 and abs(cy + 0.1) < 0.03 and abs(r - 0.8) < 0.03
