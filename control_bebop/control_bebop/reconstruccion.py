"""
Reconstrucción 3D (fotogrametría) a partir del video de la órbita.

Dos etapas, sin ROS (sirve igual en la laptop y en el escritorio):

1. Selección de frames. De un AVI (nodo_video) o de una carpeta de imágenes
   se eligen N frames repartidos a lo largo del vuelo. El video se parte en N
   ventanas del mismo número de frames y en cada una se toma el más nítido
   (varianza del Laplaciano). Las ventanas cuyo mejor frame sigue borroso se
   descartan. Se reparte por índice de frame y no por tiempo: el AVI se graba
   con un fps nominal (15) que no coincide con el de la cámara (~30 Hz), así
   que su duración no es confiable.

2. COLMAP (línea de comandos). Extracción SIFT, emparejamiento de pares de
   la órbita (vecinos en el tiempo, cerrando el lazo), mapper (nube dispersa
   + poses de cámara). Después:
     - sin CUDA (paquete apt de Ubuntu): normales por PCA sobre la nube
       dispersa y malla Poisson (poisson_mesher corre en CPU);
     - con CUDA: además nube densa (patch_match_stereo + stereo_fusion) y
       malla Poisson densa.

La escala de la reconstrucción es arbitraria (fotogrametría monocular).

El módulo no importa rclpy. Ver scripts/reconstruccion_3d.py y
docs en RECONSTRUCCION_3D.md.
"""

import glob
import os
import re
import shutil
import subprocess

import numpy as np

IMAGE_EXTS = ('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff')
VIDEO_EXTS = ('.avi', '.mp4', '.mkv', '.mov')


# ── Selección de frames ──────────────────────────────────────────────

def sharpness(gray):
    """Varianza del Laplaciano: mayor = más nítido."""
    import cv2
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def select_frames(scores, n_target, blur_ratio=0.35):
    """Índices de frames a conservar.

    scores: nitidez de cada frame, en orden.
    n_target: número de ventanas (frames a devolver como máximo).
    blur_ratio: una ventana se descarta si su mejor frame tiene menos de
        blur_ratio · mediana(scores). 0 desactiva el filtro.
    """
    scores = np.asarray(scores, dtype=float)
    n = len(scores)
    if n == 0 or n_target <= 0:
        return []
    if n_target >= n:
        windows = [np.array([i]) for i in range(n)]
    else:
        edges = np.linspace(0, n, n_target + 1).round().astype(int)
        windows = [np.arange(a, b) for a, b in zip(edges[:-1], edges[1:]) if b > a]
    threshold = blur_ratio * float(np.median(scores)) if blur_ratio > 0 else -np.inf
    keep = []
    for w in windows:
        best = int(w[np.argmax(scores[w])])
        if scores[best] >= threshold:
            keep.append(best)
    return keep


def select_frames_by_angle(scores, angles, n_target, blur_ratio=0.35):
    """Índices de frames repartidos por ángulo alrededor del objeto.

    angles: ángulo (rad) de cada frame respecto al objeto; NaN = frame que no
        sirve (fuera de la órbita). El círculo se parte en n_target sectores
        iguales y de cada uno se toma el frame más nítido, de modo que una
        pausa de la órbita no deja un hueco ni amontona frames en un lado.
        Los sectores sin frames se omiten. blur_ratio como en select_frames.
    """
    scores = np.asarray(scores, dtype=float)
    angles = np.asarray(angles, dtype=float)
    ok = np.isfinite(angles)
    if n_target <= 0 or not ok.any():
        return []
    sector = np.floor(np.mod(np.nan_to_num(angles), 2 * np.pi) / (2 * np.pi) * n_target).astype(int)
    threshold = blur_ratio * float(np.median(scores[ok])) if blur_ratio > 0 else -np.inf
    keep = []
    for k in range(n_target):
        idx = np.flatnonzero(ok & (sector == k))
        if len(idx) == 0:
            continue
        best = int(idx[np.argmax(scores[idx])])
        if scores[best] >= threshold:
            keep.append(best)
    return sorted(keep)


def fit_circle(x, y, iters=3, tol=0.25):
    """Centro (cx, cy) y radio del círculo que mejor ajusta los puntos.

    Ajuste algebraico (Kåsa) descartando de forma iterativa los puntos a más
    de tol·R del círculo (la aproximación en línea recta antes de la órbita).
    """
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    keep = np.ones(len(x), dtype=bool)
    cx = cy = r = 0.0
    for _ in range(iters):
        if keep.sum() < 3:
            break
        a = np.column_stack([2 * x[keep], 2 * y[keep], np.ones(keep.sum())])
        b = x[keep] ** 2 + y[keep] ** 2
        (cx, cy, c), *_ = np.linalg.lstsq(a, b, rcond=None)
        r = float(np.sqrt(max(c + cx ** 2 + cy ** 2, 0.0)))
        keep = np.abs(np.hypot(x - cx, y - cy) - r) <= tol * r
    return float(cx), float(cy), r


def bag_frame_angles(bag, n_frames, center=None, offset=0.0, log=print):
    """Ángulo alrededor del objeto de cada frame del video, según el bag.

    El video (nodo_video) empieza con el primer estado de vuelo y termina al
    dejar de volar; el bag trae ese estado en /flight_state, así que el frame
    i corresponde a t = t_inicio + offset + i·duración/n_frames. Solo cuentan
    los frames en AUTOMATIC (estado 1) con la pose en /bebop1/pose dentro de
    ±50 % del radio de la órbita; el resto devuelve NaN. center = (x, y) del
    objeto; None = centro del círculo ajustado a la pose en AUTOMATIC.
    Requiere ROS 2 (lee el bag con rosidl).
    """
    import sqlite3

    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    if os.path.isdir(bag):
        dbs = sorted(glob.glob(os.path.join(bag, '*.db3')))
        if not dbs:
            raise RuntimeError(f'No hay .db3 en {bag}')
        bag = dbs[0]
    con = sqlite3.connect(bag)
    topics = {r[1]: (r[0], get_message(r[2])) for r in
              con.execute('SELECT id, name, type FROM topics')}
    for needed in ('/flight_state', '/bebop1/pose'):
        if needed not in topics:
            raise RuntimeError(f'El bag no tiene {needed}')

    def series(name):
        tid, mtype = topics[name]
        return [(ts * 1e-9, deserialize_message(raw, mtype)) for ts, raw in
                con.execute('SELECT timestamp, data FROM messages WHERE topic_id=? '
                            'ORDER BY timestamp', (tid,))]

    states = [(t, m.data) for t, m in series('/flight_state')]
    poses = [(t, m.position.x, m.position.y) for t, m in series('/bebop1/pose')]
    recording = {1, 2, 5, 6, 7, 8}   # igual que nodo_video.RECORDING_STATES
    t_start = next((t for t, st in states if st in recording), None)
    if t_start is None:
        raise RuntimeError('El bag no tiene estados de vuelo')
    t_end = next((t for t, st in states if t > t_start and st not in recording),
                 states[-1][0])
    pose = np.array(poses)
    st_t = np.array([t for t, _ in states])
    st_v = np.array([v for _, v in states])

    times = t_start + offset + (t_end - t_start) * np.arange(n_frames) / n_frames
    x = np.interp(times, pose[:, 0], pose[:, 1])
    y = np.interp(times, pose[:, 0], pose[:, 2])
    state = st_v[np.clip(np.searchsorted(st_t, times, side='right') - 1, 0, len(st_v) - 1)]
    auto = (state == 1) & (times >= pose[0, 0]) & (times <= pose[-1, 0])
    if auto.sum() < 10:
        raise RuntimeError('Menos de 10 frames en AUTOMATIC: no hay órbita que repartir')
    if center is None:
        cx, cy, r = fit_circle(x[auto], y[auto])
    else:
        cx, cy = center
        r = float(np.median(np.hypot(x[auto] - cx, y[auto] - cy)))
    dist = np.hypot(x - cx, y - cy)
    use = auto & (np.abs(dist - r) <= 0.5 * r)
    ang = np.where(use, np.arctan2(y - cy, x - cx), np.nan)
    log(f'Bag: video {t_end - t_start:.1f} s ({n_frames} frames), centro ({cx:.2f}, {cy:.2f}), '
        f'radio {r:.2f} m, {int(use.sum())} frames en la órbita')
    return ang


def _resize(img, max_width):
    import cv2
    h, w = img.shape[:2]
    if max_width and w > max_width:
        img = cv2.resize(img, (max_width, round(h * max_width / w)),
                         interpolation=cv2.INTER_AREA)
    return img


def _score_image(img):
    import cv2
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    # Nitidez sobre una versión reducida: más rápido y menos sensible a ruido
    return sharpness(_resize(gray, 640))


def _read_video(path):
    import cv2
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise RuntimeError(f'No se pudo abrir el video {path}')
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                return
            yield frame
    finally:
        cap.release()


def list_images(folder):
    return sorted(p for p in glob.glob(os.path.join(folder, '*'))
                  if p.lower().endswith(IMAGE_EXTS))


def extract_frames(source, out_dir, n_frames=120, max_width=0,
                   blur_ratio=0.35, skip_start=0, skip_end=0, log=print,
                   angles_fn=None):
    """Copia a out_dir los frames elegidos de un video o carpeta.

    skip_start / skip_end: frames a ignorar al principio y al final
    (despegue y aterrizaje). angles_fn(total) -> ángulo de cada frame (NaN =
    descartar): reparte por ángulo alrededor del objeto en vez de por índice.
    Devuelve la lista de rutas escritas.
    """
    import cv2
    if os.path.isdir(source):
        files = list_images(source)
        if not files:
            raise RuntimeError(f'No hay imágenes en {source}')
        load = cv2.imread
        items = files
        scores = [_score_image(load(p)) for p in files]
    elif source.lower().endswith(VIDEO_EXTS) and os.path.isfile(source):
        items = None
        scores = [_score_image(f) for f in _read_video(source)]
        if not scores:
            raise RuntimeError(f'El video {source} no tiene frames legibles')
    else:
        raise RuntimeError(f'Entrada no válida (AVI o carpeta): {source}')

    total = len(scores)
    lo, hi = skip_start, total - skip_end
    if hi - lo < 2:
        raise RuntimeError(f'Quedan {max(hi - lo, 0)} frames tras recortar '
                           f'inicio/fin (total {total})')
    if angles_fn is not None:
        ang = np.asarray(angles_fn(total), dtype=float)
        ang[:lo] = np.nan
        ang[hi:] = np.nan
        chosen = select_frames_by_angle(scores, ang, n_frames, blur_ratio)
    else:
        chosen = [lo + i for i in select_frames(scores[lo:hi], n_frames, blur_ratio)]
    if not chosen:
        raise RuntimeError('No quedó ningún frame tras la selección')
    log(f'Frames: {total} leídos, {hi - lo} útiles, {len(chosen)} elegidos '
        f'(nitidez mediana {np.median(scores[lo:hi]):.0f})')

    os.makedirs(out_dir, exist_ok=True)
    for old in list_images(out_dir):
        os.remove(old)
    wanted = set(chosen)
    written = []
    frames = (load(p) for p in items) if items else _read_video(source)
    for idx, frame in enumerate(frames):
        if idx in wanted:
            path = os.path.join(out_dir, f'frame_{idx:06d}.png')
            cv2.imwrite(path, _resize(frame, max_width))
            written.append(path)
        if idx >= chosen[-1]:
            break
    return written


# ── COLMAP ───────────────────────────────────────────────────────────

def orbit_pairs(names, overlap=8):
    """Pares a emparejar en una órbita: cada imagen con sus `overlap`
    siguientes y con dos más lejanas (2·overlap y 4·overlap), cerrando el
    lazo (la última con las primeras). Mucho menos pares que exhaustive.
    """
    n = len(names)
    steps = sorted({j for j in list(range(1, overlap + 1)) + [2 * overlap, 4 * overlap]
                    if 0 < j < n})
    pairs = set()
    for i in range(n):
        for j in steps:
            a, b = i, (i + j) % n
            pairs.add((min(a, b), max(a, b)))
    return [(names[a], names[b]) for a, b in sorted(pairs)]


class Colmap:
    """Envoltura mínima de la CLI de COLMAP (3.7 a 3.13)."""

    def __init__(self, exe='colmap', log=print):
        self.exe = shutil.which(exe) or exe
        if not shutil.which(self.exe):
            raise RuntimeError(
                'No se encontró COLMAP. Instálalo con `sudo apt install colmap` '
                'o pasa --colmap /ruta/a/colmap')
        self.log = log
        info = subprocess.run([self.exe, '-h'], capture_output=True, text=True)
        head = (info.stdout + info.stderr).splitlines()
        self.version = next((ln.strip() for ln in head if 'COLMAP' in ln), 'COLMAP ?')
        self.has_cuda = not any('without CUDA' in ln for ln in head)
        self._help = {}

    def options(self, command):
        if command not in self._help:
            out = subprocess.run([self.exe, command, '-h'],
                                 capture_output=True, text=True)
            self._help[command] = set(
                re.findall(r'--([A-Za-z_.]+)', out.stdout + out.stderr))
        return self._help[command]

    def gpu_option(self, command):
        """Nombre de la opción use_gpu: cambió de Sift* a Feature* en 3.12."""
        opts = self.options(command)
        for name in ('FeatureExtraction.use_gpu', 'SiftExtraction.use_gpu',
                     'FeatureMatching.use_gpu', 'SiftMatching.use_gpu'):
            if name in opts:
                return name
        return None

    def run(self, command, **kwargs):
        args = [self.exe, command]
        for key, val in kwargs.items():
            if val is None:
                continue
            if isinstance(val, bool):
                val = int(val)
            args += [f'--{key}', str(val)]
        self.log('$ ' + ' '.join(args))
        res = subprocess.run(args, capture_output=True, text=True)
        if res.returncode != 0:
            tail = '\n'.join((res.stdout + res.stderr).strip().splitlines()[-25:])
            raise RuntimeError(f'colmap {command} falló (código {res.returncode}):\n{tail}')
        return res.stdout + res.stderr


def count_model(model_dir):
    """(imágenes registradas, puntos 3D) de un modelo en formato TXT."""
    def lines(name):
        path = os.path.join(model_dir, name)
        if not os.path.isfile(path):
            return []
        with open(path, encoding='utf-8') as f:
            return [ln for ln in f if not ln.startswith('#')]
    # images.txt usa dos líneas por imagen (la de puntos 2D puede estar vacía)
    images = sum(1 for ln in lines('images.txt')[0::2] if ln.strip())
    points = sum(1 for ln in lines('points3D.txt') if ln.strip())
    return images, points


def reconstruct(images_dir, work_dir, camera_model='SIMPLE_RADIAL',
                camera_params='', matcher='orbita', overlap=8, dense='auto',
                use_gpu=None, max_features=4096, colmap_exe='colmap', log=print):
    """Corre COLMAP sobre images_dir. Devuelve dict con rutas de resultados."""
    cm = Colmap(colmap_exe, log=log)
    if use_gpu is None:
        use_gpu = cm.has_cuda
    log(f'{cm.version} | CUDA: {"sí" if cm.has_cuda else "no"} | '
        f'GPU: {"sí" if use_gpu else "no"}')

    n_images = len(list_images(images_dir))
    if n_images < 3:
        raise RuntimeError(f'Se necesitan al menos 3 imágenes (hay {n_images})')
    os.makedirs(work_dir, exist_ok=True)
    db = os.path.join(work_dir, 'database.db')
    if os.path.exists(db):
        os.remove(db)

    feat = {
        'database_path': db,
        'image_path': images_dir,
        'ImageReader.camera_model': camera_model,
        'ImageReader.single_camera': True,
        'ImageReader.camera_params': camera_params or None,
    }
    gpu = cm.gpu_option('feature_extractor')
    if gpu:
        feat[gpu] = use_gpu
    key = next((k for k in ('FeatureExtraction.max_num_features',
                            'SiftExtraction.max_num_features')
                if k in cm.options('feature_extractor')), None)
    if key and max_features:
        feat[key] = max_features
    cm.run('feature_extractor', **feat)

    match = {'database_path': db}
    if matcher == 'orbita':
        command = 'matches_importer'
        names = [os.path.basename(p) for p in list_images(images_dir)]
        pairs = orbit_pairs(names, overlap)
        pairs_path = os.path.join(work_dir, 'pares_orbita.txt')
        with open(pairs_path, 'w', encoding='utf-8') as f:
            f.writelines(f'{a} {b}\n' for a, b in pairs)
        match.update(match_list_path=pairs_path, match_type='pairs')
        log(f'{len(pairs)} pares de imágenes (órbita, traslape {overlap})')
    elif matcher == 'sequential':
        command = 'sequential_matcher'
        match['SequentialMatching.overlap'] = overlap
        match['SequentialMatching.quadratic_overlap'] = True
    elif matcher == 'exhaustive':
        command = 'exhaustive_matcher'
    else:
        raise ValueError(f'matcher desconocido: {matcher}')
    gpu = cm.gpu_option(command)
    if gpu:
        match[gpu] = use_gpu
    cm.run(command, **match)

    sparse = os.path.join(work_dir, 'sparse')
    if os.path.isdir(sparse):
        shutil.rmtree(sparse)
    os.makedirs(sparse)
    cm.run('mapper', database_path=db, image_path=images_dir, output_path=sparse)

    # El mapper puede dejar varios modelos (0, 1, ...): se usa el más grande.
    models = []
    for d in sorted(os.listdir(sparse)):
        mdir = os.path.join(sparse, d)
        if not os.path.isdir(mdir):
            continue
        txt = os.path.join(work_dir, 'sparse_txt', d)
        os.makedirs(txt, exist_ok=True)
        cm.run('model_converter', input_path=mdir, output_path=txt, output_type='TXT')
        models.append((count_model(txt), mdir, txt))
    if not models:
        raise RuntimeError('COLMAP no pudo reconstruir ningún modelo: revisa que '
                           'los frames tengan textura y traslape suficiente.')
    models.sort(key=lambda m: m[0], reverse=True)
    (n_reg, n_pts), best, best_txt = models[0]
    log(f'Modelo disperso: {len(models)} modelo(s); el mayor registra '
        f'{n_reg}/{n_images} imágenes y {n_pts} puntos')

    result = {'n_images': n_images, 'n_registered': n_reg, 'n_points': n_pts,
              'n_models': len(models), 'sparse_model': best,
              'sparse_txt': best_txt}
    sparse_ply = os.path.join(work_dir, 'sparse.ply')
    cm.run('model_converter', input_path=best, output_path=sparse_ply,
           output_type='PLY')
    result['sparse_ply'] = sparse_ply

    # Espacio de trabajo sin distorsión: lo necesitan el mallado y la nube densa
    dense_dir = os.path.join(work_dir, 'dense')
    if os.path.isdir(dense_dir):
        shutil.rmtree(dense_dir)
    cm.run('image_undistorter', image_path=images_dir, input_path=best,
           output_path=dense_dir, output_type='COLMAP')

    do_dense = cm.has_cuda if dense == 'auto' else dense == 'on'
    if do_dense and not cm.has_cuda:
        log('AVISO: esta compilación de COLMAP no tiene CUDA; se omite la '
            'nube densa.')
        do_dense = False

    # Malla sin CUDA: normales por PCA + Poisson de COLMAP (CPU). No se usa
    # delaunay_mesher sobre la nube dispersa: en COLMAP 3.9 de apt se cuelga.
    try:
        with_normals = os.path.join(work_dir, 'sparse_normales.ply')
        sparse_normals_ply(best_txt, with_normals)
        mesh = os.path.join(work_dir, 'mesh_sparse_poisson.ply')
        cm.run('poisson_mesher', input_path=with_normals, output_path=mesh,
               **{'PoissonMeshing.depth': 9, 'PoissonMeshing.trim': 7})
        result['mesh_sparse'] = mesh
    except RuntimeError as e:
        log(f'AVISO: no se generó la malla sobre la nube dispersa: {e}')

    if do_dense:
        cm.run('patch_match_stereo', workspace_path=dense_dir,
               workspace_format='COLMAP',
               **{'PatchMatchStereo.geom_consistency': True})
        fused = os.path.join(work_dir, 'dense.ply')
        cm.run('stereo_fusion', workspace_path=dense_dir,
               workspace_format='COLMAP', input_type='geometric',
               output_path=fused)
        result['dense_ply'] = fused
        poisson = os.path.join(work_dir, 'mesh_poisson.ply')
        cm.run('poisson_mesher', input_path=fused, output_path=poisson)
        result['mesh_poisson'] = poisson
    return result


# ── Malla sin CUDA ───────────────────────────────────────────────────

def _quat_to_rot(qw, qx, qy, qz):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


def read_model_txt(model_dir):
    """Lee un modelo COLMAP en TXT.

    Devuelve (centros de cámara {image_id: xyz}, puntos Nx3, colores Nx3
    uint8, lista con los image_id que ven cada punto).
    """
    centers = {}
    with open(os.path.join(model_dir, 'images.txt'), encoding='utf-8') as f:
        lines = [ln for ln in f if not ln.startswith('#')]
    for ln in lines[0::2]:
        v = ln.split()
        if len(v) < 10:
            continue
        qw, qx, qy, qz, tx, ty, tz = map(float, v[1:8])
        centers[int(v[0])] = -_quat_to_rot(qw, qx, qy, qz).T @ np.array([tx, ty, tz])
    pts, cols, tracks = [], [], []
    with open(os.path.join(model_dir, 'points3D.txt'), encoding='utf-8') as f:
        for ln in f:
            if ln.startswith('#') or not ln.strip():
                continue
            v = ln.split()
            pts.append([float(x) for x in v[1:4]])
            cols.append([int(x) for x in v[4:7]])
            tracks.append([int(x) for x in v[8::2]])
    return (centers, np.array(pts, dtype=float).reshape(-1, 3),
            np.array(cols, dtype=np.uint8).reshape(-1, 3), tracks)


def estimate_normals(points, view_dirs, radius, k=20):
    """Normales por PCA de los vecinos (rejilla de vóxeles, sin scipy).

    view_dirs: vector de cada punto hacia las cámaras que lo ven; orienta la
    normal hacia afuera y se usa tal cual si el punto tiene pocos vecinos.
    """
    normals = view_dirs / np.maximum(np.linalg.norm(view_dirs, axis=1, keepdims=True), 1e-12)
    keys = np.floor(points / radius).astype(np.int64)
    cells = {}
    for i, key in enumerate(map(tuple, keys)):
        cells.setdefault(key, []).append(i)
    cells = {key: np.array(idx) for key, idx in cells.items()}
    offsets = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)]
    for key, idx in cells.items():
        cand = [cells[c] for c in ((key[0] + a, key[1] + b, key[2] + c)
                                   for a, b, c in offsets) if c in cells]
        cand = np.concatenate(cand)
        if len(cand) < 5:
            continue
        P = points[cand]
        for i in idx:
            d = np.sum((P - points[i]) ** 2, axis=1)
            nb = P[np.argsort(d)[:k]]
            if len(nb) < 5:
                continue
            cov = np.cov((nb - nb.mean(axis=0)).T)
            normal = np.linalg.eigh(cov)[1][:, 0]
            normals[i] = normal if normal @ view_dirs[i] >= 0 else -normal
    return normals


def write_ply(path, points, normals, colors):
    with open(path, 'w', encoding='utf-8') as f:
        f.write('ply\nformat ascii 1.0\n'
                f'element vertex {len(points)}\n'
                'property float x\nproperty float y\nproperty float z\n'
                'property float nx\nproperty float ny\nproperty float nz\n'
                'property uchar red\nproperty uchar green\nproperty uchar blue\n'
                'end_header\n')
        for p, nrm, c in zip(points, normals, colors):
            f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} '
                    f'{nrm[0]:.5f} {nrm[1]:.5f} {nrm[2]:.5f} {c[0]} {c[1]} {c[2]}\n')


def sparse_normals_ply(model_txt, out_path):
    """Nube dispersa con normales (entrada de poisson_mesher sin CUDA).

    El radio de vecindad es 3% del radio de la órbita de las cámaras, así no
    depende de la escala arbitraria de COLMAP. Devuelve el número de puntos.
    """
    centers, points, colors, tracks = read_model_txt(model_txt)
    if len(points) == 0:
        raise RuntimeError('El modelo no tiene puntos')
    cams = np.array(list(centers.values()))
    orbit = float(np.median(np.linalg.norm(cams - np.median(cams, axis=0), axis=1)))
    view = np.array([np.mean([centers[i] for i in t if i in centers], axis=0) - p
                     if any(i in centers for i in t) else np.zeros(3)
                     for p, t in zip(points, tracks)])
    normals = estimate_normals(points, view, radius=max(orbit, 1e-6) * 0.03)
    write_ply(out_path, points, normals, colors)
    return len(points)
