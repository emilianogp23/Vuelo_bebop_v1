#!/usr/bin/env python3
"""
Reconstrucción 3D del objeto a partir del video de la órbita (COLMAP).

Entrada: el AVI que graba nodo_video (videos/vuelo_*.avi) o una carpeta de
imágenes. Salida en <paquete>/reconstrucciones/<nombre>/:
    frames/                 frames elegidos (los más nítidos, repartidos)
    sparse/0, sparse_txt/0  modelo COLMAP (poses de cámara + puntos)
    sparse.ply              nube dispersa
    mesh_sparse_poisson.ply  malla sobre la nube dispersa
    dense.ply, mesh_poisson.ply  nube densa y su malla (solo con CUDA)

No necesita ROS. Requiere colmap (`sudo apt install colmap`), opencv y numpy.

Uso:
    python3 scripts/reconstruccion_3d.py videos/vuelo_20261019_120000.avi
    python3 scripts/reconstruccion_3d.py carpeta_frames/ --frames 80
    python3 scripts/reconstruccion_3d.py vuelo.avi --skip-start 150 --skip-end 150
    python3 scripts/reconstruccion_3d.py vuelo.avi --solo-frames   # sin COLMAP
    # Frames repartidos por ángulo alrededor del objeto (requiere ROS 2 sourced):
    python3 scripts/reconstruccion_3d.py videos/vuelo_X.avi --bag bags/vuelo_real_X

Detalles en control_bebop/RECONSTRUCCION_3D.md.
"""
import argparse
import json
import os
import sys
import time

# Permite correr el script sin instalar el paquete (igual que desde src/)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_bebop import reconstruccion as rec  # noqa: E402
from control_bebop.paths import data_dir  # noqa: E402

# Valores iniciales de intrínsecos (SIMPLE_RADIAL: f, cx, cy, k). COLMAP los
# refina en el ajuste. En el dron real no hay calibración: con 'auto' COLMAP
# parte de f = 1.2·ancho.
CAMERAS = {
    # bebop_gz/models/parrot_bebop_2/model.sdf: 640x480, horizontal_fov 1.2,
    # f = 320 / tan(0.6). El SDF declara una lente custom (r = 1.05·tan(θ/4)),
    # por eso no se fija un modelo PINHOLE.
    'gazebo': ('SIMPLE_RADIAL', '470.0,320.0,240.0,0.0'),
}


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('entrada', help='AVI de la órbita o carpeta de imágenes')
    ap.add_argument('--salida', default='',
                    help='carpeta de salida (por defecto reconstrucciones/<nombre>)')
    ap.add_argument('--frames', type=int, default=120,
                    help='frames a usar (por defecto 120)')
    ap.add_argument('--skip-start', type=int, default=0,
                    help='frames a ignorar al inicio (despegue)')
    ap.add_argument('--skip-end', type=int, default=0,
                    help='frames a ignorar al final (aterrizaje)')
    ap.add_argument('--blur', type=float, default=0.35,
                    help='descarta ventanas con nitidez < blur·mediana (0 = no)')
    ap.add_argument('--bag', default='',
                    help='bag del vuelo: reparte los frames por ángulo alrededor del '
                         'objeto, solo en AUTOMATIC (requiere ROS 2)')
    ap.add_argument('--centro', default='',
                    help='x,y del objeto en el marco del bag (por defecto, el centro '
                         'del círculo ajustado a la órbita)')
    ap.add_argument('--offset-bag', type=float, default=0.0,
                    help='segundos a sumar al tiempo del video respecto al bag')
    ap.add_argument('--ancho', type=int, default=0,
                    help='reduce los frames a este ancho en px (0 = original)')
    ap.add_argument('--camara', default='auto', choices=['auto'] + sorted(CAMERAS),
                    help='intrínsecos: auto (COLMAP los estima) o gazebo')
    ap.add_argument('--camera-model', default='',
                    help='modelo de cámara COLMAP (sobrescribe --camara)')
    ap.add_argument('--camera-params', default='',
                    help='parámetros del modelo, separados por comas')
    ap.add_argument('--matcher', default='orbita',
                    choices=['orbita', 'sequential', 'exhaustive'],
                    help='orbita: vecinos en el tiempo cerrando el lazo '
                         '(rápido); exhaustive: todos contra todos (lento)')
    ap.add_argument('--traslape', type=int, default=8,
                    help='vecinos por imagen con --matcher orbita/sequential')
    ap.add_argument('--max-features', type=int, default=4096,
                    help='puntos SIFT por imagen (por defecto 4096)')
    ap.add_argument('--denso', default='auto', choices=['auto', 'on', 'off'],
                    help='nube densa: auto = solo si COLMAP tiene CUDA')
    ap.add_argument('--gpu', default='auto', choices=['auto', 'on', 'off'])
    ap.add_argument('--colmap', default='colmap', help='ejecutable de COLMAP')
    ap.add_argument('--solo-frames', action='store_true',
                    help='solo extrae los frames, no corre COLMAP')
    args = ap.parse_args()
    sys.stdout.reconfigure(line_buffering=True)  # progreso visible aunque se redirija

    entrada = os.path.abspath(os.path.expanduser(args.entrada))
    nombre = os.path.splitext(os.path.basename(entrada.rstrip(os.sep)))[0]
    salida = (os.path.abspath(os.path.expanduser(args.salida)) if args.salida
              else os.path.join(data_dir('reconstrucciones'), nombre))
    frames_dir = os.path.join(salida, 'frames')
    t0 = time.time()

    print(f'Entrada: {entrada}\nSalida:  {salida}')
    angles_fn = None
    if args.bag:
        centro = tuple(float(v) for v in args.centro.split(',')) if args.centro else None
        bag = os.path.abspath(os.path.expanduser(args.bag))
        angles_fn = lambda total: rec.bag_frame_angles(  # noqa: E731
            bag, total, center=centro, offset=args.offset_bag)
    try:
        written = rec.extract_frames(
            entrada, frames_dir, n_frames=args.frames, max_width=args.ancho,
            blur_ratio=args.blur, skip_start=args.skip_start, skip_end=args.skip_end,
            angles_fn=angles_fn)
    except (RuntimeError, ImportError) as e:
        print(f'\nERROR: {e}', file=sys.stderr)
        return 1
    print(f'{len(written)} frames en {frames_dir}')
    if args.solo_frames:
        return 0

    model, params = 'SIMPLE_RADIAL', ''
    if args.camara != 'auto':
        if args.ancho:
            sys.exit('--camara con --ancho no es válido: los intrínsecos son '
                     'para la resolución original')
        model, params = CAMERAS[args.camara]
    if args.camera_model:
        model, params = args.camera_model, args.camera_params

    gpu = None if args.gpu == 'auto' else args.gpu == 'on'
    try:
        res = rec.reconstruct(frames_dir, salida, camera_model=model,
                              camera_params=params, matcher=args.matcher,
                              overlap=args.traslape,
                              max_features=args.max_features, dense=args.denso, use_gpu=gpu,
                              colmap_exe=args.colmap)
    except RuntimeError as e:
        print(f'\nERROR: {e}', file=sys.stderr)
        return 1

    res['entrada'] = entrada
    res['segundos'] = round(time.time() - t0, 1)
    with open(os.path.join(salida, 'resumen.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)

    print(f'\nListo en {res["segundos"]:.0f} s. Registradas '
          f'{res["n_registered"]}/{res["n_images"]} imágenes, '
          f'{res["n_points"]} puntos.')
    for key in ('sparse_ply', 'mesh_sparse', 'dense_ply', 'mesh_poisson'):
        if key in res:
            print(f'  {key:12s} {res[key]}')
    if res['n_registered'] < 0.6 * res['n_images']:
        print('AVISO: se registraron pocas imágenes. Ver "Si falla" en '
              'RECONSTRUCCION_3D.md.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
