#!/usr/bin/env python3
"""
Genera las texturas del mundo de Gazebo (bebop1.sdf) para fotogrametría.

COLMAP necesita textura: superficies lisas de un color no dan puntos SIFT.
Crea en control_bebop/resource/meshes/texturas_lab/materials/textures/:
    madera.jpg    paredes del corral (tablones con veta, 4 m x 0.6 m)
    caja.jpg      caja base (granito gris rosado con granos)
    cilindro.jpg  objeto de interés (piedra con vetas, 0.63 m x 0.25 m)
    piso.jpg      tapete del corral (laminado de roble, 4 m x 4 m)

El mundo las carga con model://texturas_lab/materials/textures/<nombre>.jpg
(sim_common.py agrega resource/meshes a GZ_SIM_RESOURCE_PATH).

Uso:  python3 scripts/generar_texturas_gazebo.py
"""
import os

import cv2
import numpy as np

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   'resource', 'meshes', 'texturas_lab', 'materials', 'textures')


def noise(rng, h, w, scales=(4, 16, 64, 256)):
    """Ruido fractal en [0, 1], suave a varias escalas."""
    acc = np.zeros((h, w), np.float32)
    for s in scales:
        small = rng.random((max(2, s * h // max(h, w)), max(2, s * w // max(h, w))))
        acc += cv2.resize(small.astype(np.float32), (w, h), interpolation=cv2.INTER_CUBIC)
    acc -= acc.min()
    return acc / max(acc.max(), 1e-6)


def madera(rng, w=2048, h=308, planks=6, base=(0.36, 0.52, 0.72), dark=(0.16, 0.28, 0.45)):
    """Tablones horizontales con veta, nudos y juntas oscuras (colores BGR 0-1)."""
    img = np.zeros((h, w, 3), np.float32)
    ph = h / planks
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    for p in range(planks):
        y0, y1 = int(p * ph), int((p + 1) * ph)
        n = noise(rng, h, w, (3, 9, 40))
        # Veta: líneas casi horizontales deformadas por ruido
        grain = np.sin((yy * 0.9 + n * 60 + rng.uniform(0, 100)) * 0.55)
        grain = 0.5 + 0.5 * grain
        fine = noise(rng, h, w, (200, 600))
        tone = rng.uniform(0.85, 1.15)
        hi = np.array(base) * tone
        lo = np.array(dark) * tone
        t = (0.65 * grain + 0.35 * fine)[..., None]
        img[y0:y1] = (lo + (hi - lo) * t)[y0:y1]
        # Nudos
        for _ in range(rng.integers(2, 6)):
            cx, cy = rng.uniform(0, w), rng.uniform(y0 + 5, y1 - 5)
            r = rng.uniform(4, 12)
            d = np.sqrt(((xx - cx) / 2.5) ** 2 + (yy - cy) ** 2)
            ring = 0.5 + 0.5 * np.sin(d * 1.6)
            m = (d < r * 2.5)[..., None] * (1 - np.clip(d / (r * 2.5), 0, 1))[..., None]
            img = img * (1 - m * 0.6) + (lo * 0.7 * ring[..., None]) * m * 0.6
        # Junta entre tablones y cortes verticales escalonados
        img[y0:y0 + 2] *= 0.35
        for x in rng.uniform(0, w, 2).astype(int):
            img[y0:y1, x:x + 3] *= 0.4
    return np.clip(img * 255, 0, 255).astype(np.uint8)


def caja(rng, size=1024):
    """Granito gris rosado con granos negros, blancos y rosas (caja de 0.56 m)."""
    n = noise(rng, size, size, (8, 32, 128))
    img = np.array([0.60, 0.60, 0.68]) * (0.8 + 0.3 * n[..., None]) * 255   # BGR
    # Granos irregulares: ruido fino umbralizado (color BGR, fracción cubierta, escalas)
    for color, cover, scales in (((35, 35, 40), 0.16, (96, 192)), ((235, 235, 240), 0.07, (160, 320)),
                                 ((125, 115, 195), 0.14, (64, 128)), ((95, 95, 100), 0.10, (192, 384))):
        g = noise(rng, size, size, scales)
        img[g > np.quantile(g, 1 - cover)] = color
    return cv2.GaussianBlur(img.astype(np.uint8), (3, 3), 0)


def cilindro(rng, w=1536, h=612):
    """Piedra gris cálida con vetas y motas (alrededor del cilindro)."""
    n = noise(rng, h, w, (6, 24, 96, 384))
    img = (np.array([0.50, 0.55, 0.60]) * (0.55 + 0.6 * n[..., None]) * 255).astype(np.uint8)
    for _ in range(35):   # vetas: caminata aleatoria con rumbo, oscuras o claras
        step = np.column_stack([np.full(40, 12.0), rng.normal(0, 6, 40)])
        ang = rng.uniform(-0.6, 0.6)
        step = step @ np.array([[np.cos(ang), np.sin(ang)], [-np.sin(ang), np.cos(ang)]])
        pts = np.cumsum(step, axis=0) + rng.uniform(0, [w, h])
        gray = int(rng.choice([rng.integers(45, 80), rng.integers(185, 220)]))
        cv2.polylines(img, [pts.astype(np.int32)], False, (gray,) * 3, int(rng.integers(1, 4)), cv2.LINE_AA)
    speck = rng.random((h, w)) > 0.97
    img[speck] = (rng.random((speck.sum(), 1)) * 255).astype(np.uint8)
    return img


def piso(rng, size=2048):
    """Laminado de roble claro: tablas de 20 cm (20 sobre 4 m) con juntas."""
    return madera(rng, w=size, h=size, planks=20, base=(0.55, 0.72, 0.86), dark=(0.33, 0.48, 0.66))


def main():
    rng = np.random.default_rng(23)
    os.makedirs(OUT, exist_ok=True)
    for name, fn in (('madera', madera), ('caja', caja),
                     ('cilindro', cilindro), ('piso', piso)):
        path = os.path.join(OUT, f'{name}.jpg')
        cv2.imwrite(path, fn(rng), [cv2.IMWRITE_JPEG_QUALITY, 92])
        print(f'{path} ({os.path.getsize(path) // 1024} KB)')


if __name__ == '__main__':
    main()
