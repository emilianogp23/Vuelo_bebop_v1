#!/usr/bin/env python3
"""
Genera las texturas del mundo de Gazebo (bebop1.sdf) para fotogrametría.

COLMAP necesita textura: superficies lisas de un color no dan puntos SIFT.
Crea en control_bebop/resource/meshes/texturas_lab/materials/textures/:
    madera.jpg    paredes del corral (tablones con veta, 4 m x 0.6 m)
    caja.jpg      caja base (calcomanías de colores sobre cartón)
    cilindro.jpg  objeto de interés (etiqueta con figuras y letras)
    piso.jpg      tapete del corral (concreto con motas, 4 m x 4 m)

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


def madera(rng, w=2048, h=308, planks=6):
    """Tablones horizontales con veta, nudos y juntas oscuras."""
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
        base = np.array([0.36, 0.52, 0.72]) * tone        # BGR, café claro
        dark = np.array([0.16, 0.28, 0.45]) * tone
        t = (0.65 * grain + 0.35 * fine)[..., None]
        img[y0:y1] = (dark + (base - dark) * t)[y0:y1]
        # Nudos
        for _ in range(rng.integers(2, 6)):
            cx, cy = rng.uniform(0, w), rng.uniform(y0 + 5, y1 - 5)
            r = rng.uniform(4, 12)
            d = np.sqrt(((xx - cx) / 2.5) ** 2 + (yy - cy) ** 2)
            ring = 0.5 + 0.5 * np.sin(d * 1.6)
            m = (d < r * 2.5)[..., None] * (1 - np.clip(d / (r * 2.5), 0, 1))[..., None]
            img = img * (1 - m * 0.6) + (dark * 0.7 * ring[..., None]) * m * 0.6
        # Junta entre tablones y cortes verticales escalonados
        img[y0:y0 + 2] *= 0.35
        for x in rng.uniform(0, w, 2).astype(int):
            img[y0:y1, x:x + 3] *= 0.4
    return np.clip(img * 255, 0, 255).astype(np.uint8)


def figuras(rng, img, n, size_range, alpha=True):
    """Círculos, rectángulos, triángulos y líneas de colores saturados."""
    h, w = img.shape[:2]
    for _ in range(n):
        color = tuple(int(c) for c in rng.integers(0, 256, 3))
        s = int(rng.uniform(*size_range))
        cx, cy = int(rng.uniform(0, w)), int(rng.uniform(0, h))
        kind = rng.integers(0, 4)
        layer = img.copy()
        if kind == 0:
            cv2.circle(layer, (cx, cy), s // 2, color, -1, cv2.LINE_AA)
        elif kind == 1:
            cv2.rectangle(layer, (cx, cy), (cx + s, cy + int(s * rng.uniform(0.3, 1.5))),
                          color, -1)
        elif kind == 2:
            pts = np.array([[cx, cy], [cx + s, cy + s // 3], [cx + s // 4, cy + s]], np.int32)
            cv2.fillPoly(layer, [pts], color, cv2.LINE_AA)
        else:
            cv2.line(layer, (cx, cy), (cx + int(rng.uniform(-2, 2) * s),
                                       cy + int(rng.uniform(-2, 2) * s)),
                     color, max(2, s // 8), cv2.LINE_AA)
        a = rng.uniform(0.6, 1.0) if alpha else 1.0
        img[:] = cv2.addWeighted(layer, a, img, 1 - a, 0)
    return img


def texto(rng, img, n, scale_range):
    h, w = img.shape[:2]
    chars = 'ABCDEFGHJKLMNPRSTUVWXYZ0123456789'
    for _ in range(n):
        txt = ''.join(rng.choice(list(chars), rng.integers(2, 6)))
        sc = rng.uniform(*scale_range)
        color = tuple(int(c) for c in rng.integers(0, 200, 3))
        org = (int(rng.uniform(0, w * 0.9)), int(rng.uniform(h * 0.05, h)))
        cv2.putText(img, txt, org, cv2.FONT_HERSHEY_DUPLEX, sc, color,
                    max(1, int(sc * 2)), cv2.LINE_AA)
    return img


def caja(rng, size=1024):
    """Cartón con calcomanías y letras de varios tamaños."""
    n = noise(rng, size, size, (8, 32, 128, 512))
    img = (np.array([0.45, 0.62, 0.78]) * (0.75 + 0.5 * n[..., None]) * 255).astype(np.uint8)
    figuras(rng, img, 90, (25, 160))
    texto(rng, img, 25, (0.8, 3.0))
    return img


def cilindro(rng, w=1536, h=612):
    """Etiqueta: franjas, figuras y letras (alrededor del cilindro)."""
    img = np.full((h, w, 3), 235, np.uint8)
    for y in range(0, h, int(h / 6)):
        color = tuple(int(c) for c in rng.integers(40, 256, 3))
        cv2.rectangle(img, (0, y), (w, y + int(rng.uniform(10, 40))), color, -1)
    figuras(rng, img, 120, (15, 110))
    texto(rng, img, 40, (0.7, 2.6))
    return img


def piso(rng, size=2048):
    """Concreto gris con motas y manchas de colores (tapete del corral)."""
    n = noise(rng, size, size, (8, 32, 128, 512, 1024))
    img = ((0.42 + 0.3 * n[..., None]) * np.array([1.0, 1.0, 1.0]) * 255).astype(np.uint8)
    figuras(rng, img, 600, (10, 70))
    speck = rng.random((size, size)) > 0.985
    img[speck] = (rng.random((speck.sum(), 3)) * 255).astype(np.uint8)
    return img


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
