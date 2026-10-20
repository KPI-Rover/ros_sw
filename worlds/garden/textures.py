"""
Ground textures without visible repetition.

A photo texture repeated over a large area shows a regular grid of copies.
make() builds a macro texture of GRID x GRID repeats of a photo texture from
worlds/media/textures, where every repeat is a differently shifted, flipped
or turned copy, blended with soft irregular seams, and the brightness varies
in broad patches. The macro texture is seamless too, but repeats GRID times
less often, and its copies differ, so the eye doesn't catch the grid.
"""

import os

import numpy as np
from PIL import Image

TEXTURES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'media', 'textures')

GRID = 3            # repeats of the photo texture along each side of the macro texture
SIZE = 3072         # macro texture side, px: the full resolution of the photo texture
SHARPNESS = 20.0    # how narrow the seams between copies are
BRIGHTNESS = 0.08   # broad brightness variation, fraction


def _periodic_noise(n, cells, rng):
    """Smooth seamless noise on an n x n image with 'cells' random values per side, in about [-1, 1]."""
    v = rng.uniform(-1, 1, (cells, cells))
    t = np.arange(n) / n * cells
    i0 = np.floor(t).astype(int)
    f = t - i0
    f = f * f * (3 - 2 * f)
    i1 = (i0 + 1) % cells
    rows = v[i0][:, i0] * (1 - f)[None, :] + v[i0][:, i1] * f[None, :]
    rows1 = v[i1][:, i0] * (1 - f)[None, :] + v[i1][:, i1] * f[None, :]
    return rows * (1 - f)[:, None] + rows1 * f[:, None]


def _variant(img, kind, normal):
    """Flipped or turned copy of a texture; normal map vectors are turned with it."""
    if kind == 1:       # turned 180 degrees
        img = img[::-1, ::-1]
        if normal:
            img = img.copy()
            img[..., :2] = 1 - img[..., :2]
    elif kind == 2:     # mirrored left-right
        img = img[:, ::-1]
        if normal:
            img = img.copy()
            img[..., 0] = 1 - img[..., 0]
    elif kind == 3:     # mirrored top-bottom
        img = img[::-1]
        if normal:
            img = img.copy()
            img[..., 1] = 1 - img[..., 1]
    return img


def make(texture, out_dir, name, seed):
    """
    Build a macro texture of the photo texture directory 'texture' as
    out_dir/name/name_{albedo,normal,roughness}.jpg and return the directory.
    """
    path = os.path.join(out_dir, name)
    if os.path.isfile(os.path.join(path, name + '_roughness.jpg')):
        return path
    os.makedirs(path, exist_ok=True)
    rng = np.random.default_rng(seed)
    cell = SIZE // GRID

    # Each copy: a cell it is centered at (jittered), a shift and a variant.
    centers = (np.stack(np.meshgrid(np.arange(GRID), np.arange(GRID)), -1).reshape(-1, 2)
               + 0.5 + rng.uniform(-0.3, 0.3, (GRID * GRID, 2))) * cell
    shifts = rng.integers(0, cell, (GRID * GRID, 2))
    kinds = rng.integers(0, 4, GRID * GRID)

    # Soft weights of the copies: nearest copy wins, seams wobble with noise.
    yy, xx = np.mgrid[0:SIZE, 0:SIZE].astype(np.float32)
    xx += _periodic_noise(SIZE, GRID * 4, rng).astype(np.float32) * cell * 0.25
    yy += _periodic_noise(SIZE, GRID * 4, rng).astype(np.float32) * cell * 0.25

    def weight(k):
        cx, cy = centers[k]
        dx = (xx - cx + SIZE / 2) % SIZE - SIZE / 2
        dy = (yy - cy + SIZE / 2) % SIZE - SIZE / 2
        return np.exp(-SHARPNESS * (dx * dx + dy * dy) / (cell * cell))

    brightness = 1 + BRIGHTNESS * _periodic_noise(SIZE, GRID, rng) + BRIGHTNESS / 2 * _periodic_noise(SIZE, GRID * 3, rng)

    src = os.path.join(TEXTURES_DIR, texture, texture)
    for kind in ('albedo', 'normal', 'roughness'):
        img = Image.open(f'{src}_{kind}.jpg').convert('RGB').resize((cell, cell), Image.LANCZOS)
        img = np.asarray(img).astype(np.float32) / 255
        out = np.zeros((SIZE, SIZE, 3), np.float32)
        total = np.zeros((SIZE, SIZE), np.float32)
        for k in range(GRID * GRID):
            v = _variant(img, kinds[k], kind == 'normal')
            v = np.roll(v, tuple(shifts[k]), axis=(0, 1))
            w = weight(k)
            out += w[..., None] * np.tile(v, (GRID, GRID, 1))
            total += w
        out /= total[..., None]
        if kind == 'normal':
            # Blending shortens the normals; make them unit length again.
            n = out * 2 - 1
            n /= np.linalg.norm(n, axis=-1, keepdims=True) + 1e-6
            out = (n + 1) / 2
        if kind == 'albedo':
            out *= brightness[..., None]
        Image.fromarray((np.clip(out, 0, 1) * 255).astype(np.uint8)).save(
            os.path.join(path, f'{name}_{kind}.jpg'), quality=90)
    return path
