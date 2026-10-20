"""
Download real terrain heights around a place for the garden relief and store
them in relief/<name>.png (16-bit) and relief/<name>.yaml.

Heights come from the Terrain Tiles on AWS Open Data (terrarium format,
https://registry.opendata.aws/terrain-tiles/), mostly SRTM with about 30 m
resolution. That is enough for the overall slope and broad undulations of the
place; terrain.py smooths out the SRTM noise and adds the small relief.

The heights are resampled to a north-up grid in meters centered at the place.

Usage (needs internet access):
    python3 fetch_relief.py                          # the default place
    python3 fetch_relief.py --name steppe --lat 47.0787331 --lon 33.1950273
"""

import argparse
import io
import math
import os
import urllib.request

import numpy as np
import yaml
from PIL import Image

GARDEN_DIR = os.path.join(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'), 'garden')
URL = 'https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png'
ZOOM = 15
TILE = 256


def mercator(lat, lon, zoom):
    """Global pixel coordinates of a point in Web Mercator at a zoom level."""
    n = 2 ** zoom * TILE
    x = (lon + 180) / 360 * n
    y = (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n
    return x, y


def tile_heights(x, y):
    with urllib.request.urlopen(URL.format(z=ZOOM, x=x, y=y), timeout=30) as r:
        a = np.asarray(Image.open(io.BytesIO(r.read())).convert('RGB')).astype(float)
    return a[..., 0] * 256 + a[..., 1] + a[..., 2] / 256 - 32768


def fetch(lat, lon, size, step):
    """Heights on a north-up grid of 'size' m with 'step' m spacing centered at (lat, lon); row 0 is north."""
    m_per_px = 2 * math.pi * 6378137 * math.cos(math.radians(lat)) / (2 ** ZOOM * TILE)
    cx, cy = mercator(lat, lon, ZOOM)
    half = size / 2 / m_per_px + 2
    tx0, tx1 = int((cx - half) // TILE), int((cx + half) // TILE)
    ty0, ty1 = int((cy - half) // TILE), int((cy + half) // TILE)
    mosaic = np.zeros(((ty1 - ty0 + 1) * TILE, (tx1 - tx0 + 1) * TILE))
    for ty in range(ty0, ty1 + 1):
        for tx in range(tx0, tx1 + 1):
            mosaic[(ty - ty0) * TILE:(ty - ty0 + 1) * TILE, (tx - tx0) * TILE:(tx - tx0 + 1) * TILE] = tile_heights(tx, ty)

    n = int(round(size / step)) + 1
    c = (np.arange(n) - (n - 1) / 2) * step
    east, north = np.meshgrid(c, c[::-1])
    # Bilinear sampling of the mosaic at the grid points.
    px = cx + east / m_per_px - tx0 * TILE - 0.5
    py = cy - north / m_per_px - ty0 * TILE - 0.5
    x0, y0 = np.floor(px).astype(int), np.floor(py).astype(int)
    fx, fy = px - x0, py - y0
    h = (mosaic[y0, x0] * (1 - fx) * (1 - fy) + mosaic[y0, x0 + 1] * fx * (1 - fy)
         + mosaic[y0 + 1, x0] * (1 - fx) * fy + mosaic[y0 + 1, x0 + 1] * fx * fy)
    return h


def main():
    parser = argparse.ArgumentParser(description='Download real terrain heights for the garden relief.')
    parser.add_argument('--name', default='steppe')
    parser.add_argument('--lat', type=float, default=47.0787331)
    parser.add_argument('--lon', type=float, default=33.1950273)
    parser.add_argument('--size', type=float, default=800.0, help='side of the area, m')
    parser.add_argument('--step', type=float, default=5.0, help='grid step, m')
    args = parser.parse_args()

    h = fetch(args.lat, args.lon, args.size, args.step)
    h_min, h_max = float(h.min()), float(h.max())
    out = os.path.join(GARDEN_DIR, 'relief')
    os.makedirs(out, exist_ok=True)
    Image.fromarray(np.round((h - h_min) / (h_max - h_min) * 65535).astype(np.uint16)).save(
        os.path.join(out, args.name + '.png'))
    info = dict(
        lat=args.lat, lon=args.lon, size=args.size, step=args.step,
        h_min=round(h_min, 3), h_max=round(h_max, 3),
        source='Terrain Tiles on AWS Open Data (terrarium, mostly SRTM), https://registry.opendata.aws/terrain-tiles/',
    )
    with open(os.path.join(out, args.name + '.yaml'), 'w') as f:
        f.write('# Real terrain heights for the garden relief, made by fetch_relief.py.\n')
        f.write('# The PNG holds heights from h_min (black) to h_max (white), m, on a\n')
        f.write('# north-up grid of size x size m with step m spacing centered at lat, lon.\n')
        yaml.safe_dump(info, f, sort_keys=False)
    print(f'{args.name}: {h.shape[0]}x{h.shape[1]} samples, heights {h_min:.1f}..{h_max:.1f} m')


if __name__ == '__main__':
    main()
