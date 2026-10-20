"""
Random scattering of library objects (objects.yaml) over the free area of a
world, used by world generators.

The area is a rectangle with shapes in it:
  - 'keep_out' shapes, where objects are not placed (routes, start points,
    structures), and
  - 'obstacles', which the robot cannot pass (walls, structures).
Shapes are tuples:
    ('circle', x, y, radius)
    ('segment', x1, y1, x2, y2, half_width)
    ('arc', cx, cy, radius, a1, a2, half_width)    counterclockwise from a1 to a2
    ('rect', x_min, x_max, y_min, y_max)

An object is accepted only if the free space for the robot (everything
farther than ROBOT_RADIUS from obstacles, objects and the area border) stays
as connected as it was without objects, so objects never close a passage or
cut off a part of the area.
"""

import math
import os
import random
from collections import deque

import numpy as np
import yaml

RESOLUTION = 0.05     # grid cell size, m
ROBOT_RADIUS = 0.2    # robot footprint radius (0.27 x 0.26 m), m
GAP = 0.05            # minimal gap between objects and keep-out shapes, m
ATTEMPTS = 200        # placement attempts per object

OBJECTS_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'include', 'objects.yaml')


def load_library():
    with open(OBJECTS_YAML) as f:
        return yaml.safe_load(f)


class Grid:
    def __init__(self, x_min, x_max, y_min, y_max):
        self.x_min, self.y_min = x_min, y_min
        self.nx = int(math.ceil((x_max - x_min) / RESOLUTION))
        self.ny = int(math.ceil((y_max - y_min) / RESOLUTION))
        xs = x_min + (np.arange(self.nx) + 0.5) * RESOLUTION
        ys = y_min + (np.arange(self.ny) + 0.5) * RESOLUTION
        self.x, self.y = np.meshgrid(xs, ys, indexing='ij')

    def mask(self, shapes, inflate=0.0):
        """Cells within 'inflate' of any of the shapes."""
        result = np.zeros((self.nx, self.ny), bool)
        for shape in shapes:
            kind, *p = shape
            if kind == 'circle':
                x, y, r = p
                result |= np.hypot(self.x - x, self.y - y) <= r + inflate
            elif kind == 'segment':
                x1, y1, x2, y2, half = p
                dx, dy = x2 - x1, y2 - y1
                t = np.clip(((self.x - x1) * dx + (self.y - y1) * dy) / max(dx * dx + dy * dy, 1e-12), 0, 1)
                result |= np.hypot(self.x - x1 - t * dx, self.y - y1 - t * dy) <= half + inflate
            elif kind == 'arc':
                cx, cy, r, a1, a2, half = p
                a = np.mod(np.arctan2(self.y - cy, self.x - cx) - a1, 2 * math.pi)
                within = a <= (a2 - a1)
                ring = np.abs(np.hypot(self.x - cx, self.y - cy) - r) <= half + inflate
                ends = [('circle', cx + r * math.cos(b), cy + r * math.sin(b), half) for b in (a1, a2)]
                result |= (within & ring) | self.mask(ends, inflate)
            elif kind == 'rect':
                x0, x1, y0, y1 = p
                result |= ((self.x >= x0 - inflate) & (self.x <= x1 + inflate) &
                           (self.y >= y0 - inflate) & (self.y <= y1 + inflate))
            else:
                raise ValueError(f'Unknown shape {kind}.')
        return result

    def border(self, inflate):
        """Cells within 'inflate' of the area border."""
        r = int(math.ceil(inflate / RESOLUTION))
        result = np.zeros((self.nx, self.ny), bool)
        result[:r, :] = result[-r:, :] = result[:, :r] = result[:, -r:] = True
        return result


def components(free):
    """Number of 4-connected components of free cells."""
    seen = np.zeros_like(free)
    count = 0
    nx, ny = free.shape
    for i, j in zip(*np.nonzero(free)):
        if seen[i, j]:
            continue
        count += 1
        seen[i, j] = True
        queue = deque([(i, j)])
        while queue:
            a, b = queue.popleft()
            for c, d in ((a + 1, b), (a - 1, b), (a, b + 1), (a, b - 1)):
                if 0 <= c < nx and 0 <= d < ny and free[c, d] and not seen[c, d]:
                    seen[c, d] = True
                    queue.append((c, d))
    return count


def scatter(rng, area, keep_out, obstacles, count, scale=1.0):
    """
    Place 'count' random objects of the library (any kind, each type as likely),
    scaled by 'scale' (for a scaled-down world).
    Returns ([[type, x, y, yaw], ...], number of objects that did not fit).
    """
    library = load_library()
    types = sorted(library)
    grid = Grid(*area)
    blocked = grid.mask(keep_out) | grid.mask(obstacles)
    fixed = grid.mask(obstacles, ROBOT_RADIUS) | grid.border(ROBOT_RADIUS)
    baseline = components(~fixed)

    placed, shapes, skipped = [], [], 0
    for _ in range(count):
        type_ = rng.choice(types)
        sx, sy = (s * scale for s in library[type_]['size'][:2])
        radius = math.hypot(sx, sy) / 2
        for _ in range(ATTEMPTS):
            x = rng.uniform(area[0] + radius, area[1] - radius)
            y = rng.uniform(area[2] + radius, area[3] - radius)
            if any(math.hypot(x - px, y - py) < radius + pr + GAP for _, px, py, pr in shapes):
                continue
            if grid.mask([('circle', x, y, radius)], GAP)[blocked].any():
                continue
            candidate = shapes + [('circle', x, y, radius)]
            free = ~(fixed | grid.mask(candidate, ROBOT_RADIUS))
            if components(free) > baseline:
                continue
            shapes = candidate
            placed.append([type_, round(x, 3), round(y, 3), round(rng.uniform(-math.pi, math.pi), 3)])
            break
        else:
            skipped += 1
    return placed, skipped


def scatter_from_args(args, area, keep_out, obstacles, label, scale=1.0):
    """
    Scatter objects for world arguments 'objects' (their number) and 'objects_seed'
    (empty for a new random seed) and print the seed, so the placement can be
    repeated with objects_seed:=<seed>.
    """
    seed = args.get('objects_seed') or str(random.SystemRandom().randrange(1000000))
    rng = random.Random(f'objects:{seed}')
    placed, skipped = scatter(rng, area, keep_out, obstacles, int(args.get('objects', 0)), scale)
    note = f', {skipped} did not fit' if skipped else ''
    print(f'[{label}] objects_seed:={seed}: {len(placed)} objects placed{note}', flush=True)
    return placed
