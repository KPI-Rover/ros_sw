"""
Generator of the garden world (garden.sdf.xacro).

The launch file calls prepare() before processing the world. It builds the
terrain for the world arguments (see garden.world.yaml and terrain.py) into a
cache directory, where it is reused on the next launches, and sets the
'terrain' mapping to its terrain.yaml. It also scatters the objects
('objects', 'objects_seed') over the garden, around the trees, pits,
puddles and start points, each at the ground height under it, and sets the
'layout' mapping to a file listing them. It can also be run directly to build a
terrain:
    python3 generator.py --relief steppe --seed 11 --arg grass:=0.5 --arg mud:=0.2
"""

import argparse
import math
import os
import shutil
import sys
import tempfile

import numpy as np
import yaml
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lib'))
import scatter  # noqa: E402
import terrain  # noqa: E402
import world_info  # noqa: E402

WORLD = os.path.join(HERE, '..', 'garden.sdf.xacro')
# World arguments passed to terrain.configure().
TERRAIN_OPTIONS = ('grass', 'soil', 'mud', 'pits', 'puddles', 'tree_rows', 'grass_friction', 'mud_friction',
                   'track_friction')

CACHE_DIR = os.path.join(os.path.expanduser('~'), '.cache', 'kpi_rover', 'garden')
CACHE_KEEP = 6      # built terrains kept in the cache; older ones are removed

# Scattered objects: the square they lie in (m from the center), free room
# around pits, puddles and start points (m), and the height above the ground
# they are dropped from (m).
OBJECTS_AREA = 9.4
OBJECTS_CLEARANCE = 0.15
START_CLEARANCE = 0.45
OBJECTS_LIFT = 0.01


class Heights:
    """Ground heights of a built terrain, from its height map (row 0 is the +y edge)."""

    def __init__(self, info):
        self.z = np.asarray(Image.open(info['heightmap']), float) / 65535 * (info['z_max'] - info['z_min']) + \
            info['z_min']
        self.size = info['size']

    def at(self, x, y):
        n = self.z.shape[0] - 1
        col = np.clip((np.asarray(x) / self.size + 0.5) * n, 0, n)
        row = np.clip((0.5 - np.asarray(y) / self.size) * n, 0, n)
        c0, r0 = np.floor(col).astype(int).clip(0, n - 1), np.floor(row).astype(int).clip(0, n - 1)
        fc, fr = col - c0, row - r0
        z = self.z
        return ((z[r0, c0] * (1 - fc) + z[r0, c0 + 1] * fc) * (1 - fr) +
                (z[r0 + 1, c0] * (1 - fc) + z[r0 + 1, c0 + 1] * fc) * fr)


def scatter_objects(args, info):
    """[[type, x, y, yaw, z, scale]] of the objects scattered over the garden for the world arguments."""
    features = info.get('features', {})
    keep_out = [('circle', p['x'], p['y'], p['radius'] + OBJECTS_CLEARANCE) for p in features.get('pits', [])]
    keep_out += [('circle', p['x'], p['y'], p['length'] / 2 + OBJECTS_CLEARANCE) for p in features.get('puddles', [])]
    keep_out += [('circle', p['x'], p['y'], START_CLEARANCE) for p in info.get('start_points', [])]
    obstacles = [('circle', s['x'], s['y'], s['radius']) for s in info.get('trees', {}).get('stems', [])]
    area = (-OBJECTS_AREA, OBJECTS_AREA, -OBJECTS_AREA, OBJECTS_AREA)
    # The objects are scaled down with the whole garden.
    objects = scatter.scatter_from_args(args, area, keep_out, obstacles, 'garden', terrain.SCALE)
    heights = Heights(info)
    library = scatter.load_library()
    for obj in objects:
        kind, x, y, yaw = obj[:4]
        # The highest ground under the object footprint, so it never starts inside the ground.
        r = math.hypot(*library[kind]['size'][:2]) / 2 * terrain.SCALE
        a = np.linspace(0, 2 * math.pi, 8, endpoint=False)
        z = max(float(heights.at(x, y)), float(heights.at(x + r * np.cos(a), y + r * np.sin(a)).max()))
        obj += [round(z + OBJECTS_LIFT, 4), terrain.SCALE]
    return objects


def clean_cache(keep):
    """Remove the least recently used terrains from the cache, keeping 'keep' of them."""
    dirs = [os.path.join(CACHE_DIR, d) for d in os.listdir(CACHE_DIR) if d.startswith('terrain_')]
    dirs.sort(key=lambda d: os.path.getmtime(os.path.join(d, 'terrain.yaml')) if os.path.isfile(
        os.path.join(d, 'terrain.yaml')) else 0)
    for d in dirs[:-keep]:
        shutil.rmtree(d, ignore_errors=True)


def prepare(args):
    """Launch hook: build the terrain and return the xacro mappings with 'terrain' set."""
    if args.get('terrain'):
        return args
    specs = world_info.describe(WORLD)['args']
    values = {}
    for name, spec in specs.items():
        try:
            values[name] = world_info.parse(spec, args[name])
        except ValueError as e:
            raise ValueError(f"{name}: {e}.")
    terrain.configure(**{name: values[name] for name in TERRAIN_OPTIONS})
    path = terrain.build(CACHE_DIR, values['relief'], values['terrain_seed'])
    os.utime(path)      # marks it as recently used
    clean_cache(CACHE_KEEP)
    mappings = dict(args, terrain=path)
    if values['objects']:
        with open(path) as f:
            info = yaml.safe_load(f)
        objects = scatter_objects(args, info)
        with tempfile.NamedTemporaryFile('w', prefix='kpi_rover_garden_', suffix='.yaml', delete=False) as f:
            yaml.safe_dump(dict(objects=objects), f, default_flow_style=None)
        mappings['layout'] = f.name
    return mappings


def main():
    parser = argparse.ArgumentParser(description='Build the garden terrain and print its terrain.yaml.')
    parser.add_argument('--relief', default='steppe', choices=terrain.RELIEFS)
    parser.add_argument('--seed', type=int, default=11)
    parser.add_argument('--arg', action='append', default=[], help='other world argument, name:=value')
    args = parser.parse_args()
    values = dict(world_info.xacro_defaults(WORLD), relief=args.relief, terrain_seed=str(args.seed))
    values.update(kv.split(':=', 1) for kv in args.arg)
    path = prepare(values)['terrain']
    print(path)
    with open(path) as f:
        print(yaml.safe_load(f))


if __name__ == '__main__':
    main()
