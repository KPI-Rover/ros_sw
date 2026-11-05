"""
Garden: rows of fruit trees on grass with patches of loose soil and mud, pits
and puddles, and a field road around, on uneven terrain (terrain.py).

The garden is modeled in real sizes, then scaled down to the robot, so the
trees are 4 robot heights tall (trees.SCALE). The robot collides with
the height map and sees meshes of the same heights, one for each ground;
grass and the surroundings up to the horizon are only seen. Axes: x east, y north.
"""

import dataclasses
import math
import os
import shutil

import numpy as np
import yaml
from PIL import Image

from kpi_rover_worlds import scatter, sdf

import terrain      # garden/terrain.py, next to this file

CACHE_DIR = os.path.join(os.path.expanduser('~'), '.cache', 'kpi_rover', 'garden')
CACHE_KEEP = 6      # built terrains kept in the cache; older ones are removed

# Scattered objects: the square they lie in (m from the center), free room
# around pits, puddles and start points (m), and the height above the ground
# they are dropped from (m).
OBJECTS_AREA = 9.4
OBJECTS_CLEARANCE = 0.15
START_CLEARANCE = 0.45
OBJECTS_LIFT = 0.01
STEM_HEIGHT = 0.45      # tree stem collisions


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
    z_map = np.asarray(Image.open(info['heightmap']), float) / 65535 * (info['z_max'] - info['z_min']) + info['z_min']
    half, step = info['size'] / 2, info['size'] / (len(z_map) - 1)

    def ground_z(px, py):
        return terrain.bilinear(z_map, (px + half) / step, (half - py) / step)

    for obj in objects:
        kind, x, y, yaw = obj[:4]
        # The highest ground under the object footprint, so it never starts inside the ground.
        r = math.hypot(*sdf.OBJECTS[kind]['size'][:2]) / 2 * terrain.SCALE
        a = np.linspace(0, 2 * math.pi, 8, endpoint=False)
        z = float(ground_z(np.append(x + r * np.cos(a), x), np.append(y + r * np.sin(a), y)).max())
        obj += [round(z + OBJECTS_LIFT, 4), terrain.SCALE]
    return objects


def clean_cache(keep):
    """Remove the least recently used terrains from the cache, keeping 'keep' of them."""
    dirs = [os.path.join(CACHE_DIR, d) for d in os.listdir(CACHE_DIR) if d.startswith('terrain_')]
    dirs.sort(key=lambda d: os.path.getmtime(os.path.join(d, 'terrain.yaml')) if os.path.isfile(
        os.path.join(d, 'terrain.yaml')) else 0)
    for d in dirs[:-keep]:
        shutil.rmtree(d, ignore_errors=True)


def mesh_geometry(parent, uri):
    sdf.el(sdf.el(sdf.el(parent, 'geometry'), 'mesh'), 'uri', uri)


def ground_visuals(link, grounds):
    # Grounds cast no shadows: on this flat land they would hardly show, but cost much rendering.
    for ground in grounds:
        visual = sdf.el(link, 'visual', name=f"ground_{ground['name']}")
        sdf.el(visual, 'cast_shadows', False)
        mesh_geometry(visual, ground['mesh'])
        sdf.material(visual, ground['color'], ground['texture'])


def build(args):
    options = terrain.Options(**{f.name: args[f.name] for f in dataclasses.fields(terrain.Options)})
    path = terrain.build(CACHE_DIR, args['relief'], args['terrain_seed'], options)
    os.utime(path)      # marks it as recently used
    clean_cache(CACHE_KEEP)
    with open(path) as f:
        info = yaml.safe_load(f)

    # Afternoon sun, low enough to show the relief.
    root, world = sdf.world('garden', shadows=args['shadows'], skybox=True, ambient=0.35, sun=(-0.65, 0.55, -0.45))
    link = sdf.static_model(world, 'terrain')
    collision = sdf.el(link, 'collision', name='collision')
    sdf.el(collision, 'pose', 0, 0, info['z_min'], 0, 0, 0)     # DART ignores the height map <pos>
    heightmap = sdf.el(sdf.el(collision, 'geometry'), 'heightmap')
    sdf.el(heightmap, 'uri', info['heightmap'])
    sdf.el(heightmap, 'size', info['size'], info['size'], info['z_max'] - info['z_min'])
    sdf.friction(collision, info['mu'])
    # Mud and wheel tracks have their own friction: collision meshes 1 cm above the height map there.
    for ground in info['grounds']:
        if ground['collision']:
            collision = sdf.el(link, 'collision', name=f"ground_{ground['name']}")
            mesh_geometry(collision, ground['mesh'])
            sdf.friction(collision, ground['mu'])
    ground_visuals(link, info['grounds'])
    features = info['features']
    if features['water']:
        # Only seen, the robot drives through it to the bottom.
        visual = sdf.el(link, 'visual', name='water')
        sdf.el(visual, 'cast_shadows', False)
        sdf.el(visual, 'visibility_flags', sdf.VISIBLE_NO_LIDAR)
        mesh_geometry(visual, features['water'])
        material = sdf.el(visual, 'material')
        sdf.el(material, 'ambient', *features['water_color'], 1)
        sdf.el(material, 'diffuse', *features['water_color'], 1)
        sdf.el(material, 'specular', '0.6 0.6 0.6 1')
        metal = sdf.el(sdf.el(material, 'pbr'), 'metal')
        sdf.el(metal, 'metalness', '0.0')
        sdf.el(metal, 'roughness', features['water_roughness'])

    trees = info['trees']
    link = sdf.static_model(world, 'trees')
    for k, stem in enumerate(trees['stems']):
        # Up to above the robot, where the crowns begin.
        collision = sdf.el(link, 'collision', name=f'stem_{k}')
        sdf.el(collision, 'pose', stem['x'], stem['y'], stem['z'] + STEM_HEIGHT / 2, 0, 0, 0)
        cylinder = sdf.el(sdf.el(collision, 'geometry'), 'cylinder')
        sdf.el(cylinder, 'radius', stem['radius'])
        sdf.el(cylinder, 'length', STEM_HEIGHT)
    for wood in trees['wood']:
        visual = sdf.el(link, 'visual', name=wood['name'])
        mesh_geometry(visual, wood['mesh'])
        sdf.material(visual, wood['color'], wood['texture'])
    if trees['leaves']:
        # They cast the crown shadows; above the lidar scan plane.
        visual = sdf.el(link, 'visual', name='leaves')
        sdf.el(visual, 'visibility_flags', sdf.VISIBLE_NO_LIDAR)
        mesh_geometry(visual, trees['leaves'])

    link = sdf.static_model(world, 'grass')
    for plant in info['plants']:
        visual = sdf.el(link, 'visual', name=plant['name'])
        sdf.el(visual, 'cast_shadows', False)
        sdf.el(visual, 'visibility_flags', sdf.VISIBLE_NO_LIDAR)
        mesh_geometry(visual, plant['mesh'])
    ground_visuals(sdf.static_model(world, 'surroundings'), info['surroundings'])

    for i, (kind, x, y, yaw, z, scale) in enumerate(scatter_objects(args, info) if args['objects'] else []):
        sdf.scatter_object(world, f'{kind}_{i}', kind, x, y, yaw, z, scale)
    for point in info['start_points']:
        sdf.start_point(world, point['name'], point['x'], point['y'], point['yaw'], point['z'])
    return root
