"""
Download small object models from Poly Haven (CC0) for the object library and
prepare them for the worlds: glTF meshes with textures reduced to
TEXTURE_SIZE px in models/<asset>/, models/CREDITS.md and
include/objects.yaml entries with sizes for collisions.

glTF models are Y-up and Gazebo loads them as they are, so the visual is
rolled 90 degrees about X to make it Z-up, and sizes are written for that
frame: X, Y = -glTF Z, Z = glTF Y.

Usage (needs internet access):
    python3 fetch_models.py
"""

import io
import json
import math
import os
import urllib.request

import yaml
from PIL import Image

API = 'https://api.polyhaven.com'
TEXTURE_SIZE = 512

# Poly Haven asset: (mass in kg, empty containers, collision shape, roll in radians to lay the model on the floor).
MODELS = {
    'cardboard_box_01': (0.5, 'box', 0),
    'plastic_crate_02': (1.2, 'box', 0),
    'metal_toolbox': (3.0, 'box', 0),
    'wooden_crate_01': (6.0, 'box', 0),
    'wooden_bucket_01': (1.5, 'cylinder', 0),
    'old_tyre': (8.0, 'cylinder', math.pi / 2),
    'cement_bag': (25.0, 'box', 0),
    # Low objects: under the lidar scan plane (0.245 m), so only the camera sees them.
    'medical_box': (0.8, 'box', 0),
    'can_rusted': (0.4, 'cylinder', 0),
    'oil_tin': (0.6, 'box', 0),
}

WORLDS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
MODELS_DIR = os.path.join(WORLDS_DIR, 'media', 'models')
OBJECTS_YAML = os.path.join(WORLDS_DIR, 'include', 'objects.yaml')


def get(url, attempts=4):
    # Poly Haven rejects the default urllib user agent.
    request = urllib.request.Request(url, headers={'User-Agent': 'kpi_rover-fetch-models'})
    for attempt in range(attempts):
        try:
            with urllib.request.urlopen(request, timeout=60) as response:
                return response.read()
        except (OSError, TimeoutError):
            # Downloads of large files time out now and then: try again.
            if attempt == attempts - 1:
                raise


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def node_matrix(node):
    """4x4 transform of a glTF node."""
    if 'matrix' in node:
        m = node['matrix']
        return [[m[c * 4 + r] for c in range(4)] for r in range(4)]
    tx, ty, tz = node.get('translation', [0, 0, 0])
    qx, qy, qz, qw = node.get('rotation', [0, 0, 0, 1])
    sx, sy, sz = node.get('scale', [1, 1, 1])
    r = [[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
         [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
         [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]]
    return [[r[0][0] * sx, r[0][1] * sy, r[0][2] * sz, tx],
            [r[1][0] * sx, r[1][1] * sy, r[1][2] * sz, ty],
            [r[2][0] * sx, r[2][1] * sy, r[2][2] * sz, tz],
            [0, 0, 0, 1]]


def bounds(gltf):
    """Axis aligned bounds (min, max) of all meshes of the default scene in glTF coordinates."""
    lo, hi = [math.inf] * 3, [-math.inf] * 3
    identity = [[float(i == j) for j in range(4)] for i in range(4)]

    def visit(index, parent):
        node = gltf['nodes'][index]
        m = matmul(parent, node_matrix(node))
        if 'mesh' in node:
            for primitive in gltf['meshes'][node['mesh']]['primitives']:
                accessor = gltf['accessors'][primitive['attributes']['POSITION']]
                for corner in range(8):
                    p = [accessor['max'][i] if corner >> i & 1 else accessor['min'][i] for i in range(3)]
                    q = [sum(m[r][c] * p[c] for c in range(3)) + m[r][3] for r in range(3)]
                    for i in range(3):
                        lo[i], hi[i] = min(lo[i], q[i]), max(hi[i], q[i])
        for child in node.get('children', []):
            visit(child, m)

    for index in gltf['scenes'][gltf.get('scene', 0)]['nodes']:
        visit(index, identity)
    return lo, hi


def fetch(asset, roll):
    """Download the 1k glTF model, reduce its textures and return its bounds in the Gazebo frame after 'roll'."""
    files = json.loads(get(f'{API}/files/{asset}'))['gltf']['1k']['gltf']
    out_dir = os.path.join(MODELS_DIR, asset)
    os.makedirs(os.path.join(out_dir, 'textures'), exist_ok=True)
    gltf = json.loads(get(files['url']))

    for rel, entry in files.get('include', {}).items():
        data = get(entry['url'])
        if rel.endswith('.bin'):
            with open(os.path.join(out_dir, os.path.basename(rel)), 'wb') as f:
                f.write(data)
    for buffer in gltf.get('buffers', []):
        buffer['uri'] = os.path.basename(buffer['uri'])

    # Reduced textures get new names, which stay unique as they include the asset name.
    for image in gltf.get('images', []):
        data = get(files['include'][image['uri']]['url'])
        picture = Image.open(io.BytesIO(data)).convert('RGB')
        picture.thumbnail((TEXTURE_SIZE, TEXTURE_SIZE), Image.LANCZOS)
        name = os.path.basename(image['uri']).replace('_1k', f'_{TEXTURE_SIZE}')
        picture.save(os.path.join(out_dir, 'textures', name), quality=88)
        image['uri'] = f'textures/{name}'
        image['mimeType'] = 'image/jpeg'

    with open(os.path.join(out_dir, f'{asset}.gltf'), 'w') as f:
        json.dump(gltf, f)

    lo, hi = bounds(gltf)
    # glTF (x, y, z) is Gazebo (x, -z, y); a roll of 90 degrees turns Gazebo (x, y, z) to (x, -z, y) again.
    gz_lo, gz_hi = [lo[0], -hi[2], lo[1]], [hi[0], -lo[2], hi[1]]
    if abs(roll - math.pi / 2) < 1e-6:
        gz_lo, gz_hi = [gz_lo[0], -gz_hi[2], gz_lo[1]], [gz_hi[0], -gz_lo[2], gz_hi[1]]
    elif roll:
        raise ValueError('Only rolls of 0 and 90 degrees are supported.')
    size = [round(h - l, 4) for l, h in zip(gz_lo, gz_hi)]
    center = [round((h + l) / 2, 4) for l, h in zip(gz_lo, gz_hi)]
    return size, center


def main():
    credits = ['# Model credits', '',
               'Models from [Poly Haven](https://polyhaven.com), licensed [CC0](https://polyhaven.com/license).',
               f'Textures reduced to {TEXTURE_SIZE} px by fetch_models.py.', '',
               '| Model | Authors |', '|---|---|']
    # Keep credit rows of models downloaded by other scripts (e.g. garden/fetch_plants.py).
    path = os.path.join(MODELS_DIR, 'CREDITS.md')
    if os.path.isfile(path):
        credits += [line.rstrip('\n') for line in open(path)
                    if line.startswith('| [') and not any(f'[{asset}]' in line for asset in MODELS)]
    objects = {}
    for asset, (mass, shape, roll) in MODELS.items():
        size, center = fetch(asset, roll)
        info = json.loads(get(f'{API}/info/{asset}'))
        credits.append(f"| [{asset}](https://polyhaven.com/a/{asset}) | {', '.join(info.get('authors', {}))} |")
        objects[asset] = dict(kind='item', mesh=f'models/{asset}/{asset}.gltf', size=size, center=center,
                              roll=round(math.pi / 2 + roll, 4), mass=mass, shape=shape)
        print(f'{asset}: size {size} m, center {center}')

    with open(os.path.join(MODELS_DIR, 'CREDITS.md'), 'w') as f:
        f.write('\n'.join(credits) + '\n')

    # Keep entries that are not downloaded here (e.g. generated cones).
    existing = {}
    if os.path.isfile(OBJECTS_YAML):
        with open(OBJECTS_YAML) as f:
            existing = yaml.safe_load(f) or {}
    existing.update(objects)
    with open(OBJECTS_YAML, 'w') as f:
        f.write(OBJECTS_HEADER + '\n')
        yaml.safe_dump(existing, f, default_flow_style=None, sort_keys=False)


OBJECTS_HEADER = '''# Object library for scattered objects (see include/objects.xacro and lib/scatter.py).
# Item entries are written by tools/assets/fetch_models.py; cone and board entries are written
# by hand, their meshes are generated by tools/assets/gen_media.py.
#
# kind   - 'item' or 'cone'
# mesh   - visual mesh in worlds/media
# size   - bounding box size [x, y, z] in the model frame, m
# center - bounding box center in the model frame (after 'roll'), m
# roll   - rotation of the mesh about X, rad: 90 degrees to turn Y-up glTF meshes
#          to Z-up, plus any rotation to lay the object on the floor
# mass   - kg
# shape  - collision shape filling the bounding box: box, cylinder (vertical) or
#          cone (a base plate 3 cm high and a cylinder of 'radius' above it)
# texture - optional texture in worlds/media/textures for meshes without materials
'''


if __name__ == '__main__':
    main()
