"""
Trees of the garden: apple trees generated with Blender's Sapling Tree Gen
(make_trees.py), or other converted models (convert_trees.py), listed in
trees.yaml, planted in rows and scaled down to the robot (see params.yaml).

Each tree is one of the models, turned and sized at random; a few are
missing or younger, as in a real orchard. The wood gets the bark photo
texture of worlds/media/textures in real size, each triangle projected onto
its own plane with v along the branch. The leaf cards of the model get the
leaf cluster pictures (fetch_plants.py), a random one on each card, mirrored
at random. Both sides of a card are separate faces with normals pointing
away from the crown middle, so the crown is lit as one rounded mass from
any side (a two-sided material would make the back sides dark).

All trees of a model go into one wood mesh, and all leaves into one mesh, to
keep the drawing cheap. The robot collides with the stems only: cylinders up
to above the robot, where the crowns begin.

Trees stand in rows along x, row_spacing (params.yaml) apart, TREE_SPACING
apart in a row.
"""

import math
import os
import shutil

import numpy as np
import yaml

import gltf

HERE = os.path.dirname(os.path.abspath(__file__))
MEDIA_DIR = os.path.join(HERE, '..', 'media')

with open(os.path.join(HERE, 'params.yaml')) as f:
    PARAMS = yaml.safe_load(f)
with open(os.path.join(HERE, 'trees.yaml')) as f:
    MODELS = yaml.safe_load(f)['models']
SCALE = PARAMS['robot_height'] * PARAMS['tree_height_robots'] / PARAMS['real_tree_height']

GARDEN_HALF_X = 6.0     # half length of the rows, world m
ROWS = 6
TREE_SPACING = 4.5      # real distance between trees in a row, m
JITTER = 0.15           # real random offset of a tree from its place, m
SIZE = (0.9, 1.1)       # random size of a tree
MISSING = 0.04          # share of missing trees
YOUNG = 0.08            # share of young trees
YOUNG_SIZE = 0.65
BARK = 'tree_bark'      # bark photo texture in worlds/media/textures
BARK_TILE = 2.15        # real size one bark texture repeat covers, m (see textures/CREDITS.md)


def layout(seed):
    """Tree places: [(x, y, young)] in world m."""
    rng = np.random.default_rng(seed * 7 + 9)
    spacing_y = PARAMS['row_spacing'] * SCALE
    spacing_x = TREE_SPACING * SCALE
    count_x = int(2 * GARDEN_HALF_X / spacing_x) + 1
    places = []
    for row in range(ROWS):
        y = (row - (ROWS - 1) / 2) * spacing_y
        for i in range(count_x):
            x = (i - (count_x - 1) / 2) * spacing_x
            r = rng.uniform(0, 1)
            if r < MISSING:
                continue
            jx, jy = rng.normal(0, JITTER * SCALE, 2)
            places.append((x + jx, y + jy, r < MISSING + YOUNG))
    return places


def bark_uvs(p, f):
    """UVs in real meters: each triangle projected onto its plane, v along the steepest direction in it."""
    uv = np.zeros((len(p), 2))
    a, b, c = p[f[:, 0]], p[f[:, 1]], p[f[:, 2]]
    n = np.cross(b - a, c - a)
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    side = np.cross([0.0, 0.0, 1.0], n)
    flat = np.linalg.norm(side, axis=1) < 0.2        # facing up or down: any direction will do
    side[flat] = [1.0, 0.0, 0.0]
    side /= np.linalg.norm(side, axis=1, keepdims=True)
    up = np.cross(n, side)
    for k in range(3):
        q = p[f[:, k]]
        uv[f[:, k]] = np.stack([(q * side).sum(1), -(q * up).sum(1)], 1) / BARK_TILE
    return uv


def load(name):
    """Wood arrays with bark UVs, and leaf cards of a tree model."""
    model = MODELS[name]
    g, buffers = gltf.read(os.path.join(MEDIA_DIR, model['wood']))
    p, n, _, f = gltf.mesh_data(g, buffers, g['nodes'][0]['mesh'])
    # Vertices are separate for each triangle (convert_trees.py), so each can have its own projection.
    wood = (p, n, bark_uvs(p, f), f)
    with open(os.path.join(MEDIA_DIR, model['leaves'])) as f:
        leaves = yaml.safe_load(f)
    cards = tuple(np.array(leaves[k], float) for k in ('centers', 'u', 'v'))
    return wood, cards


def build(out_dir, key, z_at, seed, clusters):
    """
    Build all trees: write the wood and leaves glTFs into out_dir and return
    {wood: [{name, mesh, texture, color}], leaves, stems: [{x, y, z, radius}], count, cards}.
    """
    rng = np.random.default_rng(seed * 7 + 10)
    names = sorted(MODELS)
    models = {name: load(name) for name in names}
    places = layout(seed)
    if not places:
        return dict(wood=[], leaves='', stems=[], count=0, cards=0)
    zs = z_at(np.array([p[0] for p in places]), np.array([p[1] for p in places]))
    wood_parts = {name: [] for name in names}
    centers, us, vs, mids, stems = [], [], [], [], []
    for (x, y, young), z in zip(places, zs):
        name = names[rng.integers(len(names))]
        (p, n, uv, f), (c, u, v) = models[name]
        k = rng.uniform(*SIZE) * (YOUNG_SIZE if young else 1.0) * SCALE
        yaw = rng.uniform(0, 2 * math.pi)
        cs, sn = math.cos(yaw), math.sin(yaw)
        rot = np.array([[cs, -sn, 0], [sn, cs, 0], [0, 0, 1]])
        offset = np.array([x, y, z - 0.01])     # a little into the ground
        wood_parts[name].append((p @ rot.T * k + offset, n @ rot.T, uv, f))
        c = c @ rot.T * k + offset
        middle = c.mean(0)
        centers.append(c)
        us.append(u @ rot.T * k)
        vs.append(v @ rot.T * k)
        mids.append(np.repeat([middle], len(c), 0))
        stems.append(dict(x=float(x), y=float(y), z=float(z), radius=round(float(MODELS[name]['stem_radius'] * k), 4)))

    wood = []
    for name, parts in wood_parts.items():
        if not parts:
            continue
        offsets = np.cumsum([0] + [len(part[0]) for part in parts[:-1]])
        path = os.path.join(out_dir, f'trees_{name}_{key}.gltf')
        gltf.write(path, np.concatenate([part[0] for part in parts]), np.concatenate([part[1] for part in parts]),
                   np.concatenate([part[2] for part in parts]),
                   np.concatenate([part[3] + o for part, o in zip(parts, offsets)]))
        wood.append(dict(name=name, mesh=path, texture=BARK, color=[1.0, 1.0, 1.0]))

    # Leaf cards: two-sided rectangles with a random cluster picture along their long side, mirrored at random.
    c, u, v, mid = (np.concatenate(a) for a in (centers, us, vs, mids))
    m = len(c)
    long_v = np.linalg.norm(v, axis=1) > np.linalg.norm(u, axis=1)
    u[long_v], v[long_v] = v[long_v].copy(), u[long_v].copy()
    u *= rng.choice([-1, 1], (m, 1))
    v *= rng.choice([-1, 1], (m, 1))
    corners = np.stack([c - u - v, c + u - v, c + u + v, c - u + v], 1)
    normals = corners - mid[:, None, :]
    normals[..., 2] += 0.3 * np.linalg.norm(normals, axis=-1)      # crowns are lit more from above
    normals /= np.linalg.norm(normals, axis=-1, keepdims=True)
    variant = rng.integers(0, clusters['variants'], m)
    u0, u1 = variant / clusters['variants'], (variant + 1) / clusters['variants']
    uvs = np.stack([np.stack([u0, np.ones(m)], -1), np.stack([u1, np.ones(m)], -1),
                    np.stack([u1, np.zeros(m)], -1), np.stack([u0, np.zeros(m)], -1)], 1)
    # Both sides as separate faces with the same normals.
    corners = np.concatenate([corners, corners])
    normals = np.concatenate([normals, normals])
    uvs = np.concatenate([uvs, uvs])
    q = np.arange(m)[:, None] * 4
    back = q + m * 4
    faces = np.concatenate([q + [0, 1, 2], q + [0, 2, 3], back + [0, 2, 1], back + [0, 3, 2]])
    atlas = os.path.basename(clusters['atlas'])
    shutil.copy(os.path.join(MEDIA_DIR, clusters['atlas']), os.path.join(out_dir, atlas))
    material = dict(name='apple_leaves', alphaMode='MASK', alphaCutoff=0.5,
                    pbrMetallicRoughness=dict(baseColorTexture=dict(index=0), metallicFactor=0.0, roughnessFactor=0.8))
    leaves = os.path.join(out_dir, f'trees_leaves_{key}.gltf')
    gltf.write(leaves, corners.reshape(-1, 3), normals.reshape(-1, 3), uvs.reshape(-1, 2), faces,
               material=material, images=[atlas])

    return dict(wood=wood, leaves=leaves, stems=stems, count=len(places), cards=m)
