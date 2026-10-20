"""
Convert a tree model from a .blend file (e.g. downloaded with BlenderKit) into
a light tree for the garden. Run with Blender:

    blender -b "<tree>.blend" --python convert_trees.py -- <name>

The tree is the mesh with a 'Bark' material and leaf cards (any other
material). Its wood is simplified to about WOOD_TRIANGLES triangles and
written with its bark texture as media/models/<name>/<name>_wood.gltf. The leaf
cards are kept as rectangles only (center and two half-axes), thinned out to
about LEAF_CARDS and enlarged to cover the same crown, into
media/models/<name>/<name>_leaves.yaml: the garden gives them its own leaf
texture (trees.py), as the leaf shaders of such models are procedural and
can't be exported. The tree stands at the origin, in real meters, Z up,
scaled to the real tree height of params.yaml and narrowed to MAX_WIDTH.
garden/trees.yaml lists the converted trees.
"""

import math
import os
import sys

import bmesh
import bpy
import numpy as np

try:
    import yaml
except ImportError:
    # Blender's Python has no PyYAML; the system one is pure Python and works.
    sys.path.append('/usr/lib/python3/dist-packages')
    import yaml

WORLDS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
GARDEN_DIR = os.path.join(WORLDS_DIR, 'garden')
MODELS_DIR = os.path.join(WORLDS_DIR, 'media', 'models')
sys.path.insert(0, os.path.join(WORLDS_DIR, 'lib'))
import gltf  # noqa: E402

with open(os.path.join(GARDEN_DIR, 'params.yaml')) as f:
    PARAMS = yaml.safe_load(f)

WOOD_TRIANGLES = 1000
LEAF_CARDS = 2500
MAX_WIDTH = 4.5     # real crown width, m; wider trees are narrowed
BARK_SIZE = 512     # bark texture side, px


def tree_object():
    meshes = [o for o in bpy.context.scene.objects if o.type == 'MESH'
              and any(m and m.name == 'Bark' for m in o.data.materials)]
    if not meshes:
        sys.exit('No mesh with a Bark material found.')
    return max(meshes, key=lambda o: len(o.data.polygons))


def split(obj):
    """Evaluated mesh of the object in world coordinates, split into (wood bmesh, leaf bmesh)."""
    dg = bpy.context.evaluated_depsgraph_get()
    me = obj.evaluated_get(dg).to_mesh()
    me.transform(obj.matrix_world)
    names = [m.name if m else '' for m in me.materials]
    parts = []
    for wood in (True, False):
        bm = bmesh.new()
        bm.from_mesh(me)
        drop = [f for f in bm.faces if (names[f.material_index] == 'Bark') != wood]
        bmesh.ops.delete(bm, geom=drop, context='FACES')
        parts.append(bm)
    return parts


def simplify_wood(bm, target=WOOD_TRIANGLES):
    """Decimate the wood to about 'target' triangles (0: keep it as is) with a temporary object; return its mesh."""
    me = bpy.data.meshes.new('wood')
    bm.to_mesh(me)
    obj = bpy.data.objects.new('wood', me)
    bpy.context.scene.collection.objects.link(obj)
    triangles = sum(len(p.vertices) - 2 for p in me.polygons)
    if target and triangles > target:
        mod = obj.modifiers.new('decimate', 'DECIMATE')
        mod.ratio = target / triangles
        dg = bpy.context.evaluated_depsgraph_get()
        me = bpy.data.meshes.new_from_object(obj.evaluated_get(dg))
    me.calc_loop_triangles()
    return me


def mesh_arrays(me):
    """Triangles of a mesh with split normals and UVs: positions, normals, uvs, faces."""
    loops = [l for t in me.loop_triangles for l in t.loops]
    p = np.array([me.vertices[me.loops[l].vertex_index].co[:] for l in loops], np.float32)
    n = np.array([t.split_normals[k][:] for t in me.loop_triangles for k in range(3)], np.float32)
    if me.uv_layers.active:
        uv_layer = me.uv_layers.active.data
        uv = np.array([uv_layer[l].uv[:] for l in loops], np.float32)
    else:
        uv = np.zeros((len(loops), 2), np.float32)
    uv[:, 1] = 1 - uv[:, 1]     # glTF v points down
    return p, n, uv, np.arange(len(loops)).reshape(-1, 3)


def leaf_cards(bm):
    """Leaf cards (connected leaf faces) as rectangles: centers (N, 3), half-axes u, v (N, 3) each."""
    bm.faces.ensure_lookup_table()
    seen, cards = set(), []
    for f in bm.faces:
        if f.index in seen:
            continue
        stack, island = [f], []
        seen.add(f.index)
        while stack:
            g = stack.pop()
            island.append(g)
            for e in g.edges:
                for h in e.link_faces:
                    if h.index not in seen:
                        seen.add(h.index)
                        stack.append(h)
        pts = np.array([v.co[:] for g in island for v in g.verts])
        c = pts.mean(0)
        # Principal axes of the card plane and its extent along them.
        _, _, vt = np.linalg.svd(pts - c)
        a, b = vt[0], vt[1]
        cards.append((c, a * np.abs((pts - c) @ a).max(), b * np.abs((pts - c) @ b).max()))
    return (np.array([c[0] for c in cards]), np.array([c[1] for c in cards]), np.array([c[2] for c in cards]))


def main():
    convert(tree_object(), sys.argv[sys.argv.index('--') + 1])


def convert(obj, name, wood_triangles=WOOD_TRIANGLES, leaf_cards_count=LEAF_CARDS, source=None):
    """
    Convert a tree object (a 'Bark' material on the wood, any other on the leaf
    cards) into media/models/<name>/ and a trees.yaml entry.
    """
    out_dir = os.path.join(MODELS_DIR, name)
    os.makedirs(out_dir, exist_ok=True)
    wood_bm, leaf_bm = split(obj)
    centers, u, v = leaf_cards(leaf_bm)
    wood = simplify_wood(wood_bm, wood_triangles)
    p, n, uv, faces = mesh_arrays(wood)

    # Stand the tree at the origin: the middle of its lowest wood points.
    base = p[p[:, 2] < p[:, 2].min() + 0.05]
    origin = np.array([base[:, 0].mean(), base[:, 1].mean(), p[:, 2].min()])
    p -= origin
    centers -= origin
    height = max(p[:, 2].max(), (centers[:, 2] + np.abs(u[:, 2]) + np.abs(v[:, 2])).max())
    k = PARAMS['real_tree_height'] / height
    width = max(np.ptp(np.concatenate([p[:, 0], centers[:, 0]])), np.ptp(np.concatenate([p[:, 1], centers[:, 1]])))
    narrow = min(1.0, MAX_WIDTH / (width * k))
    s = np.array([k * narrow, k * narrow, k])
    p *= s
    n /= s                       # normals of a scaled mesh
    n /= np.linalg.norm(n, axis=1, keepdims=True)
    centers, u, v = centers * s, u * s, v * s

    # Fewer cards, each larger, to cover the same crown.
    rng = np.random.default_rng(1)
    keep = min(1.0, leaf_cards_count / len(centers))
    sel = rng.uniform(0, 1, len(centers)) < keep
    grow = 1 / math.sqrt(keep)
    centers, u, v = centers[sel], u[sel] * grow, v[sel] * grow

    # The garden gives the wood its own bark texture (trees.py); the model one is kept if there is one.
    bark = bpy.data.images.get('Bark')
    if bark:
        bark.scale(BARK_SIZE, BARK_SIZE)
        bark_file = f'{name}_bark.png'
        bark.filepath_raw = os.path.join(out_dir, bark_file)
        bark.file_format = 'PNG'
        bark.save()
        material = dict(name=f'{name}_bark', pbrMetallicRoughness=dict(
            baseColorTexture=dict(index=0), metallicFactor=0.0, roughnessFactor=0.9))
        gltf.write(os.path.join(out_dir, f'{name}_wood.gltf'), p, n, uv, faces, material=material, images=[bark_file])
    else:
        gltf.write(os.path.join(out_dir, f'{name}_wood.gltf'), p, n, uv, faces)
    with open(os.path.join(out_dir, f'{name}_leaves.yaml'), 'w') as f:
        f.write(f'# Leaf cards of {name}, written by tools/assets/convert_trees.py: centers and half-axes, real m.\n')
        yaml.safe_dump(dict(centers=np.round(centers, 4).tolist(), u=np.round(u, 4).tolist(),
                            v=np.round(v, 4).tolist()), f, default_flow_style=True, width=1 << 20)

    # Stem: the wood below the first branches, for the collision.
    low = p[p[:, 2] < 0.3]
    stem_radius = float(np.percentile(np.hypot(low[:, 0], low[:, 1]), 90))
    path = os.path.join(GARDEN_DIR, 'trees.yaml')
    models = {}
    if os.path.isfile(path):
        with open(path) as f:
            models = (yaml.safe_load(f) or {}).get('models', {})
    models[name] = dict(wood=f'models/{name}/{name}_wood.gltf', leaves=f'models/{name}/{name}_leaves.yaml',
                        source=source or os.path.basename(bpy.data.filepath), stem_radius=round(stem_radius, 3),
                        triangles=int(len(faces)), cards=int(len(centers)))
    with open(path, 'w') as f:
        f.write('# Tree models for the garden, written by convert_trees.py and make_trees.py (real sizes, m).\n')
        yaml.safe_dump(dict(models=models), f, sort_keys=False)
    print(f'{name}: {len(faces)} wood triangles, {len(centers)} leaf cards, stem radius {stem_radius:.3f} m, '
          f'narrowed x{narrow:.2f}')


if __name__ == '__main__':
    main()
