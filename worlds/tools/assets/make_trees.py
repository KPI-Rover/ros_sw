"""
Generate apple trees for the garden with the Sapling Tree Gen add-on of
Blender (a parametric tree generator, extensions.blender.org) and convert them
like the other tree models (convert_trees.py). Run with Blender:

    blender -b --python make_trees.py -- [names...]

Each tree of TREES is a set of Sapling parameters for a pruned orchard apple
tree: a clear stem about a quarter of the height, an open bowl of 4-6 spreading
scaffold branches, shoots on them turning up at their ends, a round crown.
The wood is a smooth curve mesh (no decimation, so no broken branches); the
leaves are Sapling leaf rectangles, which the garden turns into leaf cluster
cards. The trees are our own, so they can be kept in the repository.

Install the add-on once:
    blender --online-mode --command extension install -s sapling_tree_gen --enable
"""

import os
import sys

import bpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import convert_trees  # noqa: E402

# Shared parameters of the apple trees (see the Sapling add-on for their meaning).
APPLE = dict(
    levels=3,
    scale=3.0, scaleV=0.25,
    baseSize=0.3, baseSize_s=0.25, baseSplits=0,
    shape='2',                              # hemispherical crown
    branches=(0, 16, 18, 10),               # scaffolds, shoots on them, twigs on shoots
    length=(1.0, 0.78, 0.5, 0.4),
    lengthV=(0.0, 0.15, 0.15, 0.1),
    downAngle=(90.0, 62.0, 45.0, 40.0),
    downAngleV=(0.0, -15.0, 15.0, 10.0),    # negative: lower scaffolds spread wider
    rotate=(99.5, 137.5, 137.5, 137.5),
    rotateV=(15.0, 20.0, 20.0, 0.0),
    curve=(0.0, -20.0, -10.0, 0.0),         # scaffolds bend out, then
    curveBack=(0.0, 35.0, 10.0, 0.0),       # their ends turn up again
    curveV=(10.0, 25.0, 40.0, 0.0),
    curveRes=(6, 5, 2, 1),
    segSplits=(0.0, 0.15, 0.1, 0.0),
    splitAngle=(0.0, 20.0, 20.0, 0.0),
    attractUp=(0.0, 0.2, 0.4, 0.0),
    attractOut=(0.0, 0.2, 0.1, 0.0),
    ratio=0.035, ratioPower=1.3, minRadius=0.003, rootFlare=1.15,
    taper=(1.0, 1.0, 1.0, 1.0), autoTaper=True,
    bevel=True, bevelRes=0, resU=2,         # 6 sides around a branch: smooth enough at the robot's size
    showLeaves=True, leafShape='rect', leaves=34,
    leafScale=0.3, leafScaleX=0.75, leafScaleV=0.25,
    leafDownAngle=50.0, leafDownAngleV=15.0, leafRotate=137.5, leafangle=-20.0,
    leafDist='6', horzLeaves=False,
    prune=False, makeMesh=False, useArm=False,
)
# Trees: name and differences from APPLE.
TREES = {
    'apple_1': dict(seed=1),
    'apple_2': dict(seed=7, branches=(0, 14, 18, 10), downAngle=(90.0, 66.0, 45.0, 40.0)),
    'apple_3': dict(seed=13, branches=(0, 18, 16, 10), baseSize=0.34),
    'apple_4': dict(seed=21, branches=(0, 15, 20, 10), length=(1.0, 0.85, 0.5, 0.4)),
}
LEAF_CARDS = 950        # leaf cards kept per tree


def generate(params):
    """Generate a Sapling tree; return one mesh object with 'Bark' on the wood and 'Leaves' on the leaves."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    # Factory settings turn add-ons off.
    bpy.ops.preferences.addon_enable(module='bl_ext.blender_org.sapling_tree_gen')
    bpy.ops.curve.tree_add(**params)
    curve = next(o for o in bpy.context.scene.objects if o.type == 'CURVE')
    leaves = next((o for o in bpy.context.scene.objects if o.type == 'MESH'), None)
    bark, leaf = bpy.data.materials.new('Bark'), bpy.data.materials.new('Leaves')
    bpy.context.view_layer.objects.active = curve
    for o in bpy.context.scene.objects:
        o.select_set(o is curve)
    bpy.ops.object.convert(target='MESH')
    wood = bpy.context.view_layer.objects.active
    wood.data.materials.clear()
    wood.data.materials.append(bark)
    wood.data.materials.append(leaf)
    if leaves:
        leaves.data.materials.clear()
        leaves.data.materials.append(bark)
        leaves.data.materials.append(leaf)
        for poly in leaves.data.polygons:
            poly.material_index = 1
        for o in bpy.context.scene.objects:
            o.select_set(o in (wood, leaves))
        bpy.context.view_layer.objects.active = wood
        bpy.ops.object.join()
    return wood


def main():
    names = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    for name in names or TREES:
        obj = generate(dict(APPLE, **TREES[name]))
        convert_trees.convert(obj, name, wood_triangles=0, leaf_cards_count=LEAF_CARDS,
                              source='make_trees.py (Sapling Tree Gen)')


main()
