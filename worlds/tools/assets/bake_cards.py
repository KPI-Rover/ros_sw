"""
Bake grass card pictures from 3D plant models (run by fetch_plants.py):

    blender -b --python bake_cards.py -- <sources dir> <out dir>

Every plant (mesh object) of every glTF model in <sources dir>/<asset>/ is
rendered alone from the side with an orthographic camera onto a transparent
background, as games bake impostors: <out dir>/<asset>__<object>.png. Its
real width and height go into <out dir>/cards.json. The plants keep their real
look: shapes, colors and shading of the 3D models.
"""

import glob
import json
import math
import os
import sys

import bpy
import mathutils

SIZE = 512          # picture height, px
MIN_HEIGHT = 0.06   # smaller plants are left out, m
MAX_HEIGHT = 0.7    # and larger ones too


def reset():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'
    scene.cycles.device = 'CPU'
    scene.cycles.samples = 24
    scene.render.film_transparent = True
    scene.view_settings.view_transform = 'Standard'
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.color_mode = 'RGBA'
    world = bpy.data.worlds.new('world')
    world.use_nodes = True
    world.node_tree.nodes['Background'].inputs['Color'].default_value = (0.55, 0.6, 0.65, 1)
    world.node_tree.nodes['Background'].inputs['Strength'].default_value = 0.9
    scene.world = world
    sun = bpy.data.objects.new('sun', bpy.data.lights.new('sun', 'SUN'))
    sun.data.energy = 3.0
    sun.rotation_euler = (math.radians(50), 0, math.radians(-30))
    scene.collection.objects.link(sun)
    cam = bpy.data.objects.new('cam', bpy.data.cameras.new('cam'))
    cam.data.type = 'ORTHO'
    scene.collection.objects.link(cam)
    scene.camera = cam
    return scene, cam


def flat_materials():
    """Make every material show its plain color (base color texture and alpha) without any lighting."""
    for mat in bpy.data.materials:
        if not mat.use_nodes:
            continue
        nodes, links = mat.node_tree.nodes, mat.node_tree.links
        bsdf = next((n for n in nodes if n.type == 'BSDF_PRINCIPLED'), None)
        out = next((n for n in nodes if n.type == 'OUTPUT_MATERIAL'), None)
        if not bsdf or not out:
            continue
        emission = nodes.new('ShaderNodeEmission')
        transparent = nodes.new('ShaderNodeBsdfTransparent')
        mix = nodes.new('ShaderNodeMixShader')
        color, alpha = bsdf.inputs['Base Color'], bsdf.inputs['Alpha']
        if color.is_linked:
            links.new(color.links[0].from_socket, emission.inputs['Color'])
        else:
            emission.inputs['Color'].default_value = color.default_value
        if alpha.is_linked:
            links.new(alpha.links[0].from_socket, mix.inputs['Fac'])
        else:
            mix.inputs['Fac'].default_value = alpha.default_value
        links.new(transparent.outputs[0], mix.inputs[1])
        links.new(emission.outputs[0], mix.inputs[2])
        links.new(mix.outputs[0], out.inputs['Surface'])


def bounds(obj):
    pts = [obj.matrix_world @ mathutils.Vector(c) for c in obj.bound_box]
    lo = mathutils.Vector([min(p[i] for p in pts) for i in range(3)])
    hi = mathutils.Vector([max(p[i] for p in pts) for i in range(3)])
    return lo, hi


def main():
    src, out = sys.argv[sys.argv.index('--') + 1:][:2]
    os.makedirs(out, exist_ok=True)
    cards = {}
    for path in sorted(glob.glob(os.path.join(src, '*', '*.gltf'))):
        asset = os.path.basename(os.path.dirname(path))
        scene, cam = reset()
        bpy.ops.import_scene.gltf(filepath=path)
        flat_materials()
        meshes = [o for o in scene.objects if o.type == 'MESH']
        for obj in meshes:
            lo, hi = bounds(obj)
            height = hi.z - lo.z
            if not MIN_HEIGHT <= height <= MAX_HEIGHT:
                continue
            for other in meshes:
                other.hide_render = other is not obj
            width = hi.x - lo.x
            center = (lo + hi) / 2
            # Looking along +y at the plant, with a little margin.
            cam.data.ortho_scale = max(width, height) * 1.04
            cam.location = (center.x, lo.y - 2.0, center.z)
            cam.rotation_euler = (math.radians(90), 0, 0)
            scene.render.resolution_y = SIZE
            scene.render.resolution_x = max(16, int(SIZE * width / height)) if width < height else SIZE
            if width >= height:
                scene.render.resolution_y = max(16, int(SIZE * height / width))
            name = f'{asset}__{obj.name}'.replace('.', '_')
            scene.render.filepath = os.path.join(out, name + '.png')
            bpy.ops.render.render(write_still=True)
            # The picture covers ortho_scale along its longer side.
            rx, ry = scene.render.resolution_x, scene.render.resolution_y
            k = cam.data.ortho_scale / max(rx, ry)
            cards[name] = dict(width=round(rx * k, 4), height=round(ry * k, 4))
    with open(os.path.join(out, 'cards.json'), 'w') as f:
        json.dump(cards, f, indent=1)
    print(f'baked {len(cards)} cards')


main()
