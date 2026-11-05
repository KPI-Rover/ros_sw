"""SDF building blocks for the world generators."""

import math
import os
import xml.etree.ElementTree as ET

import yaml

from . import textured_box
from .world_info import CONFIG_DIR, SHARE_DIR

ASSETS_DIR = os.path.join(SHARE_DIR, 'assets')
TEXTURES_DIR = os.path.join(ASSETS_DIR, 'textures')
MESHES_DIR = os.path.join(ASSETS_DIR, 'meshes')


def _load(name):
    with open(os.path.join(CONFIG_DIR, name + '.yaml')) as f:
        return yaml.safe_load(f)


FLOORS = _load('floors')
MARKINGS = _load('markings')
WALLS = _load('walls')
OBJECTS = _load('objects')

# Visibility flags; the robot sensor masks are in kpi_rover_description (camera.xacro, lidar.xacro).
VISIBLE_ALL = 4294967295
VISIBLE_NO_LIDAR = 4            # grass the robot drives through
VISIBLE_GUI_ONLY = 2
VISIBLE_ROBOT_ONLY = 536870912  # bit 29, outside the GUI mask 0x0FFFFFFF

MARKING_THICKNESS = 0.001


def num(v):
    if isinstance(v, bool):
        return str(v).lower()
    if isinstance(v, (int, str)):
        return str(v)
    text = f'{v:.10g}'
    return '0' if text == '-0' else text


def el(parent, tag, *values, **attrib):
    e = ET.SubElement(parent, tag, attrib)
    if values:
        e.text = ' '.join(map(num, values))
    return e


def to_string(root):
    ET.indent(root, '  ')
    return ET.tostring(root, encoding='unicode', xml_declaration=True) + '\n'


def material(parent, color, texture=''):
    """'texture' is an image in assets/textures, or a photo texture directory (name or absolute path)."""
    m = el(parent, 'material')
    el(m, 'ambient', *color, 1)
    el(m, 'diffuse', *color, 1)
    if texture:
        metal = el(el(m, 'pbr'), 'metal')
        name = os.path.basename(texture)
        if '.' in name:
            el(metal, 'albedo_map', os.path.join(TEXTURES_DIR, texture))
            el(metal, 'roughness', '0.9')
        else:
            directory = os.path.join(TEXTURES_DIR, texture)
            for kind in ('albedo', 'normal', 'roughness'):
                el(metal, f'{kind}_map', f'{directory}/{name}_{kind}.jpg')
            el(metal, 'roughness', '1.0')
        el(metal, 'metalness', '0.0')


def world(name, shadows=False, sky=(0.7, 0.7, 0.7), skybox=False, ambient=0.6, sun=(-0.3, 0.2, -0.9)):
    root = ET.Element('sdf', version='1.9')
    w = el(root, 'world', name=name)
    for plugin, system in (('physics', 'Physics'), ('user-commands', 'UserCommands'),
                           ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors'), ('imu', 'Imu')):
        p = el(w, 'plugin', filename=f'gz-sim-{plugin}-system', name=f'gz::sim::systems::{system}')
        if system == 'Sensors':
            el(p, 'render_engine', 'ogre2')
    el(w, 'gravity', '0 0 -9.8')
    physics = el(w, 'physics', default='0', name='default_physics', type='ode')
    el(physics, 'max_step_size', '0.01')
    el(physics, 'real_time_factor', '1')
    el(physics, 'real_time_update_rate', '100')
    scene = el(w, 'scene')
    el(scene, 'ambient', ambient, ambient, ambient, 1)
    el(scene, 'background', *sky, 1)
    el(scene, 'shadows', shadows)
    if skybox:
        el(scene, 'sky')
    light = el(w, 'light', name='sun', type='directional')
    el(light, 'pose', '0 0 10 0 0 0')
    el(light, 'diffuse', '0.8 0.8 0.8 1')
    el(light, 'specular', '0.2 0.2 0.2 1')
    el(light, 'direction', *sun)
    el(light, 'cast_shadows', shadows)
    return root, w


def static_model(parent, name, pose=None):
    """Returns the link of a new static model."""
    model = el(parent, 'model', name=name)
    el(model, 'static', 'true')
    if pose:
        el(model, 'pose', *pose)
    return el(model, 'link', name='link')


def plane(parent, size_x, size_y):
    p = el(el(parent, 'geometry'), 'plane')
    el(p, 'normal', '0 0 1')
    el(p, 'size', size_x, size_y)


def friction(collision, mu):
    ode = el(el(el(collision, 'surface'), 'friction'), 'ode')
    el(ode, 'mu', mu)
    el(ode, 'mu2', mu)


def floor(parent, kind, size_x, size_y):
    """A textured floor is made of tiles, one texture repeat each."""
    cfg = FLOORS[kind]
    link = static_model(parent, 'ground_plane')
    collision = el(link, 'collision', name='collision')
    plane(collision, size_x, size_y)
    friction(collision, cfg['mu'])
    if not cfg['texture']:
        visual = el(link, 'visual', name='visual')
        plane(visual, size_x, size_y)
        material(visual, cfg['color'])
        return
    nx, ny = max(1, round(size_x / cfg['tile'])), max(1, round(size_y / cfg['tile']))
    tx, ty = size_x / nx, size_y / ny
    for k in range(nx * ny):
        visual = el(link, 'visual', name=f'tile_{k}')
        el(visual, 'pose', tx * (k % nx + 0.5 - nx / 2), ty * (k // nx + 0.5 - ny / 2), 0, 0, 0, 0)
        plane(visual, tx, ty)
        material(visual, cfg['color'], cfg['texture'])


def marking_line(link, name, x1, y1, x2, y2, style, z1=0.0, z2=0.0, width=0.05, start='flat', end='flat', extend=0.0,
                 fine=False, visibility=VISIBLE_ALL):
    """
    start, end: 'flat' (lengthened by 'extend') or 'left'/'right', a 45 degree cut for a corner turning that way.
    Striped lines are split into segments 4 widths long, 1 width if 'fine' (to follow a curved surface).
    """
    cfg = MARKINGS[style]
    dist = math.dist((x1, y1, z1), (x2, y2, z2))
    dx, dy, dz = (x2 - x1) / dist, (y2 - y1) / dist, (z2 - z1) / dist
    yaw, pitch = math.atan2(dy, dx), -math.asin(dz)
    start_offset = extend if start == 'flat' else -width / 2
    length = dist + start_offset + (extend if end == 'flat' else -width / 2)
    bx, by, bz = x1 - dx * start_offset, y1 - dy * start_offset, z1 - dz * start_offset
    texture = cfg['texture'].replace('.png', '_tile.png') if fine else cfg['texture']
    n = max(1, round(length / (width if fine else 4 * width))) if fine or texture else 1
    seg = length / n
    for k in range(n):
        s = seg * (k + 0.5)
        visual = el(link, 'visual', name=f'{name}_{k}')
        el(visual, 'pose', bx + dx * s, by + dy * s, bz + dz * s + MARKING_THICKNESS / 2, 0, pitch, yaw)
        el(el(el(visual, 'geometry'), 'box'), 'size', seg, width, MARKING_THICKNESS)
        material(visual, cfg['color'], texture)
        el(visual, 'visibility_flags', visibility)
    for side, turn, x, y in (('start', start, bx, by), ('end', end, bx + dx * length, by + dy * length)):
        if turn != 'flat':
            visual = el(link, 'visual', name=f'{name}_{side}_miter')
            el(visual, 'pose', x, y, 0, 0, 0, yaw)
            mesh = el(el(visual, 'geometry'), 'mesh')
            el(mesh, 'uri', f'file://{MESHES_DIR}/miter_{side}_{turn}.obj')
            el(mesh, 'scale', width, width, MARKING_THICKNESS)
            material(visual, cfg['color'], cfg['texture'])
            el(visual, 'visibility_flags', visibility)


def marking_arc(link, name, cx, cy, r, a1, a2, style, width=0.05, visibility=VISIBLE_ALL):
    """Counterclockwise from a1 to a2, of straight segments 4 widths long; alternate ones are raised against z-fighting."""
    cfg = MARKINGS[style]
    n = max(1, round((a2 - a1) * r / (4 * width)))
    step = (a2 - a1) / n
    for k in range(n):
        a = a1 + step * (k + 0.5)
        visual = el(link, 'visual', name=f'{name}_{k}')
        el(visual, 'pose', cx + r * math.cos(step / 2) * math.cos(a), cy + r * math.cos(step / 2) * math.sin(a),
           MARKING_THICKNESS / 2 + (k % 2) * 0.0001, 0, 0, a + math.pi / 2)
        el(el(el(visual, 'geometry'), 'box'), 'size', 2 * r * math.sin(step / 2) + width * math.tan(step / 2), width,
           MARKING_THICKNESS)
        material(visual, cfg['color'], cfg['texture'])
        el(visual, 'visibility_flags', visibility)


def mesh_visual(link, name, mesh):
    visual = el(link, 'visual', name=name)
    el(el(el(visual, 'geometry'), 'mesh'), 'uri', f'file://{mesh}')
    return visual


def start_point(parent, name, x, y, yaw, z=0.0):
    """Robot footprint and heading arrow, GUI only; the launch file finds it by the model name."""
    link = static_model(parent, f'start_point_{name}', (x, y, z + 0.001, 0, 0, yaw))
    corners = [(0.15, -0.15), (0.15, 0.15), (-0.15, 0.15), (-0.15, -0.15)]
    for i, side in enumerate(('front', 'left', 'back', 'right')):
        marking_line(link, f'outline_{side}', *corners[i], *corners[(i + 1) % 4], 'green', width=0.01,
                     start='left', end='left', visibility=VISIBLE_GUI_ONLY)
    marking_line(link, 'arrow_shaft', -0.15, -0.25, 0.15, -0.25, 'green', width=0.02, visibility=VISIBLE_GUI_ONLY)
    visual = el(link, 'visual', name='arrow_head')
    polyline = el(el(visual, 'geometry'), 'polyline')
    for point in ('0.15 -0.31', '0.27 -0.25', '0.15 -0.19'):
        el(polyline, 'point', point)
    el(polyline, 'height', '0.001')
    material(visual, MARKINGS['green']['color'])
    el(visual, 'visibility_flags', VISIBLE_GUI_ONLY)


def box_geometry(parent, sx, sy, sz, mesh=''):
    geometry = el(parent, 'geometry')
    if mesh:
        el(el(geometry, 'mesh'), 'uri', f'file://{mesh}')
    else:
        el(el(geometry, 'box'), 'size', sx, sy, sz)


def solid_box(link, name, x, y, z, sx, sy, sz, color=None, wall_material=None, pitch=0, yaw=0, view='solid',
              transparency=0.7, footprint_scale=1, footprint_color=None):
    """
    Box with a collision; a wall_material of walls.yaml gives its color and texture at the real size.
    view (GUI only): solid, transparent, hidden, or marking - its footprint, footprint_scale as wide.
    """
    texture = mesh = ''
    if wall_material:
        cfg = WALLS[wall_material]
        color, texture = cfg['color'], cfg['texture']
        if texture:
            mesh = textured_box.box_mesh(sx, sy, sz, cfg['segment'], textured_box.WALL_TEXTURE_HEIGHT, z - sz / 2)
    pose = (x, y, z, 0, pitch, yaw)
    collision = el(link, 'collision', name=f'{name}_collision')
    el(collision, 'pose', *pose)
    box_geometry(collision, sx, sy, sz)
    visual = el(link, 'visual', name=f'{name}_visual')
    el(visual, 'pose', *pose)
    box_geometry(visual, sx, sy, sz, mesh)
    material(visual, color, texture)
    el(visual, 'visibility_flags', VISIBLE_ALL if view == 'solid' else VISIBLE_ROBOT_ONLY)
    if view == 'transparent':
        visual = el(link, 'visual', name=f'{name}_gui_visual')
        el(visual, 'pose', *pose)
        box_geometry(visual, sx, sy, sz, mesh)
        material(visual, color, texture)
        el(visual, 'transparency', transparency)
        el(visual, 'visibility_flags', VISIBLE_GUI_ONLY)
    if view == 'marking':
        visual = el(link, 'visual', name=f'{name}_footprint')
        el(visual, 'pose', x, y, 0.0015, 0, 0, yaw)
        box_geometry(visual, sx, sy * footprint_scale, 0.001)
        material(visual, footprint_color or color)
        el(visual, 'visibility_flags', VISIBLE_GUI_ONLY)


def wall(link, name, x, y, yaw, length, height=0.5, thickness=0.05, material='plain', view='solid'):
    solid_box(link, name, x, y, height / 2, length, thickness, height, wall_material=material, yaw=yaw, view=view,
              transparency=0.35, footprint_color=WALLS['plain']['color'])


def perimeter_walls(parent, size_x, size_y, material, height=0.5, thickness=0.1):
    link = static_model(parent, 'perimeter_walls')
    for name, x, y, yaw, length in (('north', 0, (size_y + thickness) / 2, 0, size_x + 2 * thickness),
                                    ('south', 0, -(size_y + thickness) / 2, math.pi, size_x + 2 * thickness),
                                    ('east', (size_x + thickness) / 2, 0, math.pi / 2, size_y),
                                    ('west', -(size_x + thickness) / 2, 0, -math.pi / 2, size_y)):
        wall(link, name, x, y, yaw, length, height, thickness, material)


def scatter_object(parent, name, kind, x, y, yaw, z=0.0, scale=1.0):
    """An object of objects.yaml, its origin on the ground under its center; mass scales with the volume."""
    cfg = OBJECTS[kind]
    k = float(scale)
    sx, sy, sz = (v * k for v in cfg['size'])
    mass = cfg['mass'] * k ** 3
    model = el(parent, 'model', name=name)
    el(model, 'pose', x, y, z, 0, 0, yaw)
    link = el(model, 'link', name='link')
    inertial = el(link, 'inertial')
    el(inertial, 'pose', 0, 0, sz / 2, 0, 0, 0)
    el(inertial, 'mass', mass)
    inertia = el(inertial, 'inertia')
    el(inertia, 'ixx', mass * (sy ** 2 + sz ** 2) / 12)
    el(inertia, 'iyy', mass * (sx ** 2 + sz ** 2) / 12)
    el(inertia, 'izz', mass * (sx ** 2 + sy ** 2) / 12)
    for axes in ('ixy', 'ixz', 'iyz'):
        el(inertia, axes, 0)
    if cfg['shape'] == 'box':
        _object_collision(link, 'collision', sz / 2, 'box', size=(sx, sy, sz))
    elif cfg['shape'] == 'cylinder':
        _object_collision(link, 'collision', sz / 2, 'cylinder', radius=max(sx, sy) / 2, length=sz)
    else:
        base = 0.03 * k     # cone base plate
        _object_collision(link, 'base_collision', base / 2, 'box', size=(sx, sy, base))
        _object_collision(link, 'body_collision', (sz + base) / 2, 'cylinder', radius=cfg['radius'] * k,
                          length=sz - base)
    visual = el(link, 'visual', name='visual')
    cx, cy, cz = (v * k for v in cfg['center'])
    el(visual, 'pose', -cx, -cy, sz / 2 - cz, cfg['roll'], 0, 0)
    mesh = el(el(visual, 'geometry'), 'mesh')
    el(mesh, 'uri', f"file://{ASSETS_DIR}/{cfg['mesh']}")
    el(mesh, 'scale', k, k, k)
    if 'texture' in cfg:
        material(visual, (1, 1, 1), cfg['texture'])


def _object_collision(link, name, z, shape, **dims):
    collision = el(link, 'collision', name=name)
    el(collision, 'pose', 0, 0, z, 0, 0, 0)
    geometry = el(el(collision, 'geometry'), shape)
    for dim, value in dims.items():
        el(geometry, dim, *(value if isinstance(value, tuple) else (value,)))
    friction(collision, 0.6)
