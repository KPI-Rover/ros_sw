"""
Render the preview pictures of a world for tools such as a GUI: a picture of
the world, a view from the robot camera at each start point, and example
pictures of argument values. What to render is in the 'previews' section of
the world manifest (see world_info.py):

    previews:
      cameras:                    # name: [x, y, z, roll, pitch, yaw, horizontal fov], degrees
        over: [-11, -11, 4, 0, 18, 45, 70]
        close: [-2, -2, 0.25, 0, 25, 30, 60]
      gui_cameras: [over]         # cameras that see what the Gazebo window shows (default: the robot camera)
      world: over                 # camera of the world picture
      start_points: true          # robot camera views from the start points
      map: {center: [0, 0], size: 21}   # where the map made with --sdf is: the square of 'size' m seen
                                        # from right above, or {fit: <model>, margin: 0.3}: around the plane of that model
      presets: true               # a picture of each preset of the world (with the world camera)
      layers:                     # instead of examples of every combination of two arguments:
        camera: over              # the base pictures, with the overlay models hidden, and
        base: {arg: floor, values: [plain, wood], hide: [square_marking]}
        overlay: {arg: marking, values: [white, red], against: [plain, asphalt]}   # the overlay cut out
      args:                       # example pictures of argument values
        grass: {camera: close, values: [0, 1, 3]}
        items: {camera: close, values: [0, 20], with: {objects_seed: 1}}   # 'with': more arguments

The pictures are saved as JPEG files in worlds/<world>/previews/ (world.jpg,
start_<name>.jpg, <arg>_<value>.jpg, preset_<name>.jpg), each
rendered with the other arguments at their defaults. Layers make the base
pictures <base arg>_<value>.jpg without the overlay, and the overlay pictures
<overlay arg>_<value>.png: the overlay alone with a transparent background,
cut out by comparing it with the bare base on contrasting base values; a
tool puts any overlay on any base, so every combination is shown. Needs
Gazebo and xacro (a sourced ROS environment); runs a headless Gazebo server
for each set of world arguments.

    python3 worlds/tools/render_previews.py garden
    python3 worlds/tools/render_previews.py garden --only grass,start_points

With --sdf it makes the map (map.jpg, map.json) of a world file the launch
file generated, e.g. for a GUI to show where the robot can start in the world
just started; --partition keeps its Gazebo apart from a running simulation:
    python3 worlds/tools/render_previews.py garden --sdf /tmp/kpi_rover_world_x.sdf --out /tmp/map --partition map

map.json tells where the map is in the world and where the start points
are: {center: [x, y], size, start_points: {name: [x, y, yaw]}}; the map has
north up and east to the right. It also has the ground heights: 'heights' [[z, ...], ...], rows from
north to south over the map square (0 where the world has no height map),
and the top surface heights 'surface' the same way: the ground with the
objects and structures on it, for checking where the robot can be put.
"""

import argparse
import glob
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lib'))
import world_info  # noqa: E402

WORLDS_DIR = world_info.WORLDS_DIR

# The robot camera relative to a start point: ahead, up (m), pitch and horizontal fov (degrees).
ROBOT_CAMERA = (0.135, 0.31, 3, 70)

# Visibility masks of the preview cameras: like the robot camera (description/camera.xacro), which
# doesn't see helpers shown only in the Gazebo window, or like the Gazebo window itself (cameras
# listed in 'gui_cameras' of the previews).
ROBOT_MASK = 4294967293
GUI_MASK = 0x0FFFFFFF

# Map picture size (px) and the height it is seen from (m): high, so it is nearly orthographic,
# but at most MAP_HEIGHT_RATIO map sizes, so the field of view stays wide enough for Gazebo.
MAP_PIXELS = 900
MAP_HEIGHT = 60
MAP_HEIGHT_RATIO = 5

# Ground height samples along each side of the map.
HEIGHT_SAMPLES = 129
# Surface heights (map.json 'surface'): depth image size (px) and the grid it is reduced to (the highest
# point of each block, so thin walls and stems stay), frames to wait for.
DEPTH_PIXELS = 600
SURFACE_SAMPLES = 200
DEPTH_FRAMES = 4

# Frames to wait for from each camera before taking the last one: big models load for a few seconds.
SETTLE_FRAMES = 10


def ground_heights(sdf, center, size, samples=HEIGHT_SAMPLES):
    """
    Ground heights [[z]] over the map square, rows from north to south, from
    the first height map collision of the world (0 without one).
    """
    c = (np.arange(samples) / (samples - 1) - 0.5) * size
    x, y = np.meshgrid(center[0] + c, center[1] - c)
    z = np.zeros_like(x)
    for collision in ET.fromstring(sdf).iter('collision'):
        heightmap = collision.find('geometry/heightmap')
        if heightmap is None:
            continue
        uri = heightmap.findtext('uri', '').replace('file://', '')
        sx, sy, sz = map(float, heightmap.findtext('size', '1 1 1').split())
        z0 = float(collision.findtext('pose', '0 0 0 0 0 0').split()[2])
        image = np.asarray(Image.open(uri), float)
        image /= 65535 if image.max() > 255 else 255
        rows, cols = image.shape[:2]
        col = np.clip((x / sx + 0.5) * (cols - 1), 0, cols - 1).round().astype(int)
        row = np.clip((0.5 - y / sy) * (rows - 1), 0, rows - 1).round().astype(int)
        inside = (abs(x) <= sx / 2) & (abs(y) <= sy / 2)
        z = np.where(inside, z0 + image[row, col] * sz, z)
        break
    return np.round(z, 3).tolist()


def collision_world(sdf):
    """
    The world with its collisions shown instead of its visuals (except height
    maps, which are only sampled): what a top view of it sees is the surface
    the robot stands on, without leaves, grass and other things it drives
    under or through. Included models (e.g. from Gazebo Fuel) stay as they are.
    """
    root = ET.fromstring(sdf)
    for link in root.iter('link'):
        for visual in link.findall('visual'):
            link.remove(visual)
        for i, collision in enumerate(link.findall('collision')):
            geometry = collision.find('geometry')
            if geometry is None or geometry.find('heightmap') is not None:
                continue
            visual = ET.SubElement(link, 'visual', name=f'surface_{i}')
            if collision.find('pose') is not None:
                visual.append(collision.find('pose'))
            visual.append(geometry)
    return ET.tostring(root, encoding='unicode')


def surface_heights(sdf, center, size, timeout=120):
    """
    Heights [[z]] of the top surface over the map square, rows from north to
    south (SURFACE_SAMPLES each way): the ground, objects, walls and other
    structures the robot would stand on or hit, from a depth camera looking
    down at the collisions; None if it couldn't be rendered.
    """
    from gz.msgs10.image_pb2 import Image as ImageMsg
    from gz.transport13 import Node
    x, y, height, _, _, _, fov = map_camera(center, size)[:7]
    topic = f'/kpi_rover_preview_depth_{os.getpid()}'
    sensor = f'''<model name="preview_depth"><static>true</static><link name="link">
      <sensor name="depth" type="depth_camera"><topic>{topic}</topic>
        <pose>{x} {y} {height} 0 {math.pi / 2} {math.pi / 2}</pose><update_rate>2</update_rate><always_on>1</always_on>
        <camera><horizontal_fov>{math.radians(fov)}</horizontal_fov>
          <image><width>{DEPTH_PIXELS}</width><height>{DEPTH_PIXELS}</height><format>R_FLOAT32</format></image>
          <clip><near>0.1</near><far>{height * 2}</far></clip></camera></sensor></link></model>'''
    world = collision_world(sdf).replace('</world>', sensor + '</world>', 1)
    frames = []
    node = Node()
    node.subscribe(ImageMsg, topic, lambda msg: frames.append(msg))
    with tempfile.NamedTemporaryFile('w', prefix='kpi_rover_surface_', suffix='.sdf', delete=False) as f:
        f.write(world)
    server = subprocess.Popen(['gz', 'sim', '-s', '-r', f.name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                              start_new_session=True)
    start = time.time()
    try:
        while len(frames) < DEPTH_FRAMES and time.time() - start < timeout and server.poll() is None:
            time.sleep(0.5)
    finally:
        stop(server)
        os.unlink(f.name)
    if not frames:
        print('  no depth frames: no surface heights', file=sys.stderr)
        return None
    msg = frames[-1]
    depth = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).astype(float)
    top = np.where(np.isfinite(depth) & (depth < height * 2 - 0.01), height - depth, -np.inf)
    ground = np.array(ground_heights(sdf, center, size, DEPTH_PIXELS))
    top = np.maximum(top, ground)
    k = DEPTH_PIXELS // SURFACE_SAMPLES
    top = top[:k * SURFACE_SAMPLES, :k * SURFACE_SAMPLES].reshape(SURFACE_SAMPLES, k, SURFACE_SAMPLES, k).max(axis=(1, 3))
    print(f'  surface heights in {time.time() - start:.0f} s')
    return np.round(top, 3).tolist()


# A pixel of an overlay render that differs from the bare base by more than this (0-255) is the
# overlay; the alpha ramps up over OVERLAY_SOFT above it, for smooth edges.
OVERLAY_DIFF = 18
OVERLAY_SOFT = 30


def remove_models(sdf, names):
    """The world without the named models."""
    for name in names:
        sdf = re.sub(rf'<model name="{re.escape(name)}">.*?</model>', '', sdf, flags=re.S)
    return sdf


def render_layers(path, info, spec, out, size, quality, only):
    """The base pictures without the overlay, and the overlay pictures cut out of their backgrounds."""
    layers = spec['layers']
    base, overlay = layers['base'], layers['overlay']
    if only and base['arg'] not in only and overlay['arg'] not in only:
        return
    camera = spec['cameras'][layers['camera']]
    bases = list(base['values']) if not only or base['arg'] in only else []
    overlays = list(overlay['values']) if not only or overlay['arg'] in only else []
    bare = {}
    for value in dict.fromkeys(bases + (list(overlay.get('against', [])) if overlays else [])):
        print(f"{info['name']} {base['arg']}:={value} (without {overlay['arg']}): layer")
        sdf = remove_models(world_info.world_sdf(path, {base['arg']: str(value)}), base.get('hide', []))
        with tempfile.TemporaryDirectory(prefix='kpi_rover_layers_') as tmp:
            png = render(sdf, {'layer': camera}, tmp, size).get('layer')
            if png:
                bare[value] = np.asarray(Image.open(png).convert('RGB'), float)
        if value in bases and value in bare:
            Image.fromarray(bare[value].astype(np.uint8)).save(os.path.join(out, f"{base['arg']}_{value}.jpg"),
                                                                 quality=quality, optimize=True)
    for value in overlays:
        alpha, color = None, None
        for back in overlay.get('against', []):
            print(f"{info['name']} {base['arg']}:={back} {overlay['arg']}:={value}: overlay")
            with tempfile.TemporaryDirectory(prefix='kpi_rover_layers_') as tmp:
                png = render(world_info.world_sdf(path, {base['arg']: str(back), overlay['arg']: str(value)}),
                             {'layer': camera}, tmp, size).get('layer')
                picture = np.asarray(Image.open(png).convert('RGB'), float) if png else None
            if picture is None or back not in bare:
                continue
            diff = np.abs(picture - bare[back]).max(axis=2)
            a = np.clip((diff - OVERLAY_DIFF) / OVERLAY_SOFT, 0, 1)
            if alpha is None:
                alpha, color = a, picture
            else:
                # The overlay color from the background it differs from the most.
                color = np.where((a > alpha)[..., None], picture, color)
                alpha = np.maximum(alpha, a)
        if alpha is not None:
            rgba = np.dstack([color, alpha * 255]).astype(np.uint8)
            Image.fromarray(rgba, 'RGBA').save(os.path.join(out, f"{overlay['arg']}_{value}.png"), optimize=True)


def write_map(sdf, out, image, center, size, quality):
    """
    map.jpg from a rendered PNG and map.json: where the map is, the start
    points, the ground heights and the surface heights (see surface_heights).
    """
    Image.open(image).convert('RGB').save(os.path.join(out, 'map.jpg'), quality=quality, optimize=True)
    info = dict(center=list(center), size=size,
                start_points={n: [round(x, 3), round(y, 3), round(yaw, 4)]
                              for n, (x, y, _, yaw) in world_info.start_poses(sdf).items()},
                heights=ground_heights(sdf, center, size))
    surface = surface_heights(sdf, center, size)
    if surface is not None:
        info['surface'] = surface
    with open(os.path.join(out, 'map.json'), 'w') as f:
        json.dump(info, f)


def robot_camera(pose):
    x, y, z, yaw = pose
    ahead, up, pitch, fov = ROBOT_CAMERA
    return [x + ahead * math.cos(yaw), y + ahead * math.sin(yaw), z + up, 0, pitch, math.degrees(yaw), fov]


def stop(server):
    """Stop a Gazebo server started in its own session, with every process it started."""
    for sig in (15, 9):
        try:
            os.killpg(server.pid, sig)
        except ProcessLookupError:
            return
        try:
            server.wait(15)
            return
        except subprocess.TimeoutExpired:
            pass


def map_camera(center, size):
    """A camera right above 'center' seeing a square of 'size' m, north up: [..., width, height]."""
    height = min(MAP_HEIGHT, size * MAP_HEIGHT_RATIO)
    fov = math.degrees(2 * math.atan(size / 2 / height))
    return [center[0], center[1], height, 0, 90, 90, fov, MAP_PIXELS, MAP_PIXELS]


def render(sdf, cameras, out_dir, size, timeout=300, gui_cameras=()):
    """
    Render {name: camera} in an SDF world with a headless Gazebo; return {name: PNG path}.
    A camera is [x, y, z, roll, pitch, yaw, fov], optionally followed by its own width and height.
    Cameras in gui_cameras see what the Gazebo window shows, the others what the robot camera sees.
    """
    frames = os.path.join(out_dir, 'frames')
    sensors = ''
    for name, camera in cameras.items():
        x, y, z, roll, pitch, yaw, fov = camera[:7]
        width, height = camera[7:9] if len(camera) >= 9 else size
        os.makedirs(os.path.join(frames, name), exist_ok=True)
        # Sensor names are prefixed: some, like 'world', are reserved.
        sensors += f'''
      <sensor name="preview_{name}" type="camera">
        <pose>{x} {y} {z} {math.radians(roll)} {math.radians(pitch)} {math.radians(yaw)}</pose>
        <update_rate>2</update_rate><always_on>1</always_on>
        <camera><horizontal_fov>{math.radians(fov)}</horizontal_fov>
          <image><width>{width}</width><height>{height}</height></image>
          <visibility_mask>{GUI_MASK if name in gui_cameras else ROBOT_MASK}</visibility_mask>
          <clip><near>0.05</near><far>{max(300, z * 2)}</far></clip>
          <save enabled="true"><path>{frames}/{name}</path></save>
        </camera>
      </sensor>'''
    model = f'<model name="preview_cameras"><static>true</static><link name="link">{sensors}</link></model>'
    sdf = sdf.replace('</world>', model + '</world>', 1)
    world = re.search(r'<world name="([^"]+)"', sdf).group(1)
    path = os.path.join(out_dir, 'world.sdf')
    with open(path, 'w') as f:
        f.write(sdf)

    def count(name):
        return len(glob.glob(os.path.join(frames, name, '*.png')))

    start = time.time()
    server = subprocess.Popen(['gz', 'sim', '-s', '-r', path], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                              text=True, start_new_session=True)
    try:
        while time.time() - start < timeout:
            time.sleep(1)
            if server.poll() is not None:
                raise RuntimeError(f'Gazebo exited with {server.returncode}: {server.stderr.read()[-2000:]}')
            # Several frames each, so all models have loaded; cameras render only while their images are subscribed.
            waiting = [name for name in cameras if count(name) < SETTLE_FRAMES]
            if not waiting:
                break
            for name in waiting:
                subprocess.run(['timeout', '3', 'gz', 'topic', '-e', '-n', '1', '-t',
                                f'/world/{world}/model/preview_cameras/link/link/sensor/preview_{name}/image'],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    finally:
        stop(server)
    result = {}
    for name in cameras:
        files = sorted(glob.glob(os.path.join(frames, name, '*.png')),
                       key=lambda f: int(re.findall(r'\d+', os.path.basename(f))[-1]))
        if files:
            result[name] = files[-1]
        else:
            print(f'  no frames from camera {name}', file=sys.stderr)
    print(f'  rendered {len(result)} pictures in {time.time() - start:.0f} s')
    return result


# The map of a world without a 'map' in its manifest previews: this square around the origin.
DEFAULT_MAP = {'center': [0, 0], 'size': 20}


def map_place(spec, sdf=None):
    """
    (center, size) of the map of a world, from its manifest previews: a fixed
    square, or with 'fit: <model>' the square around the plane collision of
    that model in the world (e.g. a floor whose size depends on the world
    arguments), plus 'margin' m around it.
    """
    m = (spec or {}).get('map') or DEFAULT_MAP
    if m.get('fit') and sdf:
        for model in ET.fromstring(sdf).iter('model'):
            plane = model.find('link/collision/geometry/plane/size')
            if model.get('name') == m['fit'] and plane is not None:
                x, y = map(float, model.findtext('pose', '0 0 0 0 0 0').split()[:2])
                sx, sy = map(float, plane.text.split())
                return [x, y], round(max(sx, sy) + 2 * m.get('margin', 0.3), 2)
    return m.get('center', [0, 0]), m['size']


def jobs(info, spec, only):
    """
    {args tuple: {picture file name: camera}} of what to render, grouped by
    the world arguments; a camera of 'gui_cameras' is marked by a 'gui' key.
    """
    cameras = spec.get('cameras', {})
    groups = {}
    gui = set()
    if spec.get('world') in (spec.get('gui_cameras') or []):
        gui.add('world')

    def add(args, filename, camera):
        groups.setdefault(tuple(sorted(args.items())), {})[filename] = camera

    def wanted(key):
        return not only or key in only

    if spec.get('world') and wanted('world'):
        add({}, 'world', cameras[spec['world']])
    if spec.get('start_points') and wanted('start_points'):
        add({}, 'start_points', None)
    if spec.get('presets') and spec.get('world') and wanted('presets'):
        for preset in info.get('presets', []):
            add(dict(preset['args']), f"preset_{preset['name']}", cameras[spec['world']])
            if spec['world'] in (spec.get('gui_cameras') or []):
                gui.add(f"preset_{preset['name']}")
    for arg, example in (spec.get('args') or {}).items():
        if not wanted(arg):
            continue
        arg_spec = info['args'][arg]
        extra = {k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in (example.get('with') or {}).items()}
        for value in example['values']:
            text = world_info.format_value(arg_spec, world_info.parse(arg_spec, str(value)))
            default = world_info.format_value(arg_spec, arg_spec['default'])
            add(dict(extra, **({} if text == default else {arg: text})), f'{arg}_{text}', cameras[example['camera']])
            if example['camera'] in (spec.get('gui_cameras') or []):
                gui.add(f'{arg}_{text}')
    return groups, gui


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('world')
    parser.add_argument('--only', default='', help="comma separated: 'world', 'start_points', 'presets' and argument names")
    parser.add_argument('--size', default='960x600')
    parser.add_argument('--quality', type=int, default=85)
    parser.add_argument('--sdf', help='make only the map of this world file (one the launch file generated)')
    parser.add_argument('--out', help='folder for the map made with --sdf')
    parser.add_argument('--partition', help='Gazebo transport partition, to keep apart from a running simulation')
    options = parser.parse_args()
    if options.partition:
        os.environ['GZ_PARTITION'] = options.partition

    path = world_info.world_files().get(options.world)
    if options.sdf:
        spec = world_info.manifest(path).get('previews') if path else None
        with open(options.sdf) as f:
            sdf = f.read()
        os.makedirs(options.out, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='kpi_rover_map_') as tmp:
            png = render(sdf, {'map': map_camera(*map_place(spec, sdf))}, tmp, None).get('map')
            if not png:
                sys.exit('The map was not rendered')
            write_map(sdf, options.out, png, *map_place(spec, sdf), options.quality)
        print(f'Map in {options.out}')
        return
    if not path:
        sys.exit(f'{options.world}: no such world')
    info = world_info.describe(path)
    spec = world_info.manifest(path).get('previews')
    if not spec:
        sys.exit(f'{options.world}: no previews section in the manifest')
    out = os.path.join(WORLDS_DIR, options.world, 'previews')
    os.makedirs(out, exist_ok=True)
    size = tuple(map(int, options.size.split('x')))
    only = set(filter(None, options.only.split(',')))

    groups, gui = jobs(info, spec, only)
    for args, pictures in groups.items():
        args = dict(args)
        print(f"{options.world} {' '.join(f'{k}:={v}' for k, v in args.items()) or '(defaults)'}: "
              f"{', '.join(pictures)}")
        sdf = world_info.world_sdf(path, args)
        cameras = {name: camera for name, camera in pictures.items() if camera}
        if 'start_points' in pictures:
            for name, pose in world_info.start_poses(sdf).items():
                cameras[f'start_{name}'] = robot_camera(pose)
        with tempfile.TemporaryDirectory(prefix='kpi_rover_previews_') as tmp:
            for name, png in render(sdf, cameras, tmp, size, gui_cameras=gui).items():
                Image.open(png).convert('RGB').save(os.path.join(out, name + '.jpg'), quality=options.quality,
                                                    optimize=True)
    if spec.get('layers'):
        render_layers(path, info, spec, out, size, options.quality, only)
    print(f'Pictures in {out}')


if __name__ == '__main__':
    main()
