"""
Descriptions of the worlds and their arguments, and building their SDF, for
the launch file and for tools such as a GUI.

Every world has a directory of its own, worlds/<name>/, with the world file
<name>.world (SDF) or <name>.py, a configurable world whose build(args)
returns the SDF root (see sdf.py) for {arg: parsed value}, and a manifest
<name>.yaml describing the world and each of its arguments:

    title: Garden
    description: Apple orchard in the steppe with a field road around it.
    start_points:                 # named start points (see the 'start' launch argument)
      road_a: South side of the road
      garden_a: West part of the orchard
    groups:                       # optional: order of argument groups in a UI
      terrain: Terrain
    args:
      relief:
        type: choice              # bool, int, float, string, choice
        choices: [steppe, hills]  # for 'choice'; or choices_from: floors
                                  # (a common/config/<name>.yaml file)
        label: Relief
        description: ...
        group: terrain
      terrain_seed: {type: int, min: 0, default: 11, label: Terrain seed}
      friction: {type: float, label: Friction, advanced: true}   # shown under 'more options' in a UI
      seed: {type: int, show_if: {maze: random}}   # shown only while other arguments have these values
    presets:                      # optional: ready sets of arguments
      light:
        label: Light
        description: No grass and shadows, the fastest simulation.
        args: {grass: 0, shadows: false}
    previews: ...                 # optional: pictures, see render_previews.py
    objects_scale: 0.345          # optional: size of library objects in a scaled-down world

Pictures rendered by render_previews.py are in worlds/<name>/previews/:
describe() gives their paths as 'image' of the world, of each start point
and of each preset, 'layers' {base, overlay} names the two arguments whose
pictures are put one over the other (the overlay ones are transparent PNG),
and as 'images' [{value, image}] of the arguments.

Command line, for tools:
    python3 -m kpi_rover_worlds.world_info --json    # all worlds, their arguments and defaults
    python3 -m kpi_rover_worlds.world_info --check   # manifests match the worlds
    python3 -m kpi_rover_worlds.world_info --build garden grass:=0.5 mud:=0.25   # prints the world file
"""

import hashlib
import json
import os
import sys
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory

SHARE_DIR = get_package_share_directory('kpi_rover_worlds')
WORLDS_DIR = os.path.join(SHARE_DIR, 'worlds')
CONFIG_DIR = os.path.join(SHARE_DIR, 'common', 'config')
EXTENSIONS = ('.py', '.world')
# World files built by build(): the CACHE_KEEP most recently used are kept.
CACHE_DIR = os.path.join(os.path.expanduser('~'), '.cache', 'kpi_rover', 'worlds')
CACHE_KEEP = 20
TYPES = ('bool', 'int', 'float', 'string', 'choice')


def world_name(filename):
    """World name for a world file name, or None if it is not a world file."""
    for ext in EXTENSIONS:
        if filename.endswith(ext):
            return filename[:-len(ext)]
    return None


def world_files(worlds_dir=WORLDS_DIR):
    """{name: path} of the worlds: <name>/<name>.py or <name>/<name>.world in the worlds folder."""
    files = {}
    for name in sorted(os.listdir(worlds_dir)):
        for ext in EXTENSIONS:
            path = os.path.join(worlds_dir, name, name + ext)
            if os.path.isfile(path):
                files[name] = path
                break
    return files


def declared_args(path):
    """{name: default as text} of the arguments of a configurable world, in order; empty for an SDF world."""
    return {arg: as_text(spec['default']) for arg, spec in (manifest(path).get('args') or {}).items()
            if 'default' in (spec or {})}


def as_text(value):
    """A manifest value (YAML: bool, number or string) as given on the command line."""
    return str(value).lower() if isinstance(value, bool) else str(value)


def resolve(world, worlds_dir=WORLDS_DIR):
    """Path of a world given by name or by path; raise ValueError if there is none."""
    if os.path.isfile(world):
        return os.path.abspath(world)
    files = world_files(worlds_dir)
    if world in files:
        return files[world]
    raise ValueError(f"World '{world}' not found. Available worlds: {', '.join(files)} "
                     f"(or pass a path to a world file).")


_generators = {}


def generator(path):
    """The module of a <name>.py world, loaded once."""
    if path not in _generators:
        import importlib.util
        world_dir = os.path.dirname(path)
        spec = importlib.util.spec_from_file_location(f'kpi_rover_world_{os.path.basename(world_dir)}', path)
        module = importlib.util.module_from_spec(spec)
        # Its own modules (e.g. garden/terrain.py) are found next to it while it is imported.
        sys.path.insert(0, world_dir)
        try:
            spec.loader.exec_module(module)
        finally:
            sys.path.remove(world_dir)
        _generators[path] = module
    return _generators[path]


def world_sdf(path, args, worlds_dir=WORLDS_DIR):
    """SDF of a world file: an SDF file as it is, or a <name>.py world built for {arg: string value}."""
    if not path.endswith('.py'):
        with open(path) as f:
            return f.read()
    validate(path, args, worlds_dir)
    from .sdf import to_string
    specs = describe(path, worlds_dir)['args']
    values = {arg: spec['default'] for arg, spec in specs.items()}
    values.update({arg: parse(specs[arg], value) for arg, value in args.items()})
    return to_string(generator(path).build(values))


def parse_args(text):
    """{name: value} of world arguments given as space separated 'name:=value' pairs."""
    args = {}
    for pair in text.split():
        name, sep, value = pair.partition(':=')
        if not sep:
            raise ValueError(f"Invalid world argument '{pair}', expected 'name:=value'.")
        args[name] = value
    return args


def build(world, args, worlds_dir=WORLDS_DIR):
    """
    Path of an SDF file for Gazebo of a world given by name or path: an SDF file as it is, or a configurable
    world built with {arg: string value}, checked against its manifest, into CACHE_DIR/<name>_<hash>.world.
    """
    path = resolve(world, worlds_dir)
    if not path.endswith('.py'):
        if args:
            raise ValueError(f"{os.path.basename(path)} is not a configurable world, it takes no arguments.")
        return path
    sdf = world_sdf(path, args, worlds_dir)
    os.makedirs(CACHE_DIR, exist_ok=True)
    digest = hashlib.sha1(sdf.encode()).hexdigest()[:12]
    file = os.path.join(CACHE_DIR, f'{world_name(os.path.basename(path))}_{digest}.world')
    if not os.path.isfile(file):
        tmp = f'{file}.{os.getpid()}.tmp'   # several launches may build the same world at once
        with open(tmp, 'w') as f:
            f.write(sdf)
        os.replace(tmp, file)
    os.utime(file)      # marks it as recently used
    clean_cache()
    return file


def clean_cache(keep=CACHE_KEEP):
    """Remove the least recently used world files from CACHE_DIR, keeping 'keep' of them."""
    files = [os.path.join(CACHE_DIR, f) for f in os.listdir(CACHE_DIR) if f.endswith('.world')]
    files.sort(key=os.path.getmtime)
    for file in files[:-keep]:
        try:
            os.remove(file)
        except FileNotFoundError:
            pass        # removed by another launch


def start_poses(sdf):
    """{name: (x, y, z, yaw)} of the start points of an SDF world: models named start_point_<name>."""
    poses = {}
    for model in ET.fromstring(sdf).iter('model'):
        name = model.get('name', '')
        if name.startswith('start_point_'):
            x, y, z, _, _, yaw = map(float, model.findtext('pose', '0 0 0 0 0 0').split())
            poses[name[len('start_point_'):]] = (x, y, z, yaw)
    return poses


def start_pose(path, name):
    """(x, y, z, yaw) of a start point of an SDF world file; raise ValueError if there is none."""
    with open(path) as f:
        poses = start_poses(f.read())
    if name not in poses:
        raise ValueError(f"Start point '{name}' not found. Start points of the world: {', '.join(sorted(poses)) or 'none'}.")
    return poses[name]


def options(name, worlds_dir=WORLDS_DIR):
    """Option names from a common/config YAML file (floors, markings, walls)."""
    with open(os.path.join(os.path.dirname(worlds_dir), 'common', 'config', name + '.yaml')) as f:
        return list(yaml.safe_load(f))


def manifest(path):
    """The manifest of a world file (<name>.yaml next to it), or an empty one."""
    name = world_name(os.path.basename(path))
    m = os.path.join(os.path.dirname(path), f'{name}.yaml') if name else None
    if m and os.path.isfile(m):
        with open(m) as f:
            return yaml.safe_load(f) or {}
    return {}


def describe(path, worlds_dir=WORLDS_DIR):
    """
    Description of a world file: {name, file, title, description, image, start_points: [{name, label, image}],
    groups, presets: [{name, label, description, args}],
    args: {name: {type, default, label, description, group, advanced, choices, min, max, images, ...}}}.
    Images are paths of existing pictures or absent.
    """
    m = manifest(path)
    name = world_name(os.path.basename(path))
    pictures = os.path.join(os.path.dirname(path), 'previews')

    layers = (m.get('previews') or {}).get('layers')
    overlay = layers['overlay']['arg'] if layers else None

    def picture(filename):
        # Overlay pictures are transparent PNG only (a JPEG of the same name would be a whole old frame).
        for ext in (('.png',) if overlay and filename.startswith(overlay + '_') else ('.jpg', '.png')):
            file = os.path.join(pictures, filename + ext)
            if os.path.isfile(file):
                return file
        return None

    info = dict(name=name, file=path, title=m.get('title', name), description=m.get('description', ''),
                image=picture('world'), objects_scale=m.get('objects_scale', 1.0),
                start_points=start_point_list(m), groups=m.get('groups', {}),
                presets=preset_list(m), args={})
    for point in info['start_points']:
        point['image'] = picture('start_' + point['name'])
    for preset in info['presets']:
        preset['image'] = picture('preset_' + preset['name'])
    if layers:
        info['layers'] = dict(base=layers['base']['arg'], overlay=layers['overlay']['arg'])
    if not path.endswith('.py'):
        return info
    specs = m.get('args', {})
    for arg, default in declared_args(path).items():
        spec = dict(specs.get(arg) or {})
        if 'choices_from' in spec:
            spec['choices'] = options(spec.pop('choices_from'), worlds_dir)
        spec.setdefault('type', 'choice' if 'choices' in spec else 'string')
        spec.setdefault('label', arg)
        spec['default'] = parse(spec, default) if default != '' else default
        previews = m.get('previews') or {}
        examples = (previews.get('args') or {}).get(arg, {}).get('values', [])
        for layer in ((previews.get('layers') or {}).get(k) for k in ('base', 'overlay')):
            if layer and layer['arg'] == arg:
                examples = layer['values']
        images = [dict(value=parse(spec, str(v)), image=picture(f'{arg}_{format_value(spec, parse(spec, str(v)))}'))
                  for v in examples]
        spec['images'] = [i for i in images if i['image']]
        info['args'][arg] = spec
    return info


def format_value(spec, value):
    """The string form of a parsed argument value, as given on the command line."""
    if spec.get('type') == 'bool':
        return 'true' if value else 'false'
    if spec.get('type') == 'float':
        return f'{value:g}'
    return str(value)


def preset_list(m):
    """[{name, label, description, args: {name: string value}}] of the presets of a manifest."""
    return [dict(name=str(n), label=p.get('label', str(n)), description=p.get('description', ''),
                 args={k: as_text(v) for k, v in (p.get('args') or {}).items()})
            for n, p in (m.get('presets') or {}).items()]


def start_point_list(m):
    """[{name, label}] of the start points of a manifest; they can be listed as names or as {name: label}."""
    points = m.get('start_points') or []
    if isinstance(points, dict):
        return [dict(name=str(n), label=label or str(n)) for n, label in points.items()]
    return [dict(name=str(n), label=str(n)) for n in points]


def describe_all(worlds_dir=WORLDS_DIR):
    return [describe(path, worlds_dir) for path in world_files(worlds_dir).values()]


def parse(spec, value):
    """Parse a string world argument value by its spec; raise ValueError if it is not valid."""
    kind = spec.get('type', 'string')
    if kind == 'bool':
        if str(value).lower() in ('true', '1'):
            return True
        if str(value).lower() in ('false', '0'):
            return False
        raise ValueError(f"expected true or false, got '{value}'")
    if kind in ('int', 'float'):
        try:
            v = int(value) if kind == 'int' else float(value)
        except ValueError:
            raise ValueError(f"expected {'an integer' if kind == 'int' else 'a number'}, got '{value}'")
        if 'min' in spec and v < spec['min'] or 'max' in spec and v > spec['max']:
            raise ValueError(f"expected a value from {spec.get('min', '-inf')} to {spec.get('max', 'inf')}, got {v}")
        return v
    if kind == 'choice' and value not in spec.get('choices', []):
        raise ValueError(f"expected one of {', '.join(map(str, spec.get('choices', [])))}, got '{value}'")
    return value


def validate(path, values, worlds_dir=WORLDS_DIR):
    """Check {arg: string value} for a world; raise ValueError naming the argument at the first problem."""
    info = describe(path, worlds_dir)
    declared = declared_args(path)
    for arg, value in values.items():
        if arg not in declared:
            raise ValueError(f"Unknown world argument '{arg}'. Arguments of {os.path.basename(path)}: "
                             f"{', '.join(info['args'])}.")
        try:
            parse(info['args'][arg], value)
        except ValueError as e:
            raise ValueError(f"World argument '{arg}': {e}.") from e


def promised_previews(m, info):
    """Names (without extension) of the pictures in previews/ that the manifest asks render_previews.py for."""
    previews = m.get('previews') or {}
    names = ['world'] if previews.get('world') else []
    if previews.get('start_points'):
        names += ['start_' + p['name'] for p in info['start_points']]
    if previews.get('presets'):
        names += ['preset_' + p['name'] for p in info['presets']]
    examples = {arg: e.get('values', []) for arg, e in (previews.get('args') or {}).items()}
    for layer in ((previews.get('layers') or {}).get(k) for k in ('base', 'overlay')):
        if layer:
            examples[layer['arg']] = layer['values']
    for arg, values in examples.items():
        spec = info['args'].get(arg)
        if spec:  # an unknown argument is reported by check()
            names += [f'{arg}_{format_value(spec, parse(spec, str(v)))}' for v in values]
    return names


def check(worlds_dir=WORLDS_DIR):
    """
    Problems of the worlds: pictures the manifest asks for are missing; arguments without a default, bad specs.
    """
    problems = []
    for name, path in world_files(worlds_dir).items():
        m = manifest(path)
        pictures = os.path.join(os.path.dirname(path), 'previews')
        try:
            for picture in promised_previews(m, describe(path, worlds_dir)):
                if not any(os.path.isfile(os.path.join(pictures, picture + ext)) for ext in ('.jpg', '.png')):
                    problems.append(f'{name}: no picture previews/{picture}.jpg (render_previews.py {name})')
        except ValueError:
            pass  # bad example values are reported below
        if not path.endswith('.py'):
            continue
        if not m:
            problems.append(f'{name}: no manifest {name}.yaml')
            continue
        declared = declared_args(path)
        for arg, spec in m.get('args', {}).items():
            if arg not in declared:
                problems.append(f'{name}: argument {arg} has no default')
            else:
                kind = spec.get('type', 'choice' if 'choices' in spec or 'choices_from' in spec else 'string')
                if kind not in TYPES:
                    problems.append(f'{name}: argument {arg} has an unknown type {kind}')
                try:
                    if declared[arg] != '':
                        parse(dict(spec, choices=options(spec['choices_from'], worlds_dir)) if 'choices_from' in spec
                              else spec, declared[arg])
                except ValueError as e:
                    problems.append(f'{name}: default of {arg}: {e}')
        info = describe(path, worlds_dir)
        for preset in info['presets']:
            try:
                validate(path, preset['args'], worlds_dir)
            except ValueError as e:
                problems.append(f"{name}: preset {preset['name']}: {e}")
        for arg, spec in info['args'].items():
            for other, value in (spec.get('show_if') or {}).items():
                if other not in info['args']:
                    problems.append(f'{name}: show_if of {arg}: unknown argument {other}')
                    continue
                try:
                    parse(info['args'][other], str(value))
                except ValueError as e:
                    problems.append(f'{name}: show_if of {arg}: {e}')
        previews = m.get('previews') or {}
        cameras = previews.get('cameras') or {}
        for arg, example in (previews.get('args') or {}).items():
            if arg not in info['args']:
                problems.append(f'{name}: previews of an unknown argument {arg}')
                continue
            if example.get('camera') not in cameras:
                problems.append(f"{name}: previews of {arg}: unknown camera {example.get('camera')}")
            for value in example.get('values', []):
                try:
                    parse(info['args'][arg], str(value))
                except ValueError as e:
                    problems.append(f'{name}: previews of {arg}: {e}')
        if previews.get('world') and previews['world'] not in cameras:
            problems.append(f"{name}: previews: unknown world camera {previews['world']}")
    return problems


def main():
    if len(sys.argv) > 2 and sys.argv[1] == '--build':
        try:
            print(build(sys.argv[2], parse_args(' '.join(sys.argv[3:]))))
        except ValueError as e:
            sys.exit(str(e))
        return
    if '--check' in sys.argv:
        problems = check()
        print('\n'.join(problems) or 'All world manifests match their worlds.')
        sys.exit(1 if problems else 0)
    if '--json' in sys.argv:
        print(json.dumps(describe_all(), ensure_ascii=False, indent=1))
        return
    for info in describe_all():
        print(f"{info['name']}: {info['title']}")
        for arg, spec in info['args'].items():
            choices = f" ({', '.join(map(str, spec['choices']))})" if 'choices' in spec else ''
            print(f"    {arg} = {spec['default']}  [{spec['type']}{choices}]  {spec.get('label', '')}")


if __name__ == '__main__':
    main()
