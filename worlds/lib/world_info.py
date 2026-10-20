"""
Descriptions of the worlds and their arguments, and building their SDF, for
the launch file and for tools such as a GUI.

A xacro world <name>.sdf.xacro can have a manifest <name>.world.yaml next to
it, describing the world and each of its arguments:

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
                                  # (a worlds/include/<name>.yaml file)
        label: Relief
        description: ...
        group: terrain
      terrain_seed: {type: int, min: 0, label: Terrain seed}
      friction: {type: float, label: Friction, advanced: true}   # shown under 'more options' in a UI
      seed: {type: int, show_if: {maze: random}}   # shown only while other arguments have these values
      layout: {internal: true}    # set by the launch file, not by users
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

Defaults come from the <xacro:arg> declarations of the world itself, so they
are never duplicated. Every non-internal xacro argument must be in the
manifest (check() reports the ones missing).

Command line, for tools:
    python3 world_info.py --json            # all worlds, their arguments and defaults
    python3 world_info.py --check           # manifests match the worlds
"""

import json
import os
import re
import sys
import tempfile
import xml.etree.ElementTree as ET

import yaml

WORLDS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INCLUDE_DIR = os.path.join(WORLDS_DIR, 'include')
EXTENSIONS = ('.sdf.xacro', '.sdf', '.world')
TYPES = ('bool', 'int', 'float', 'string', 'choice')


def world_name(filename):
    """World name for a world file name, or None if it is not a world file."""
    for ext in EXTENSIONS:
        if filename.endswith(ext):
            return filename[:-len(ext)]
    return None


def world_files(worlds_dir=WORLDS_DIR):
    """{name: path} of the world files in a worlds folder."""
    files = {}
    for f in sorted(os.listdir(worlds_dir)):
        name = world_name(f)
        if name and name not in files:
            files[name] = os.path.join(worlds_dir, f)
    return files


def xacro_defaults(path):
    """{name: default} of the args declared with <xacro:arg> in a xacro world, in order."""
    with open(path) as f:
        return dict(re.findall(r'<xacro:arg\s+name="([^"]+)"\s+default="([^"]*)"', f.read()))


def resolve(world, worlds_dir=WORLDS_DIR):
    """Path of a world given by name (with or without extension) or by path; raise ValueError if there is none."""
    if os.path.isfile(world):
        return os.path.abspath(world)
    for candidate in [world] + [world + ext for ext in EXTENSIONS]:
        path = os.path.join(worlds_dir, candidate)
        if os.path.isfile(path):
            return path
    raise ValueError(f"World '{world}' not found. Available worlds: {', '.join(world_files(worlds_dir))} "
                     f"(or pass a path to an SDF file).")


def world_sdf(path, args):
    """
    SDF of a world file: an SDF file as it is, or a xacro world processed with {arg: string value},
    after its generator <name>/generator.py, if it has one, turns all arguments into the xacro mappings.
    """
    if not path.endswith('.xacro'):
        with open(path) as f:
            return f.read()
    import importlib.util
    import xacro
    mappings = dict(xacro_defaults(path), **args)
    generator = os.path.join(os.path.dirname(path), world_name(os.path.basename(path)), 'generator.py')
    if os.path.isfile(generator):
        sys.path.insert(0, os.path.dirname(generator))  # the generator's own modules, e.g. garden/terrain.py
        spec = importlib.util.spec_from_file_location('world_generator', generator)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        mappings = module.prepare(mappings)
    return xacro.process_file(path, mappings=mappings).toxml()


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
    Path of an SDF file for Gazebo of a world given by name or path: an SDF file as it is, or a xacro
    world built with {arg: string value}, checked against its manifest, into a temporary file.
    """
    path = resolve(world, worlds_dir)
    if not path.endswith('.xacro'):
        if args:
            raise ValueError(f"{os.path.basename(path)} is not a .sdf.xacro world, it takes no arguments.")
        return path
    validate(path, args, worlds_dir)
    with tempfile.NamedTemporaryFile('w', prefix='kpi_rover_world_', suffix='.sdf', delete=False) as f:
        f.write(world_sdf(path, args))
    return f.name


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
    """Option names from a worlds/include YAML file (floors, markings, walls)."""
    with open(os.path.join(worlds_dir, 'include', name + '.yaml')) as f:
        return list(yaml.safe_load(f))


def manifest(path):
    """The manifest of a world file, or an empty one."""
    for ext in EXTENSIONS:
        if path.endswith(ext):
            m = path[:-len(ext)] + '.world.yaml'
            if os.path.isfile(m):
                with open(m) as f:
                    return yaml.safe_load(f) or {}
    return {}


def describe(path, worlds_dir=WORLDS_DIR):
    """
    Description of a world file: {name, file, title, description, image, start_points: [{name, label, image}],
    groups, presets: [{name, label, description, args}],
    args: {name: {type, default, label, description, group, advanced, choices, min, max, images, ...}}}.
    Only args users set are listed (not internal ones). Images are paths of existing pictures or absent.
    """
    m = manifest(path)
    name = world_name(os.path.basename(path))
    pictures = os.path.join(os.path.dirname(path), name, 'previews')

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
    if not path.endswith('.xacro'):
        return info
    specs = m.get('args', {})
    for arg, default in xacro_defaults(path).items():
        spec = dict(specs.get(arg) or {})
        if spec.get('internal'):
            continue
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
                 args={k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in (p.get('args') or {}).items()})
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
    declared = xacro_defaults(path)
    for arg, value in values.items():
        if arg not in declared:
            raise ValueError(f"Unknown world argument '{arg}'. Arguments of {os.path.basename(path)}: "
                             f"{', '.join(info['args'])}.")
        if arg in info['args']:
            try:
                parse(info['args'][arg], value)
            except ValueError as e:
                raise ValueError(f"World argument '{arg}': {e}.") from e


def check(worlds_dir=WORLDS_DIR):
    """Problems of the manifests: xacro args without a spec, specs of args the world doesn't have, bad specs."""
    problems = []
    for name, path in world_files(worlds_dir).items():
        if not path.endswith('.xacro'):
            continue
        m = manifest(path)
        if not m:
            problems.append(f'{name}: no manifest {name}.world.yaml')
            continue
        declared = xacro_defaults(path)
        specs = m.get('args', {})
        for arg in declared:
            if arg not in specs:
                problems.append(f'{name}: argument {arg} is not in the manifest')
        for arg, spec in specs.items():
            if arg not in declared:
                problems.append(f'{name}: manifest argument {arg} is not declared in the world')
            elif not (spec or {}).get('internal'):
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
