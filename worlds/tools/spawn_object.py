"""
Put objects of the object library (include/objects.yaml) into a running
simulation, or take them away, e.g. for a GUI placing them on a map.

    python3 worlds/tools/spawn_object.py --list                     # the library as JSON
    python3 worlds/tools/spawn_object.py --world garden --name box_1 --type cardboard_box_01 --x 1 --y 2 --z 0.05 --yaw 0.5 --scale 0.345
    python3 worlds/tools/spawn_object.py --world garden --remove box_1

The object is the same model the worlds scatter (include/objects.xacro); its
origin is on the ground under its center, so z is the ground height there.
Needs a sourced ROS environment (xacro, ros_gz_sim, gz).
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile

import yaml

INCLUDE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'include')


def library():
    with open(os.path.join(INCLUDE_DIR, 'objects.yaml')) as f:
        return yaml.safe_load(f)


def model_sdf(name, kind, scale=1.0):
    """SDF of one library object at the origin, scaled by 'scale'."""
    import xacro
    document = f'''<?xml version="1.0"?>
<sdf version="1.9" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="{os.path.join(INCLUDE_DIR, 'objects.xacro')}"/>
  <xacro:scatter_object name="{name}" type="{kind}" x="0" y="0" yaw="0" scale="{scale}"/>
</sdf>'''
    with tempfile.NamedTemporaryFile('w', suffix='.sdf.xacro', delete=False) as f:
        f.write(document)
    try:
        return xacro.process_file(f.name).toxml()
    finally:
        os.unlink(f.name)


def spawn(world, name, kind, x, y, z, yaw, scale=1.0):
    command = ['ros2', 'run', 'ros_gz_sim', 'create', '-world', world, '-name', name, '-string', model_sdf(name, kind, scale),
               '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw)]
    return subprocess.run(command).returncode


def remove(world, name):
    command = ['gz', 'service', '-s', f'/world/{world}/remove', '--reqtype', 'gz.msgs.Entity',
               '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', f'name: "{name}" type: MODEL']
    return subprocess.run(command).returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--list', action='store_true', help='print the object library as JSON')
    parser.add_argument('--world', help='name of the running world (<world name="..."> of its SDF)')
    parser.add_argument('--name', help='model name of the new object')
    parser.add_argument('--type', help='object type from objects.yaml')
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--z', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--scale', type=float, default=1.0, help="object size factor: the world's objects_scale")
    parser.add_argument('--remove', metavar='NAME', help='take this object away')
    options = parser.parse_args()
    if options.list:
        print(json.dumps([dict(name=k, kind=v['kind'], size=v['size']) for k, v in library().items()]))
        return
    if not options.world:
        sys.exit('--world is needed')
    if options.remove:
        sys.exit(remove(options.world, options.remove))
    if options.type not in library():
        sys.exit(f"Unknown object type {options.type}; see --list")
    sys.exit(spawn(options.world, options.name or options.type, options.type, options.x, options.y, options.z,
                   options.yaw, options.scale))


if __name__ == '__main__':
    main()
