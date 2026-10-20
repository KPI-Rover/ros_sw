"""
Download photo textures for floors and walls from Poly Haven (CC0) and
prepare them for the worlds. Writes textures/<name>/<name>_{albedo,normal,roughness}.jpg
(file names must be unique, as Gazebo caches textures by file name) and
textures/CREDITS.md, and prints the tile sizes for floors.yaml and walls.yaml.

Floor textures are repeated to cover about FLOOR_TILE meters, so floors need
few tiles (see the 'floor' macro). Wall textures are cropped to WALL_HEIGHT,
as a box maps a texture onto its whole face: walls are built of segments as
long as the texture is wide (see the 'wall' macro in structures.xacro).

Usage (needs internet access):
    python3 fetch_textures.py                  # all textures
    python3 fetch_textures.py floor_asphalt    # only the listed ones
"""

import io
import json
import os
import re
import sys
import urllib.request

from PIL import Image

API = 'https://api.polyhaven.com'

FLOOR_TILE = 2.0    # target floor tile size, m
WALL_HEIGHT = 0.5   # wall height the wall textures are made for, m
SIZE = 1024         # albedo and normal map width, px
ROUGHNESS_SIZE = 512

# Texture name: (kind, Poly Haven asset).
TEXTURES = {
    'floor_concrete': ('floor', 'concrete_floor_02'),
    'floor_painted_concrete': ('floor', 'concrete_floor_painted'),
    'floor_asphalt': ('floor', 'asphalt_pit_lane'),
    'floor_wood': ('floor', 'laminate_floor_02'),
    'floor_tiles': ('floor', 'floor_tiles_06'),
    'floor_carpet': ('floor', 'dirty_carpet'),
    'wall_brick': ('wall', 'red_brick_03'),
    'wall_plaster': ('wall', 'painted_plaster_wall'),
    'wall_concrete': ('wall', 'concrete_wall_007'),
    'wall_wood': ('wall', 'wood_planks'),
    'wall_metal': ('wall', 'corrugated_iron'),
    # Grounds and tree bark for outdoor worlds.
    'ground_grass': ('floor', 'leafy_grass'),
    'ground_sand': ('floor', 'sand_01'),
    'ground_gravel': ('floor', 'gravel_floor_02'),
    'ground_soil': ('floor', 'brown_mud_dry'),
    'ground_mud': ('floor', 'brown_mud_03'),
    'ground_rubble': ('floor', 'rock_ground'),
    'road_shoulder': ('floor', 'gravel_road'),
    'ground_meadow': ('floor', 'grass_ground'),
    'ground_loose_soil': ('floor', 'farm_soil'),
    'ground_forest_mud': ('floor', 'mud_forest'),
    'road_gravel': ('floor', 'gravel_ground_01'),
    'tree_bark': ('floor', 'bark_willow_02'),
}

MAPS = {'albedo': 'Diffuse', 'normal': 'nor_gl', 'roughness': 'Rough'}


def get(url):
    # Poly Haven rejects the default urllib user agent.
    request = urllib.request.Request(url, headers={'User-Agent': 'kpi_rover-fetch-textures'})
    with urllib.request.urlopen(request, timeout=60) as response:
        return response.read()


def prepare(image, kind, asset_size, width):
    """Repeat a floor texture or crop a wall texture, and resize it to 'width' px."""
    if kind == 'floor':
        repeat = max(1, round(FLOOR_TILE / asset_size))
        tiled = Image.new(image.mode, (image.width * repeat, image.height * repeat))
        for i in range(repeat):
            for j in range(repeat):
                tiled.paste(image, (i * image.width, j * image.height))
        return tiled.resize((width, width), Image.LANCZOS)
    height = round(image.height * min(1.0, WALL_HEIGHT / asset_size))
    cropped = image.crop((0, 0, image.width, height))
    return cropped.resize((width, max(1, round(width * height / image.width))), Image.LANCZOS)


def main():
    textures_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'media', 'textures')
    selected = sys.argv[1:] or list(TEXTURES)
    unknown = set(selected) - set(TEXTURES)
    if unknown:
        sys.exit(f"Unknown textures: {', '.join(sorted(unknown))}. Available: {', '.join(TEXTURES)}.")

    # Keep credit rows of textures that are not fetched now.
    credits_path = os.path.join(textures_dir, 'CREDITS.md')
    rows = {}
    if os.path.isfile(credits_path):
        for line in open(credits_path):
            match = re.match(r'\| (\w+) \|', line)
            if match and match.group(1) in TEXTURES:
                rows[match.group(1)] = line.rstrip('\n')

    for name in selected:
        kind, asset = TEXTURES[name]
        info = json.loads(get(f'{API}/info/{asset}'))
        files = json.loads(get(f'{API}/files/{asset}'))
        asset_size = info['dimensions'][0] / 1000
        out_dir = os.path.join(textures_dir, name)
        os.makedirs(out_dir, exist_ok=True)
        for map_name, key in MAPS.items():
            image = Image.open(io.BytesIO(get(files[key]['1k']['jpg']['url']))).convert('RGB')
            if map_name == 'roughness':
                image = image.convert('L')
            width = ROUGHNESS_SIZE if map_name == 'roughness' else SIZE
            prepare(image, kind, asset_size, width).save(os.path.join(out_dir, f'{name}_{map_name}.jpg'), quality=90)

        if kind == 'floor':
            covered = max(1, round(FLOOR_TILE / asset_size)) * asset_size
            print(f'{name}: tile {covered:.2f} m')
        else:
            covered = asset_size
            print(f'{name}: segment length {covered:.2f} m')
        authors = ', '.join(info.get('authors', {}))
        rows[name] = f'| {name} | [{asset}](https://polyhaven.com/a/{asset}) | {authors} | {covered:.2f} |'

    credits = ['# Texture credits', '',
               'Photo textures from [Poly Haven](https://polyhaven.com), licensed [CC0](https://polyhaven.com/license).',
               'Prepared by fetch_textures.py.', '',
               '| Texture | Poly Haven asset | Authors | Size covered, m |',
               '|---|---|---|---|']
    credits += [rows[name] for name in TEXTURES if name in rows]
    with open(credits_path, 'w') as f:
        f.write('\n'.join(credits) + '\n')


if __name__ == '__main__':
    main()
