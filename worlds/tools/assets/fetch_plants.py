"""
Download grass models from Poly Haven (CC0) for the garden into
worlds/media/models/<asset>/ (see tools/assets/fetch_models.py) and write
garden/plants.yaml. Each Poly Haven model is a set of grass tufts laid out in
a row; every tuft is written as its own glTF, <asset>_<tuft>.gltf, sharing the
model buffer, with the tuft at the origin. Plants are only seen: the robot
drives through them.

The blades are cut out of small cards by the texture transparency: the
Poly Haven glTF has the color texture without it, so the separate Alpha map
is merged into a PNG color texture and the material uses alpha masking.

Grass cards: the garden needs many more tufts than these models allow (each
has thousands of triangles), so most of the grass is cards: pictures of whole
tufts on a few crossed rectangles. The pictures are cut out of the model
textures, which have them for distant views: the tufts at the bottom of the
grass_medium_01 texture and the small plants of grass_bermuda_01. They are
packed into models/grass_cards/grass_cards.png with their places in
plants.yaml.

Leaf clusters: tree crowns are cards too (see convert_trees.py and
trees.py), with pictures of leaf clusters composed from the separate leaves
of the island_tree_02 texture (oval leaves, much like apple leaves) around a
twig end, a few variants side by side in models/apple_leaves/apple_clusters.png.

Usage (needs internet access):
    python3 fetch_plants.py
"""

import glob
import io
import json
import os

import yaml
from PIL import Image

import fetch_models

HERE = os.path.dirname(os.path.abspath(__file__))
GARDEN_DIR = os.path.join(HERE, '..', '..', 'garden')

PLANTS = ('grass_medium_01', 'grass_medium_02', 'grass_bermuda_01')
MIN_HEIGHT = 0.05    # smaller tufts are left out, m

# Card pictures: Poly Haven asset, real height of the tallest picture, m, and
# boxes (left, top, right, bottom as fractions of the texture): each is one
# picture, tightened to its opaque pixels, or, with 'search', a part of the
# texture where every separate picture is found.
# Plant models baked into card pictures (bake_cards.py): grass tufts of
# different kinds, a dry variant of the meadow grass, and a few weeds.
BAKED = ('grass_medium_01', 'grass_medium_02', 'grass_bermuda_01', 'weed_plant_02', 'dandelion_01', 'shrub_sorrel_01')
DRY = ('grass_medium_01',)  # also baked with their dry texture, as <asset>_dry
SOURCES_DIR = os.path.join(os.path.expanduser('~'), '.cache', 'kpi_rover', 'plant_sources')
ATLAS_WIDTH = 2048
CARD_PICTURE = 256          # longer side of a card picture in the atlas, px
# Card colors are matched to the grass ground: the green cards on average are
# CARD_BRIGHTER times as bright as the ground texture (as it looks with its
# tint), and the bottom CARD_FOOT of each picture fades into the ground color.
CARD_BRIGHTER = 1.15
CARD_FOOT = 0.15
CARDS_DIR = os.path.join(fetch_models.MODELS_DIR, 'grass_cards')

LEAVES_SOURCE = 'island_tree_02'
LEAVES_DIR = os.path.join(fetch_models.MODELS_DIR, 'apple_leaves')
CLUSTER_VARIANTS = 4        # leaf cluster pictures for the crowns of converted trees (convert_trees.py)
CLUSTER_SIZE = (768, 512)   # px; as long as the leaf cards of the tree models are (about 3:2)
CLUSTER_TWIGS = (6, 8)
CLUSTER_LEAVES = (8, 12)    # per twig
CLUSTER_LEAF = (0.17, 0.25) # leaf length, share of the picture width


def add_alpha(asset):
    """Merge the Poly Haven Alpha map into the model color texture and turn on alpha masking."""
    model_dir = os.path.join(fetch_models.MODELS_DIR, asset)
    path = os.path.join(model_dir, f'{asset}.gltf')
    with open(path) as f:
        gltf = json.load(f)
    files = json.loads(fetch_models.get(f'{fetch_models.API}/files/{asset}'))
    alpha = Image.open(io.BytesIO(fetch_models.get(files['Alpha']['1k']['jpg']['url']))).convert('L')
    material = gltf['materials'][0]
    image = gltf['images'][gltf['textures'][material['pbrMetallicRoughness']['baseColorTexture']['index']]['source']]
    color = Image.open(os.path.join(model_dir, image['uri'])).convert('RGB')
    rgba = color.copy()
    rgba.putalpha(alpha.resize(color.size, Image.LANCZOS))
    old = os.path.join(model_dir, image['uri'])
    image['uri'] = image['uri'].replace('.jpg', '.png')
    image['mimeType'] = 'image/png'
    rgba.save(os.path.join(model_dir, image['uri']))
    os.remove(old)
    material['alphaMode'] = 'MASK'
    material['alphaCutoff'] = 0.5
    with open(path, 'w') as f:
        json.dump(gltf, f)


def split(asset):
    """Write a glTF for every tuft of a model set and return [{name, mesh, height, radius}]."""
    model_dir = os.path.join(fetch_models.MODELS_DIR, asset)
    with open(os.path.join(model_dir, f'{asset}.gltf')) as f:
        gltf = json.load(f)
    tufts = []
    for index, node in enumerate(gltf['nodes']):
        if 'mesh' not in node:
            continue
        accessor = gltf['accessors'][gltf['meshes'][node['mesh']]['primitives'][0]['attributes']['POSITION']]
        lo, hi = accessor['min'], accessor['max']
        height = hi[1] - lo[1]
        if height < MIN_HEIGHT:
            continue
        name = node['name'].replace('_LOD0', '')
        if not name.startswith(asset):
            name = f'{asset}_{name}'
        one = dict(gltf, nodes=[dict(node, translation=[0, 0, 0])], scenes=[dict(nodes=[0])], scene=0)
        one['nodes'][0].pop('children', None)
        with open(os.path.join(model_dir, f'{name}.gltf'), 'w') as f:
            json.dump(one, f)
        radius = max(abs(v) for v in (lo[0], hi[0], lo[2], hi[2]))
        tufts.append(dict(name=name, mesh=f'models/{asset}/{name}.gltf', height=round(height, 3), radius=round(radius, 3)))
    return tufts


def components(mask):
    """Bounding boxes (left, top, right, bottom) of the connected parts of a boolean image."""
    import numpy as np
    label = np.where(mask, np.arange(mask.size).reshape(mask.shape), -1)
    # Spread the largest label over the neighbors until nothing changes.
    while True:
        new = label.copy()
        for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)):
            shifted = np.roll(np.roll(label, dy, 0), dx, 1)
            new = np.where(mask, np.maximum(new, shifted), -1)
        if (new == label).all():
            break
        label = new
    boxes = []
    for value in np.unique(label[label >= 0]):
        ys, xs = np.nonzero(label == value)
        boxes.append((xs.min(), ys.min(), xs.max() + 1, ys.max() + 1))
    return boxes


def download_sources():
    """Download the BAKED plant models (alpha merged) into SOURCES_DIR, and dry variants of the DRY ones."""
    import shutil
    models_dir, size = fetch_models.MODELS_DIR, fetch_models.TEXTURE_SIZE
    fetch_models.MODELS_DIR, fetch_models.TEXTURE_SIZE = SOURCES_DIR, 1024
    try:
        for asset in BAKED:
            if not os.path.isfile(os.path.join(SOURCES_DIR, asset, asset + '.gltf')):
                fetch_models.fetch(asset, 0)
                add_alpha(asset)
            if asset in DRY and not os.path.isdir(os.path.join(SOURCES_DIR, asset + '_dry')):
                dry_dir = os.path.join(SOURCES_DIR, asset + '_dry')
                shutil.copytree(os.path.join(SOURCES_DIR, asset), dry_dir)
                os.rename(os.path.join(dry_dir, asset + '.gltf'), os.path.join(dry_dir, asset + '_dry.gltf'))
                files = json.loads(fetch_models.get(f'{fetch_models.API}/files/{asset}'))
                dry = Image.open(io.BytesIO(fetch_models.get(files['dry_diff']['1k']['png']['url']))).convert('RGB')
                for png in glob.glob(os.path.join(dry_dir, 'textures', '*_diff_*.png')):
                    alpha = Image.open(png).getchannel('A')
                    dry = dry.resize(alpha.size, Image.LANCZOS)
                    dry.putalpha(alpha)
                    dry.save(png)
    finally:
        fetch_models.MODELS_DIR, fetch_models.TEXTURE_SIZE = models_dir, size


def ground_color():
    """Average color of the grass ground as it is drawn: its texture times its tint (grounds.yaml), 0..1."""
    import numpy as np
    with open(os.path.join(GARDEN_DIR, 'grounds.yaml')) as f:
        grass = yaml.safe_load(f)['grass']
    path = os.path.join(fetch_models.MODELS_DIR, '..', 'textures', grass['texture'], grass['texture'] + '_albedo.jpg')
    mean = np.asarray(Image.open(path).convert('RGB')).reshape(-1, 3).mean(0) / 255
    return mean * np.array(grass.get('color', [1.0, 1.0, 1.0]))


def match_colors(sprites, pictures):
    """Bring the card colors to the grass ground color (in place) and fade the picture feet into it."""
    import numpy as np
    ground = ground_color()
    data = [np.asarray(p).astype(float) / 255 for p in pictures]
    green = [d for s, d in zip(sprites, data) if s['name'].startswith('grass_') and '_dry__' not in s['name']]
    pixels = np.concatenate([d[d[..., 3] > 0.5][:, :3] for d in green])
    gain = ground * CARD_BRIGHTER / pixels.mean(0)
    for i, (sprite, d) in enumerate(zip(sprites, data)):
        green_card = sprite['name'].startswith('grass_') and '_dry__' not in sprite['name']
        # Other plants (dry grass, weeds, flowers) keep more of their own color: a weaker
        # gain, and dry grass a little paler, as sun bleached grass is.
        rgb = d[..., :3] * (gain if green_card else gain ** 0.3)
        if '_dry__' in sprite['name']:
            gray = rgb.mean(-1, keepdims=True)
            rgb = gray + (rgb - gray) * 0.55
        h = d.shape[0]
        t = np.clip((np.arange(h) - h * (1 - CARD_FOOT)) / (h * CARD_FOOT), 0, 1)[:, None, None]
        rgb = rgb * (1 - 0.6 * t) + ground * 0.6 * t
        d[..., :3] = np.clip(rgb, 0, 1)
        pictures[i] = Image.fromarray((d * 255).astype(np.uint8))


def make_cards():
    """
    Bake card pictures of the BAKED plant models (bake_cards.py, run with
    Blender) and pack them into an atlas; return the plants.yaml 'cards' entry.
    """
    import subprocess
    import numpy as np
    download_sources()
    baked = os.path.join(SOURCES_DIR, '_baked')
    subprocess.run(['blender', '-b', '--python', os.path.join(HERE, 'bake_cards.py'), '--', SOURCES_DIR, baked],
                   check=True, stdout=subprocess.DEVNULL)
    with open(os.path.join(baked, 'cards.json')) as f:
        sizes = json.load(f)
    sprites, pictures = [], []
    for name, size in sorted(sizes.items()):
        picture = Image.open(os.path.join(baked, name + '.png')).convert('RGBA')
        w, h = picture.size
        box = picture.getchannel('A').point(lambda a: 255 if a > 127 else 0).getbbox()
        if not box:
            continue
        picture = picture.crop(box)
        k = min(1.0, CARD_PICTURE / max(picture.size))
        pictures.append(picture.resize((max(1, int(picture.width * k)), max(1, int(picture.height * k))), Image.LANCZOS))
        sprites.append(dict(name=name, width=round(size['width'] * picture.width / w, 3),
                            height=round(size['height'] * picture.height / h, 3)))
    match_colors(sprites, pictures)
    # Shelf packing, tallest first, with a margin against bleeding between pictures.
    order = sorted(range(len(pictures)), key=lambda i: -pictures[i].height)
    margin, x, y, shelf = 4, 0, 0, 0
    places = {}
    for i in order:
        pw, ph = pictures[i].size
        if x + pw + margin > ATLAS_WIDTH:
            x, y, shelf = 0, y + shelf + margin, 0
        places[i] = (x, y)
        x, shelf = x + pw + margin, max(shelf, ph)
    height = 1 << (y + shelf).bit_length()
    atlas = Image.new('RGBA', (ATLAS_WIDTH, height), (0, 0, 0, 0))
    for i, (x, y) in places.items():
        atlas.paste(pictures[i], (x, y))
        pw, ph = pictures[i].size
        sprites[i]['uv'] = [round(float(x / ATLAS_WIDTH), 5), round(float(y / height), 5),
                            round(float((x + pw) / ATLAS_WIDTH), 5), round(float((y + ph) / height), 5)]
    data = np.asarray(atlas).copy()
    # Transparent pixels get the average grass color, so mipmaps don't darken the edges.
    opaque = data[..., 3] > 127
    data[~opaque, :3] = data[opaque, :3].mean(0).astype(np.uint8)
    os.makedirs(CARDS_DIR, exist_ok=True)
    Image.fromarray(data).save(os.path.join(CARDS_DIR, 'grass_cards.png'))
    print(f'grass cards: {len(sprites)} pictures in a {ATLAS_WIDTH}x{height} atlas')
    return dict(atlas='models/grass_cards/grass_cards.png', sprites=sprites)


def leaf_pictures():
    """Separate leaves of the LEAVES_SOURCE texture as RGBA pictures."""
    import numpy as np
    files = json.loads(fetch_models.get(f'{fetch_models.API}/files/{LEAVES_SOURCE}'))
    color = Image.open(io.BytesIO(fetch_models.get(files['leaves_diff']['1k']['png']['url']))).convert('RGB')
    alpha = Image.open(io.BytesIO(fetch_models.get(files['leaves_alpha']['1k']['png']['url']))).convert('L')
    opaque = np.asarray(alpha) > 127
    w = opaque.shape[1]
    # Leaves are the tall parts right of the bark strip at the left edge.
    leaves = []
    for x0, y0, x1, y1 in components(opaque[:, w // 16:]):
        x0, x1 = x0 + w // 16, x1 + w // 16
        if y1 - y0 > 120 and (x1 - x0) / (y1 - y0) > 0.15:
            leaf = color.crop((x0, y0, x1, y1))
            leaf.putalpha(alpha.crop((x0, y0, x1, y1)))
            leaves.append(leaf)
    return leaves


def make_clusters(leaves):
    """
    Compose leaf cluster pictures into models/apple_leaves/apple_clusters.png:
    a few thin twigs across the picture with leaves along them at different
    angles and shades, overlapping, as a piece of a crown looks.
    """
    import math
    import random
    import numpy as np
    from PIL import ImageDraw, ImageEnhance
    rng = random.Random(2)
    w, h = CLUSTER_SIZE
    atlas = Image.new('RGBA', (w * CLUSTER_VARIANTS, h), (0, 0, 0, 0))
    # Each picture is composed on a larger canvas and then fitted whole into its cell.
    pad = w // 2
    for v in range(CLUSTER_VARIANTS):
        cluster = Image.new('RGBA', (w + 2 * pad, h + 2 * pad), (0, 0, 0, 0))
        draw = ImageDraw.Draw(cluster)
        placed = []
        for _ in range(rng.randint(*CLUSTER_TWIGS)):
            # A twig across the picture, a little bent.
            a = math.radians(rng.uniform(-35, 35) + rng.choice((0, 180)))
            x0, y0 = pad + rng.uniform(0.3, 0.7) * w, pad + rng.uniform(0.3, 0.7) * h
            length = rng.uniform(0.45, 0.7) * w
            bend = rng.uniform(-0.3, 0.3)
            points = []
            for t in np.linspace(-0.5, 0.5, 9):
                ang = a + bend * t
                points.append((x0 + math.cos(ang) * t * length, y0 + math.sin(ang) * t * length))
            draw.line(points, fill=(88, 70, 52, 255), width=4, joint='curve')
            for i in range(rng.randint(*CLUSTER_LEAVES)):
                t = rng.uniform(0, 1)
                px, py = points[min(8, int(t * 8))]
                # Leaves grow forward and to the sides of the twig, alternately.
                side = 1 if i % 2 else -1
                angle = math.degrees(-a) + 90 - side * rng.uniform(25, 75)
                size = w * rng.uniform(*CLUSTER_LEAF)
                leaf = rng.choice(leaves)
                leaf = leaf.resize((max(1, int(size * leaf.width / leaf.height)), int(size)), Image.LANCZOS)
                rgb, alpha = leaf.convert('RGB'), leaf.getchannel('A')
                rgb = ImageEnhance.Brightness(rgb).enhance(rng.uniform(0.75, 1.1))
                leaf = rgb.convert('RGBA')
                leaf.putalpha(alpha)
                leaf = leaf.rotate(angle, expand=True, resample=Image.BICUBIC)
                # The stalk (picture bottom before turning) sits on the twig.
                r = math.radians(angle)
                cx, cy = px - math.sin(r) * size * 0.45, py - math.cos(r) * size * 0.45
                placed.append((rng.random(), leaf, (int(cx - leaf.width / 2), int(cy - leaf.height / 2))))
        # Leaves in random order, so twigs cross over each other.
        for _, leaf, at in sorted(placed, key=lambda item: item[0]):
            cluster.alpha_composite(leaf, at)
        cluster = cluster.crop(cluster.getbbox())
        k = min(1.0, (w - 8) / cluster.width, (h - 8) / cluster.height)
        cluster = cluster.resize((int(cluster.width * k), int(cluster.height * k)), Image.LANCZOS)
        atlas.paste(cluster, (v * w + (w - cluster.width) // 2, (h - cluster.height) // 2))
    data = np.asarray(atlas).copy()
    opaque = data[..., 3] > 127
    data[~opaque, :3] = data[opaque, :3].mean(0).astype(np.uint8)
    os.makedirs(LEAVES_DIR, exist_ok=True)
    Image.fromarray(data).save(os.path.join(LEAVES_DIR, 'apple_clusters.png'))
    print(f'leaf clusters: {CLUSTER_VARIANTS} pictures')
    return dict(atlas='models/apple_leaves/apple_clusters.png', variants=CLUSTER_VARIANTS)


def main():
    tufts = {}
    credits_path = os.path.join(fetch_models.MODELS_DIR, 'CREDITS.md')
    credits = open(credits_path).read().rstrip('\n').split('\n')
    for asset in PLANTS:
        size, center = fetch_models.fetch(asset, 0)
        add_alpha(asset)
        info = json.loads(fetch_models.get(f'{fetch_models.API}/info/{asset}'))
        row = f"| [{asset}](https://polyhaven.com/a/{asset}) | {', '.join(info.get('authors', {}))} |"
        credits = [line for line in credits if f'[{asset}]' not in line] + [row]
        for tuft in split(asset):
            tufts[tuft.pop('name')] = tuft
        print(f'{asset}: set of {size} m')
    cards = make_cards()
    leaves = leaf_pictures()
    clusters = make_clusters(leaves)
    info = json.loads(fetch_models.get(f'{fetch_models.API}/info/{LEAVES_SOURCE}'))
    row = f"| [{LEAVES_SOURCE}](https://polyhaven.com/a/{LEAVES_SOURCE}) (leaves) | {', '.join(info.get('authors', {}))} |"
    credits = [line for line in credits if f'[{LEAVES_SOURCE}]' not in line] + [row]
    with open(os.path.join(GARDEN_DIR, 'plants.yaml'), 'w') as f:
        f.write('# Plants for the garden, written by fetch_plants.py.\n')
        f.write('# tufts: mesh - glTF of one tuft in worlds/media (Y-up: rolled 90 degrees about X);\n')
        f.write('#        height, radius - real tuft height and radius around its origin, m.\n')
        f.write('# cards: atlas - tuft pictures in worlds/media; sprites - their places in the atlas\n')
        f.write('#        (uv: left, top, right, bottom) and real sizes, m.\n')
        f.write('# clusters: atlas - leaf cluster pictures side by side, for the tree crowns.\n')
        yaml.safe_dump(dict(tufts=tufts, cards=cards, clusters=clusters), f, default_flow_style=None, sort_keys=False)
    with open(credits_path, 'w') as f:
        f.write('\n'.join(credits) + '\n')


if __name__ == '__main__':
    main()
