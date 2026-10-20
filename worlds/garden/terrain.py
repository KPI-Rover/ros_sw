"""
Garden terrain: a height map with natural relief, covered with grounds and
grass tufts.

Two reliefs are available:
    steppe - a real place (see fetch_relief.py and relief/): the overall
             slope and broad undulations come from real heights, smoothed;
             the small unevenness of the ground and a few shallow pits are
             added in real sizes. Almost flat, like the real steppe.
    hills  - made-up hilly relief: broad hills and hollows with bumps on them.
All real sizes are scaled down to the robot, see params.yaml. World axes are
x east and y north. The noise is defined in world coordinates, so the same
seed always gives the same terrain, whatever the grid.

Grounds: grass covers the garden, with natural patches of loose soil and of
mud, which gathers in hollows. A field road goes around the garden: two worn
wheel tracks of dirt and gravel with ruts pressed into them and low berms
beside them, grass between and along them, as a real two-track road is. Each ground has its own small relief: grass
tussocks, soil clods, smooth sunken mud. Ground boundaries are smooth curves:
the terrain triangles are cut exactly along them. Grass (fetch_plants.py)
grows on the grass ground and thickest along the patch edges, softening them:
mostly light cards, pictures of tufts on crossed rectangles, and some 3D tufts.

The terrain is written as:
    heightmap_<key>.png - 16-bit height map for the Gazebo <heightmap> collision;
                          black is the lowest point, white the highest
    ground_<name>_<key>.gltf - visual mesh of each ground on the same heights,
                          with UVs repeating its macro texture (textures.py)
    tufts_<asset>_<key>.gltf - 3D grass tufts of a Poly Haven model, merged
    grass_cards_<key>.gltf - all grass cards
    surroundings<i>_<key>.gltf - visual-only rings around the terrain up to the horizon
    terrain.yaml        - size, height range, file names for the world
The key is a hash of the parameters, unique for each terrain, as Gazebo caches
meshes and images by file name.
"""

import hashlib
import math
import os
import shutil

import numpy as np
import yaml
from PIL import Image

import gltf
import textures
import trees

HERE = os.path.dirname(os.path.abspath(__file__))
MEDIA_DIR = os.path.join(HERE, '..', 'media')


def _load(name):
    with open(os.path.join(HERE, name)) as f:
        return yaml.safe_load(f)


PARAMS = _load('params.yaml')
GROUNDS = _load('grounds.yaml')
PLANTS = _load('plants.yaml')
TUFTS, CARDS = PLANTS['tufts'], PLANTS['cards']
SCALE = PARAMS['robot_height'] * PARAMS['tree_height_robots'] / PARAMS['real_tree_height']

SIZE = 20.48        # terrain side, m
SAMPLES = 1025      # height map samples per side (2^n + 1, as Gazebo needs): a 2 cm grid
STEP = SIZE / (SAMPLES - 1)
# The visual meshes use every VISUAL_STEP-th sample: an 8 cm grid with smooth
# normals looks the same and has 16 times fewer triangles; it differs from the
# collision height map (which the robot drives on, 2 cm, smooth) by 1.7 mm on
# average and under 7 mm in 99% of places.
VISUAL_STEP = 4

# Surroundings: visual-only meshes around the terrain up to the horizon, so
# the terrain doesn't end at an edge: rings of (side, grid step), m, each
# coarser than the one inside it. A ring grid is aligned with the grid inside
# and overlaps its border a little, under it, so there are no cracks.
RINGS = ((122.88, 0.64), (2048.0, 20.48))
FAR_TILE = 16       # texture repeat of the far rings, in near texture repeats

RELIEFS = ('steppe', 'hills')

# --- real sizes, m (scaled down with SCALE) ---
STEPPE_SMOOTH = 40.0    # smoothing of the real heights (removes SRTM noise)
# Unevenness of a field: (wavelength, amplitude).
BROAD_LAYERS = ((30.0, 0.12), (10.0, 0.05))
SMALL_LAYERS = ((3.0, 0.04), (1.2, 0.025), (0.5, 0.012), (0.2, 0.005))
# Ground relief: grass tussocks, soil clods and raise, mud sinking.
TUSSOCKS = (0.6, 0.05)          # wavelength, height
CLODS = ((0.35, 0.035), (0.15, 0.015))
SOIL_RAISE = 0.02
MUD_SINK = 0.05
MUD_SMOOTH = 0.7                # part of the small unevenness mud smooths out
# Shallow pits: radius and depth ranges, and how many per 100 m^2 where they gather.
PITS_RADIUS = (0.3, 0.7)
PITS_DEPTH = (0.03, 0.08)
PITS_DENSITY = 1.5
PITS_GATHER = 25.0              # size of the places where pits gather
# Ground patches: share of the terrain, patch size (noise wavelength) and the
# width of the relief transition at their edges.
SOIL_SHARE, SOIL_SIZE = 0.13, 9.0
MUD_SHARE, MUD_SIZE = 0.12, 6.0
MUD_IN_HOLLOWS = 0.6            # how much mud prefers low places (0..1)
EDGE = 0.3
PATCH_FADE = 2.0               # world m at the terrain border over which the patches fade out
# Grass tufts per real m^2: on grass, and along the ground edges (both sides,
# within TUFTS_EDGE_BAND). They are cards: pictures of tufts on 2 crossed
# rectangles, 4 triangles each. 3D tufts of separate blades (Poly Haven
# models) can be added too, but cost much more than they add among the cards.
CARDS_DENSITY = 10.0
CARDS_EDGE_DENSITY = 14.0
# How often each kind of plant picture is picked (by the start of its name, see
# fetch_plants.py): meadow grass mostly, some of it dry, a few weeds.
CARD_WEIGHTS = (('grass_medium_01_dry', 0.15), ('grass_medium_01', 1.0), ('grass_medium_02', 0.6),
                ('grass_bermuda_01', 0.5), ('weed_plant_02', 0.15), ('dandelion_01', 0.1))
CARDS_PLANES = 2                # crossed rectangles per card
TUFTS_DENSITY = 0.0             # 3D tufts are off: they cost much more than they add among the cards
TUFTS_EDGE_DENSITY = 0.0
TUFTS_BLADES = 0.6
# The grass_medium_01 tufts are the heaviest and hardly show among the cards.
TUFTS_SKIP = ('grass_medium_01',)
TUFTS_EDGE_BAND = 0.5
TUFTS_SIZE = (0.9, 1.6)         # random scale of the tufts and cards

# --- hills: made-up relief, world sizes ---
HILLS_LAYERS = ((14.0, 0.5), (6.0, 0.18), (2.5, 0.04))
HILLS_TILT = 0.01       # overall slope, m per m
HILLS_WARP = (8.0, 1.5)  # wavelength, shift, m
HORIZON_HILLS = ((40.0, 3.0), (15.0, 0.8))   # (wavelength, amplitude), m
HORIZON_RISE = 25.0     # distance from the terrain over which they rise to full height, m

OUTSIDE = 'grass'

# --- field road around the garden: two wheel tracks with gravel, grass between them ---
# Centerline: a rounded rectangle around the trees, world m, a little wavy.
ROAD_HALF = (7.4, 6.7)
ROAD_CORNER = 2.0
ROAD_WAVE = (6.0, 0.15)         # wavelength, sideways shift, world m
# Real sizes, m: a car track gauge, the worn tracks and the ruts in them.
GAUGE = 1.55                    # distance between the wheel track middles
TRACK_HALF = 0.3                # half width of a worn track (gravel and dirt)
TRACK_EDGE = (0.5, 0.07)        # wavelength, sideways wobble of the track edges
RUT_HALF = 0.25                 # half width of the rut pressed into a track
RUT_DEPTH = (0.04, 0.12)        # rut depth, changing along the road
BERM = 0.025                    # soil pushed up at the rut sides
ROAD_CLEAR = 1.0                # no soil and mud patches this far beyond the tracks

# --- pits and puddles, real sizes, m ---
# Garden pits between the tree rows: (radius, depth, water). Dry pits have
# bare soil, pits with water are muddy around it.
GARDEN_PITS = ((1.0, 0.15, False), (1.3, 0.25, True), (1.5, 0.35, False), (1.7, 0.5, True))
# Puddles on the road, in a wheel track: (side of the garden, along the side
# from -1 to 1, track: +1 outer or -1 inner, half length along the track, depth).
ROAD_PUDDLES = (('south', 0.45, 1, 1.4, 0.12), ('east', -0.3, -1, 1.8, 0.15), ('north', -0.55, 1, 1.2, 0.1))
PUDDLE_WIDTH = 0.45             # half width of a road puddle across the track, real m
WATER_LEVEL = 0.55              # water depth, share of the pit depth
WATER_COLOR = [0.07, 0.06, 0.04]    # muddy water
WATER_ROUGHNESS = 0.3

# --- start points: name and place: (road side, along it from -1 to 1, track)
# on the road, or (x world m, y in tree row spacings) in the garden ---
# On the road the robot stands in the middle (track 0), on the grass between
# the wheel tracks, which is as wide as the robot is: in a deep rut it would
# sit on its belly. It heads counterclockwise around the garden. In the
# garden it stands between tree rows, heading along them.
START_POINTS = (('road_a', ('south', -0.5, 0)), ('road_b', ('north', 0.5, 0)),
                ('garden_a', (-4.6, 0)), ('garden_b', (4.6, 1)))

# The height map is this much lower under mud and the wheel tracks, which
# have collision meshes of their own, with their own friction.
FRICTION_GAP = 0.01
OWN_COLLISION = ('mud', 'track')
GARDEN_GROUNDS = ('grass', 'soil', 'mud', 'track')


# Settings changed by the world arguments (see configure()); part of the cache key.
OPTIONS = {}
# The module settings before any configure(), which always starts from them.
_DEFAULTS = None


def configure(grass=1.0, soil=None, mud=None, pits=True, puddles=True, tree_rows=None,
              grass_friction=None, mud_friction=None, track_friction=None):
    """
    Change the module settings by the garden world arguments: grass density
    (a multiplier), soil and mud shares of the garden, pits and road puddles
    on or off, the number of tree rows and the friction of the grounds.
    """
    global CARDS_DENSITY, CARDS_EDGE_DENSITY, SOIL_SHARE, MUD_SHARE, GARDEN_PITS, ROAD_PUDDLES, GROUNDS, _DEFAULTS
    # Start from the defaults, so configuring again (e.g. for another variant in the same process) doesn't stack.
    if _DEFAULTS is None:
        _DEFAULTS = (CARDS_DENSITY, CARDS_EDGE_DENSITY, SOIL_SHARE, MUD_SHARE, GARDEN_PITS, ROAD_PUDDLES,
                     dict(GROUNDS), trees.ROWS)
    (CARDS_DENSITY, CARDS_EDGE_DENSITY, SOIL_SHARE, MUD_SHARE, GARDEN_PITS, ROAD_PUDDLES,
     GROUNDS, trees.ROWS) = _DEFAULTS[:6] + (dict(_DEFAULTS[6]), _DEFAULTS[7])
    OPTIONS.clear()
    CARDS_DENSITY *= grass
    CARDS_EDGE_DENSITY *= grass
    if soil is not None:
        SOIL_SHARE = soil
    if mud is not None:
        MUD_SHARE = mud
    if not pits:
        GARDEN_PITS = ()
    if not puddles:
        ROAD_PUDDLES = ()
    if tree_rows is not None:
        trees.ROWS = tree_rows
    for ground, mu in (('grass', grass_friction), ('soil', grass_friction), ('mud', mud_friction),
                       ('track', track_friction)):
        if mu is not None:
            GROUNDS[ground] = dict(GROUNDS[ground], mu=mu)
    OPTIONS.update(grass=grass, soil=SOIL_SHARE, mud=MUD_SHARE, pits=bool(pits), puddles=bool(puddles),
                   tree_rows=trees.ROWS)


def _hash(ix, iy, seed):
    """Pseudo-random 32-bit integers for integer lattice points."""
    h = (ix.astype(np.int64) * 374761393 + iy.astype(np.int64) * 668265263 + seed * 2147483647) & 0xFFFFFFFF
    h = ((h ^ (h >> 13)) * 1274126177) & 0xFFFFFFFF
    return h ^ (h >> 16)


def perlin(x, y, seed):
    """2D gradient noise in about [-1, 1] with a lattice step of 1."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    x0 = np.floor(x)
    y0 = np.floor(y)
    fx = x - x0
    fy = y - y0
    ix = x0.astype(np.int64)
    iy = y0.astype(np.int64)

    def grad(dx, dy):
        angle = _hash(ix + dx, iy + dy, seed) * (2 * math.pi / 2 ** 32)
        return np.cos(angle) * (fx - dx) + np.sin(angle) * (fy - dy)

    u = fx ** 3 * (fx * (fx * 6 - 15) + 10)
    v = fy ** 3 * (fy * (fy * 6 - 15) + 10)
    bottom = grad(0, 0) + u * (grad(1, 0) - grad(0, 0))
    top = grad(0, 1) + u * (grad(1, 1) - grad(0, 1))
    return (bottom + v * (top - bottom)) * math.sqrt(2)


def noise_layers(x, y, layers, seed):
    """Sum of noise layers [(wavelength, amplitude)], each rotated and shifted so their lattices don't line up."""
    rng = np.random.default_rng(seed)
    z = 0
    for i, (wavelength, amplitude) in enumerate(layers):
        a = rng.uniform(0, 2 * math.pi)
        ox, oy = rng.uniform(0, 1000, 2)
        u = (x * math.cos(a) - y * math.sin(a)) / wavelength + ox
        v = (x * math.sin(a) + y * math.cos(a)) / wavelength + oy
        z = z + amplitude * perlin(u, v, seed * 31 + i)
    return z


def smoothstep(t):
    t = np.clip(t, 0, 1)
    return t * t * (3 - 2 * t)


def _gaussian(a, sigma):
    """Separable Gaussian blur with edge padding; sigma in samples."""
    r = int(3 * sigma) + 1
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2)
    k /= k.sum()
    a = np.pad(a, r, mode='edge')
    a = np.apply_along_axis(lambda v: np.convolve(v, k, mode='valid'), 0, a)
    return np.apply_along_axis(lambda v: np.convolve(v, k, mode='valid'), 1, a)


class RealHeights:
    """Smoothed real heights from relief/<name>.png, sampled at real east/north offsets from the place."""

    def __init__(self, name):
        with open(os.path.join(HERE, 'relief', name + '.yaml')) as f:
            self.info = yaml.safe_load(f)
        data = np.asarray(Image.open(os.path.join(HERE, 'relief', name + '.png'))).astype(float) / 65535
        h = self.info['h_min'] + data * (self.info['h_max'] - self.info['h_min'])
        self.h = _gaussian(h, STEPPE_SMOOTH / self.info['step'])
        self.h0 = self.sample(np.zeros(1), np.zeros(1))[0]

    def sample(self, east, north):
        step, half = self.info['step'], self.info['size'] / 2
        n = self.h.shape[0]
        px = np.clip((east + half) / step, 0, n - 1.001)
        py = np.clip((half - north) / step, 0, n - 1.001)
        x0, y0 = px.astype(int), py.astype(int)
        fx, fy = px - x0, py - y0
        h = self.h
        return (h[y0, x0] * (1 - fx) * (1 - fy) + h[y0, x0 + 1] * fx * (1 - fy)
                + h[y0 + 1, x0] * (1 - fx) * fy + h[y0 + 1, x0 + 1] * fx * fy)


def real(layers):
    """Real (wavelength, amplitude) layers in world sizes."""
    return tuple((w * SCALE, a * SCALE) for w, a in layers)


class Terrain:
    """Relief and grounds of the terrain for a relief name and seed; all functions take world points."""

    def __init__(self, relief, seed):
        if relief not in RELIEFS:
            raise ValueError(f"Unknown relief '{relief}'. Available: {', '.join(RELIEFS)}.")
        self.relief = relief
        self.seed = seed
        self.pits, self.puddles = [], []
        if relief == 'steppe':
            self.real = RealHeights('steppe')
        # Patch thresholds give the wanted shares of the garden inside the road.
        x, y = grid(SIZE, SIZE / 256)
        inside = (self.road(x, y) < -(GAUGE / 2 + TRACK_HALF + ROAD_CLEAR + 0.5))
        # A share of 0 puts the threshold above all noise values: no patches.
        self.soil_cut = np.quantile(self._soil_noise(x, y)[inside], 1 - SOIL_SHARE) if SOIL_SHARE > 0 else 10.0
        self.mud_cut = np.quantile(self._mud_noise(x, y)[inside], 1 - MUD_SHARE) if MUD_SHARE > 0 else 10.0
        self.place_features()

    def broad(self, x, y):
        """Broad relief: the shape of the land, m."""
        if self.relief == 'hills':
            rng = np.random.default_rng(self.seed)
            tilt_dir = rng.uniform(0, 2 * math.pi)
            z = HILLS_TILT * (x * math.cos(tilt_dir) + y * math.sin(tilt_dir))
            wavelength, shift = HILLS_WARP
            wx = x + shift * perlin(x / wavelength + 100.5, y / wavelength, self.seed * 31 + 101)
            wy = y + shift * perlin(x / wavelength, y / wavelength + 200.5, self.seed * 31 + 102)
            return z + noise_layers(wx, wy, HILLS_LAYERS, self.seed)
        z = (self.real.sample(x / SCALE, y / SCALE) - self.real.h0) * SCALE
        return z + noise_layers(x, y, real(BROAD_LAYERS), self.seed)

    def horizon(self, x, y):
        """Broad relief of the surroundings: for hills, rising into hills away from the terrain."""
        z = self.broad(x, y)
        if self.relief == 'hills':
            half = SIZE / 2
            d = np.hypot(np.maximum(np.abs(x) - half, 0), np.maximum(np.abs(y) - half, 0))
            rise = smoothstep(d / HORIZON_RISE)
            for i, (wavelength, amplitude) in enumerate(HORIZON_HILLS):
                # Mostly up: hills around rather than pits.
                z = z + rise * amplitude * (0.6 + perlin(x / wavelength + 500.5, y / wavelength + 700.5,
                                                         self.seed * 31 + 200 + i))
        return z

    def _warp(self, x, y, k):
        """Domain warp for the patches, so their edges wind naturally."""
        w = 6.0 * SCALE
        return (x + 2.0 * SCALE * perlin(x / w + 11.5 * k, y / w, self.seed * 31 + 500 + k),
                y + 2.0 * SCALE * perlin(x / w, y / w + 13.5 * k, self.seed * 31 + 510 + k))

    def _soil_noise(self, x, y):
        wx, wy = self._warp(x, y, 1)
        s = SOIL_SIZE * SCALE
        return noise_layers(wx, wy, ((s, 1.0), (s / 2.5, 0.35), (s / 7, 0.08)), self.seed * 7 + 1)

    def _mud_noise(self, x, y):
        wx, wy = self._warp(x, y, 2)
        s = MUD_SIZE * SCALE
        n = noise_layers(wx, wy, ((s, 1.0), (s / 2.5, 0.3), (s / 6, 0.08)), self.seed * 7 + 2)
        # Low places: the broad relief below its local average.
        r = 4.0 * SCALE
        average = sum(self.broad(x + dx, y + dy) for dx, dy in ((r, 0), (-r, 0), (0, r), (0, -r))) / 4
        hollow = np.tanh(-(self.broad(x, y) - average) / (0.05 * SCALE))
        return (1 - MUD_IN_HOLLOWS) * n + MUD_IN_HOLLOWS * 0.8 * hollow

    @staticmethod
    def _fade(x, y):
        """0 inside the terrain, rising to 1 at its border: patches fade out there, grass goes on."""
        return smoothstep((np.maximum(np.abs(x), np.abs(y)) - (SIZE / 2 - PATCH_FADE)) / PATCH_FADE)

    def road(self, x, y):
        """Sideways offset from the road centerline, real m (negative inside the ring)."""
        hx, hy = ROAD_HALF
        qx, qy = np.abs(x) - hx + ROAD_CORNER, np.abs(y) - hy + ROAD_CORNER
        sdf = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0) - ROAD_CORNER
        wavelength, shift = ROAD_WAVE
        sdf = sdf + shift * perlin(x / wavelength + 60.5, y / wavelength + 70.5, self.seed * 31 + 600)
        return sdf / SCALE

    @staticmethod
    def _track_distance(s):
        """Distance from the nearest wheel track middle, real m, for road offsets s."""
        return np.minimum(np.abs(s - GAUGE / 2), np.abs(s + GAUGE / 2))

    def track(self, x, y, s=None):
        """Wheel track field, real m: positive on the worn tracks."""
        s = self.road(x, y) if s is None else s
        wavelength, wobble = TRACK_EDGE
        edge = wobble * perlin(x / (wavelength * SCALE) + 3.5, y / (wavelength * SCALE) + 9.5, self.seed * 31 + 610)
        return TRACK_HALF + edge - self._track_distance(s)

    def _off_road(self, x, y, s=None):
        """0 away from the road, rising to 1 near the tracks: patches stay off the road."""
        s = self.road(x, y) if s is None else s
        return smoothstep((GAUGE / 2 + TRACK_HALF + ROAD_CLEAR - np.abs(s)) / 0.5)

    def road_point(self, side, along, track):
        """World point on a wheel track (track: +1 outer, -1 inner, 0 the road middle) of a road side, 'along' it from -1 to 1."""
        hx, hy = ROAD_HALF
        sx, sy = {'south': (0, -1), 'north': (0, 1), 'east': (1, 0), 'west': (-1, 0)}[side]
        target = track * GAUGE / 2
        if sx == 0:
            x = along * (hx - ROAD_CORNER)
            ys = np.linspace(sy * (hy - 1.0), sy * (hy + 1.0), 801)
            s = self.road(np.full_like(ys, x), ys)
            return x, float(ys[np.argmin(np.abs(s - target))])
        y = along * (hy - ROAD_CORNER)
        xs = np.linspace(sx * (hx - 1.0), sx * (hx + 1.0), 801)
        s = self.road(xs, np.full_like(xs, y))
        return float(xs[np.argmin(np.abs(s - target))]), y

    def place_features(self):
        """Places of the garden pits, road puddles and start points (world m)."""
        spacing = PARAMS['row_spacing'] * SCALE
        rng = np.random.default_rng(self.seed * 7 + 11)
        # Pits on the lines between the tree rows, away from the start points and each other.
        lines = (np.arange(trees.ROWS - 1) - (trees.ROWS - 2) / 2) * spacing if trees.ROWS > 1 else np.zeros(1)
        starts = [(x, y * spacing) for _, (x, y, *_) in START_POINTS if not isinstance(x, str)]
        self.pits = []
        for i, (r, depth, water) in enumerate(GARDEN_PITS):
            for _ in range(100):
                py = lines[rng.integers(len(lines))]
                px = rng.uniform(-trees.GARDEN_HALF_X + 1.5, trees.GARDEN_HALF_X - 1.5)
                taken = [(sx, sy) for sx, sy in starts] + [(qx, qy) for qx, qy, *_ in self.pits]
                if all(math.hypot(px - qx, py - qy) > 1.6 for qx, qy in taken):
                    break
            self.pits.append((px, py, r, depth, water, 0.0, r))
        self.puddles = []
        for side, along, track, r, depth in ROAD_PUDDLES:
            px, py = self.road_point(side, along, track)
            angle = 0.0 if side in ('south', 'north') else math.pi / 2
            self.puddles.append((px, py, r, depth, True, angle, PUDDLE_WIDTH))

    def feature_heights(self, x, y):
        """Height change of the garden pits and road puddles at grid points, m."""
        dz = np.zeros(np.shape(x))
        for feature in self.pits + self.puddles:
            px, py, r, depth = feature[:4]
            r, depth = r * SCALE, depth * SCALE
            c0, c1 = np.searchsorted(x[0], [px - r, px + r])
            r0, r1 = np.searchsorted(-y[:, 0], [-(py + r), -(py - r)])
            wx, wy = x[r0:r1, c0:c1], y[r0:r1, c0:c1]
            # Irregular edges: the distance wobbles around the pit.
            wobble = 1 + 0.18 * perlin(wx / (0.3 * SCALE) + px, wy / (0.3 * SCALE) + py, self.seed * 31 + 700)
            t = np.clip(feature_distance(feature, wx, wy) * wobble, 0, 1)
            # A bowl with steeper sides and a flatter bottom than a plain bump.
            dz[r0:r1, c0:c1] -= depth * (1 - t ** 2.5) ** 1.5
        return dz

    def _pit_field(self, x, y, water):
        """Positive around the garden pits with or without water: their ground, in real m."""
        f = np.full(np.shape(x), -10.0)
        for px, py, r, depth, wet, *_ in self.pits:
            if wet == water:
                f = np.maximum(f, 1.25 * r - np.hypot(x - px, y - py) / SCALE)
        return f

    def soil(self, x, y, s=None):
        """Soil patch field: positive inside the patches."""
        f = self._soil_noise(x, y) - self.soil_cut - 2 * self._fade(x, y) - 3 * self._off_road(x, y, s)
        return np.maximum(f, self._pit_field(x, y, False) / SOIL_SIZE)

    def mud(self, x, y, s=None):
        """Mud patch field: positive inside the patches."""
        f = self._mud_noise(x, y) - self.mud_cut - 2 * self._fade(x, y) - 3 * self._off_road(x, y, s)
        return np.maximum(f, self._pit_field(x, y, True) / MUD_SIZE)

    def fields(self, x, y):
        """Ground fields (soil, mud, track), each positive on its ground; see labels()."""
        s = self.road(x, y)
        return self.soil(x, y, s), self.mud(x, y, s), self.track(x, y, s)

    def weights(self, soil, mud, track):
        """Smooth weights (soil, mud, track) of the ground relief, rising over EDGE at the ground edges."""
        e = EDGE * SCALE
        # Patch field values are noise units changing by about 1 over a patch size; convert to meters.
        w_track = smoothstep(0.5 + track / 0.15)
        w_mud = smoothstep(0.5 + mud * MUD_SIZE * SCALE / e) * (1 - w_track)
        w_soil = smoothstep(0.5 + soil * SOIL_SIZE * SCALE / e) * (1 - w_mud - w_track)
        return w_soil, w_mud, w_track

    def ruts(self, x, y):
        """Height change of the road: ruts in the wheel tracks and low berms beside them, m."""
        d = self._track_distance(self.road(x, y))
        lo, hi = RUT_DEPTH
        depth = lo + (hi - lo) * smoothstep(0.5 + perlin(x / (8 * SCALE) + 1.5, y / (8 * SCALE) + 2.5, self.seed * 31 + 620))
        t = np.clip(d / RUT_HALF, 0, 1)
        rut = -depth * (1 - t * t) ** 2
        berm = BERM * np.exp(-((d - RUT_HALF - 0.08) / 0.08) ** 2)
        return (rut + berm) * SCALE

    def heights(self, x, y, fields):
        """Terrain heights with the ground relief, road and pits at grid points, m."""
        z = self.broad(x, y)
        w_soil, w_mud, w_track = self.weights(*fields)
        w_grass = 1 - w_soil - w_mud - w_track
        small = noise_layers(x, y, real(SMALL_LAYERS), self.seed * 3 + 1)
        z = z + small * (1 - MUD_SMOOTH * w_mud - 0.5 * w_track) + self.ruts(x, y)
        wavelength, height = TUSSOCKS
        tussocks = smoothstep(noise_layers(x, y, ((wavelength * SCALE, 1.0),), self.seed * 3 + 2) * 1.5 + 0.2)
        z = z + w_grass * tussocks * height * SCALE
        clods = noise_layers(x, y, real(CLODS), self.seed * 3 + 3)
        z = z + w_soil * (np.abs(clods) * 1.5 + SOIL_RAISE * SCALE)
        z = z - w_mud * MUD_SINK * SCALE
        return z + pits(x, y, self.seed) + self.feature_heights(x, y)


def labels(fields):
    """Ground at points from the fields (soil, mud, track): indices into GARDEN_GROUNDS; later ones win."""
    g = np.zeros(np.shape(fields[0]), int)
    for i, f in enumerate(fields):
        g[f > 0] = i + 1
    return g


def pits(x, y, seed):
    """Height changes of shallow pits at world grid points (inside the terrain only), m."""
    rng = np.random.default_rng(seed * 7 + 3)
    half = SIZE / 2 - 0.5
    area = (2 * half / SCALE) ** 2      # real m^2
    n = rng.poisson(area / 100 * PITS_DENSITY)
    px, py = rng.uniform(-half, half, (2, n))
    gather = perlin(px / (PITS_GATHER * SCALE) + 300.5, py / (PITS_GATHER * SCALE) + 900.5, seed * 31 + 300)
    keep = rng.uniform(0, 1, n) < np.clip(gather * 1.6 + 0.15, 0, 1)
    dz = np.zeros(np.shape(x))
    for cx, cy in zip(px[keep], py[keep]):
        r = rng.uniform(*PITS_RADIUS) * SCALE
        depth = rng.uniform(*PITS_DEPTH) * SCALE
        # Grid window around the pit; rows go along -y.
        c0, c1 = np.searchsorted(x[0], [cx - r, cx + r])
        r0, r1 = np.searchsorted(-y[:, 0], [-(cy + r), -(cy - r)])
        wx, wy = x[r0:r1, c0:c1], y[r0:r1, c0:c1]
        t = np.clip(np.hypot(wx - cx, wy - cy) / r, 0, 1)
        dz[r0:r1, c0:c1] -= depth * (1 - t * t) ** 2
    return dz


def grid(size=SIZE, step=STEP):
    """World coordinates of a square grid; row 0 is the +y edge, as in a Gazebo height map image."""
    n = int(round(size / step)) + 1
    c = (np.arange(n) / (n - 1) - 0.5) * size
    x, y = np.meshgrid(c, c[::-1])
    return x, y


def grid_normals(z, step):
    """Smooth normals of a grid from central differences; rows go along -y."""
    dzdx = np.gradient(z, step, axis=1)
    dzdy = -np.gradient(z, step, axis=0)
    n = np.stack([-dzdx, -dzdy, np.ones_like(z)], axis=-1)
    return n / np.linalg.norm(n, axis=-1, keepdims=True)


def grid_triangles(rows, cols, keep=None):
    """Counter-clockwise (from above) vertex index triangles of a grid, two per cell, for the kept cells."""
    idx = np.arange(rows * cols).reshape(rows, cols)
    a, b = idx[:-1, :-1], idx[:-1, 1:]   # upper row (larger y)
    c, d = idx[1:, :-1], idx[1:, 1:]     # lower row
    if keep is None:
        keep = np.ones(a.shape, bool)
    return np.concatenate([np.stack([c[keep], d[keep], b[keep]], -1), np.stack([c[keep], b[keep], a[keep]], -1)])


def clip(tris, f):
    """
    Cut triangles (M, 3, D) of vertex attributes along the zero line of field
    values f (M, 3), linear over each triangle. Return the triangles on the
    positive and on the negative side, keeping the vertex order.
    """
    s = f >= 0
    count = s.sum(1)
    pos, neg = [tris[count == 3]], [tris[count == 0]]
    for lone_positive in (True, False):
        sel = count == (1 if lone_positive else 2)
        if not sel.any():
            continue
        t, fs, ss = tris[sel], f[sel], s[sel]
        lone = np.argmax(ss if lone_positive else ~ss, axis=1)
        order = (lone[:, None] + np.arange(3)) % 3
        rows = np.arange(len(t))[:, None]
        t, fs = t[rows, order], fs[rows, order]
        a, b, c = t[:, 0], t[:, 1], t[:, 2]
        tab = (fs[:, 0] / (fs[:, 0] - fs[:, 1]))[:, None]
        tac = (fs[:, 0] / (fs[:, 0] - fs[:, 2]))[:, None]
        p = a + tab * (b - a)
        q = a + tac * (c - a)
        lone_tris = np.stack([a, p, q], 1)
        rest = np.concatenate([np.stack([p, b, c], 1), np.stack([p, c, q], 1)])
        (pos if lone_positive else neg).append(lone_tris)
        (neg if lone_positive else pos).append(rest)
    return np.concatenate(pos), np.concatenate(neg)


def ground_meshes(x, y, z, normals, fields):
    """
    Triangles of each ground: [(positions, normals, faces)] in GARDEN_GROUNDS
    order. Whole triangles keep the grid vertices; triangles on a boundary are
    cut along it. fields: (soil, mud, track) values at the grid vertices.
    """
    rows, cols = z.shape
    tris = grid_triangles(rows, cols)
    lab = labels(fields).reshape(-1)[tris]
    same = (lab == lab[:, :1]).all(1)
    pos = np.stack([x, y, z], -1).reshape(-1, 3)
    nrm = normals.reshape(-1, 3)
    out = []
    for g in range(len(GARDEN_GROUNDS)):
        used, faces = np.unique(tris[same & (lab[:, 0] == g)], return_inverse=True)
        out.append([pos[used], nrm[used], faces.reshape(-1, 3)])

    # Boundary triangles with attributes: position (3), normal (3), then the fields.
    # They are cut by the last field first, as later grounds win (see labels).
    bt = tris[~same]
    attrs = np.concatenate([pos[bt], nrm[bt]] + [f.reshape(-1)[bt][..., None] for f in fields], -1)
    parts = [None] * len(GARDEN_GROUNDS)
    rest = attrs
    for i in range(len(fields), 0, -1):
        parts[i], rest = clip(rest, rest[..., 5 + i])
    parts[0] = rest
    for g, part in enumerate(parts):
        if not len(part):
            continue
        p, n, f = out[g]
        n_new = part[..., 3:6].reshape(-1, 3)
        n_new /= np.linalg.norm(n_new, axis=-1, keepdims=True)
        out[g] = [np.concatenate([p, part[..., :3].reshape(-1, 3)]), np.concatenate([n, n_new]),
                  np.concatenate([f, len(p) + np.arange(len(part) * 3).reshape(-1, 3)])]
    return out


def feature_distance(feature, x, y):
    """Distance from a pit or puddle middle in its radii: 1 at its edge (an ellipse along its angle)."""
    px, py, r, _, _, angle, width = feature
    dx, dy = x - px, y - py
    along = dx * math.cos(angle) + dy * math.sin(angle)
    across = -dx * math.sin(angle) + dy * math.cos(angle)
    return np.hypot(along / (r * SCALE), across / (width * SCALE))


def write_water(path, x, y, z, puddles):
    """
    Water of the pits and puddles in one glTF: for each, the terrain cells
    below its water level, cut exactly along the shore and flattened to the
    level. Returns the water levels.
    """
    parts, levels = [], []
    for feature in puddles:
        px, py, r, depth = feature[:4]
        r, depth = r * SCALE, depth * SCALE
        c0, c1 = np.searchsorted(x[0], [px - r, px + r])
        r0, r1 = np.searchsorted(-y[:, 0], [-(py + r), -(py - r)])
        wx, wy, wz = x[r0:r1, c0:c1], y[r0:r1, c0:c1], z[r0:r1, c0:c1]
        level = float(wz.min() + WATER_LEVEL * depth)
        levels.append(level)
        # Under water: below the level and inside the pit (ruts nearby may be lower too).
        field = np.minimum(level - wz, (0.95 - feature_distance(feature, wx, wy)) * 0.1)
        tris = grid_triangles(*wz.shape)
        attrs = np.stack([wx, wy, field], -1).reshape(-1, 3)[tris]
        wet, _ = clip(attrs, attrs[..., 2])
        if len(wet):
            parts.append(np.concatenate([wet[..., :2], np.full(wet.shape[:2] + (1,), level)], -1))
    if not parts:
        return levels
    tris = np.concatenate(parts)
    p = tris.reshape(-1, 3)
    # The water material is set in the world (glTF colors without a texture are not used by Gazebo).
    gltf.write(path, p, np.tile([0.0, 0.0, 1.0], (len(p), 1)), p[:, :2] / 0.5, np.arange(len(p)).reshape(-1, 3))
    return levels


def start_points(terrain, z_at):
    """Start points: [{name, x, y, z, yaw}] (world m, rad)."""
    spacing = PARAMS['row_spacing'] * SCALE
    points = []
    for name, place in START_POINTS:
        if isinstance(place[0], str):
            x, y = terrain.road_point(*place)
            # Counterclockwise along the road: the road offset grows outward, the heading is across it.
            e = 0.01
            gx = (terrain.road(np.array(x + e), np.array(y)) - terrain.road(np.array(x - e), np.array(y))) / (2 * e)
            gy = (terrain.road(np.array(x), np.array(y + e)) - terrain.road(np.array(x), np.array(y - e))) / (2 * e)
            yaw = math.atan2(float(gx), -float(gy))
        else:
            x, y = place[0], place[1] * spacing
            yaw = 0.0 if x < 0 else math.pi
        z = float(z_at(np.array([x]), np.array([y]))[0])
        points.append(dict(name=name, x=round(float(x), 3), y=round(float(y), 3), z=round(z, 4), yaw=round(yaw, 4)))
    return points


def write_ground(path, positions, normals, faces, uv_tile):
    uvs = np.stack([positions[:, 0], -positions[:, 1]], -1) / uv_tile
    gltf.write(path, positions, normals, uvs, faces)


def scatter(terrain, z_at, rng, density, edge_density):
    """
    Random points (x, y, z) for plants: 'density' per real m^2 on the grass,
    plus 'edge_density' along the patch edges on both sides (not in mud).
    """
    half = SIZE / 2 - 0.2
    area = (2 * half / SCALE) ** 2      # real m^2
    peak = density + edge_density
    n = rng.poisson(area * peak)
    x, y = rng.uniform(-half, half, (2, n))
    soil, mud, track = terrain.fields(x, y)
    # Distance to the nearest ground edge, real m, from the field values.
    edge = np.minimum(np.minimum(np.abs(soil) * SOIL_SIZE, np.abs(mud) * MUD_SIZE), np.abs(track))
    on_grass = (soil <= 0) & (mud <= 0) & (track <= 0)
    d = density * on_grass + edge_density * np.exp(-(edge / TUFTS_EDGE_BAND) ** 2) * (mud <= 0) * (track <= 0)
    # Thinning out towards the terrain border, where the surroundings go on without plants.
    d = d * (1 - Terrain._fade(x, y))
    keep = rng.uniform(0, peak, n) < d
    return x[keep], y[keep], z_at(x[keep], y[keep])


def tufts(terrain, z_at, seed):
    """3D grass tuft placements: [(tuft name, x, y, z, yaw, scale)]."""
    rng = np.random.default_rng(seed * 7 + 5)
    x, y, z = scatter(terrain, z_at, rng, TUFTS_DENSITY, TUFTS_EDGE_DENSITY)
    names = [name for name in TUFTS if not name.startswith(TUFTS_SKIP)]
    return [(names[rng.integers(len(names))], xi, yi, zi, rng.uniform(0, 2 * math.pi), rng.uniform(*TUFTS_SIZE) * SCALE)
            for xi, yi, zi in zip(x, y, z)]


def write_cards(out_dir, key, terrain, z_at, seed):
    """Grass cards in one glTF: CARDS_PLANES crossed rectangles each, both sides lit alike; return ({name, mesh}, count)."""
    rng = np.random.default_rng(seed * 7 + 6)
    x, y, z = scatter(terrain, z_at, rng, CARDS_DENSITY, CARDS_EDGE_DENSITY)
    n = len(x)
    if not n:
        return None, 0
    sprites = CARDS['sprites']
    weights = np.array([next((wt for prefix, wt in CARD_WEIGHTS if s['name'].startswith(prefix + '__')), 0.3)
                        for s in sprites])
    pick = rng.choice(len(sprites), n, p=weights / weights.sum())
    w = np.array([s['width'] for s in sprites])[pick]
    h = np.array([s['height'] for s in sprites])[pick]
    uv = np.array([s['uv'] for s in sprites])[pick]
    scale = rng.uniform(*TUFTS_SIZE, n) * SCALE
    w, h = w * scale, h * scale
    yaw = rng.uniform(0, math.pi, n)
    positions, uvs = [], []
    for k in range(CARDS_PLANES):
        a = yaw + k * math.pi / CARDS_PLANES
        dx, dy = np.cos(a) * w / 2, np.sin(a) * w / 2
        bottom = z - 0.003      # a little into the ground, so no gap shows
        quad = [(x - dx, y - dy, bottom), (x + dx, y + dy, bottom), (x + dx, y + dy, z + h), (x - dx, y - dy, z + h)]
        positions.append(np.stack([np.stack(c, -1) for c in quad], 1))
        quad_uv = [(uv[:, 0], uv[:, 3]), (uv[:, 2], uv[:, 3]), (uv[:, 2], uv[:, 1]), (uv[:, 0], uv[:, 1])]
        uvs.append(np.stack([np.stack(c, -1) for c in quad_uv], 1))
    positions = np.concatenate(positions)     # (CARDS_PLANES * n, 4, 3)
    uvs = np.concatenate(uvs)
    # Both sides of each rectangle are separate faces, all with normals up, so a
    # card is lit like the ground under it from any side (a two-sided material
    # would flip the normal of the back side down and make that side dark).
    positions = np.concatenate([positions, positions])
    uvs = np.concatenate([uvs, uvs])
    m = len(positions) // 2
    base = np.arange(m)[:, None] * 4
    back = base + m * 4
    faces = np.concatenate([base + [0, 1, 2], base + [0, 2, 3], back + [0, 2, 1], back + [0, 3, 2]])
    atlas = os.path.basename(CARDS['atlas'])
    shutil.copy(os.path.join(MEDIA_DIR, CARDS['atlas']), os.path.join(out_dir, atlas))
    material = dict(name='grass_cards', alphaMode='MASK', alphaCutoff=0.5,
                    pbrMetallicRoughness=dict(baseColorTexture=dict(index=0), metallicFactor=0.0, roughnessFactor=1.0))
    path = os.path.join(out_dir, f'grass_cards_{key}.gltf')
    p = positions.reshape(-1, 3)
    gltf.write(path, p, np.tile([0.0, 0.0, 1.0], (len(p), 1)), uvs.reshape(-1, 2), faces,
               material=material, images=[atlas])
    return dict(name='grass_cards', mesh=path), n


def thin_out(p, n, uv, f, rng, min_faces=200):
    """Keep TUFTS_BLADES of the blades (connected parts) of a tuft mesh with more than min_faces triangles."""
    if len(f) <= min_faces:
        return p, n, uv, f
    # Connected parts: spread the smallest vertex label over each triangle until nothing changes.
    label = np.arange(len(p))
    while True:
        m = label[f].min(1)
        new = label.copy()
        np.minimum.at(new, f.reshape(-1), np.repeat(m, 3))
        new = new[new]
        if (new == label).all():
            break
        label = new
    parts = np.unique(label)
    keep_parts = parts[rng.uniform(0, 1, len(parts)) < TUFTS_BLADES]
    faces = f[np.isin(label[f[:, 0]], keep_parts)]
    used, faces = np.unique(faces, return_inverse=True)
    return p[used], n[used], uv[used], faces.reshape(-1, 3).astype(np.uint32)


def write_tufts(out_dir, key, placements):
    """Merge the tufts of each Poly Haven model into one glTF; return [{name, mesh}]."""
    by_asset = {}
    for item in placements:
        by_asset.setdefault(TUFTS[item[0]]['mesh'].split('/')[1], []).append(item)
    meshes = []
    for asset, items in sorted(by_asset.items()):
        src_dir = os.path.join(MEDIA_DIR, 'models', asset)
        parts, cache = [], {}
        for name, x, y, z, yaw, scale in items:
            if name not in cache:
                g, buffers = gltf.read(os.path.join(MEDIA_DIR, TUFTS[name]['mesh']))
                cache[name] = thin_out(*gltf.mesh_data(g, buffers, g['nodes'][0]['mesh']),
                                       np.random.default_rng(len(cache)))
                material, images = g['materials'][0], [i['uri'] for i in g['images']]
            p, n, uv, f = cache[name]
            # glTF Y-up (x, y, z) -> Z-up (x, -z, y), then scale, turn and move.
            c, s = math.cos(yaw), math.sin(yaw)
            rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], np.float32)
            p = np.stack([p[:, 0], -p[:, 2], p[:, 1]], -1) @ rot.T * scale + [x, y, z]
            n = np.stack([n[:, 0], -n[:, 2], n[:, 1]], -1) @ rot.T
            parts.append((p, n, uv, f))
        offsets = np.cumsum([0] + [len(part[0]) for part in parts[:-1]])
        # Copy the textures next to the merged mesh; their names include the asset name.
        for uri in images:
            shutil.copy(os.path.join(src_dir, uri), os.path.join(out_dir, os.path.basename(uri)))
        path = os.path.join(out_dir, f'tufts_{asset}_{key}.gltf')
        gltf.write(path, np.concatenate([part[0] for part in parts]), np.concatenate([part[1] for part in parts]),
                   np.concatenate([part[2] for part in parts]),
                   np.concatenate([part[3] + o for part, o in zip(parts, offsets)]),
                   material=material, images=[os.path.basename(u) for u in images])
        meshes.append(dict(name=asset, mesh=path))
    return meshes


def ground_color(ground):
    return GROUNDS[ground].get('color', [1.0, 1.0, 1.0])


def macro_tile(ground):
    """World size of one repeat of the ground macro texture, m."""
    return GROUNDS[ground]['tile'] * SCALE * textures.GRID


def ground_texture(cache_dir, ground):
    """Directory of the macro texture of a ground (see textures.py), built once into cache_dir/textures."""
    texture = GROUNDS[ground]['texture']
    with open(os.path.join(textures.TEXTURES_DIR, texture, texture + '_albedo.jpg'), 'rb') as f:
        params = repr((texture, textures.GRID, textures.SIZE, textures.SHARPNESS, textures.BRIGHTNESS))
        params += hashlib.sha1(f.read()).hexdigest()
    name = f'garden_{ground}_' + hashlib.sha1(params.encode()).hexdigest()[:10]
    return textures.make(texture, os.path.join(cache_dir, 'textures'), name, seed=len(ground))


def build(cache_dir, relief_name, seed):
    """Build the terrain files into a cache_dir subdirectory (if not built yet) and return the terrain.yaml path."""
    # The key covers this code, the parameter files and the source data.
    params = repr((relief_name, seed, PARAMS, GROUNDS, PLANTS, sorted(OPTIONS.items())))
    for path in (__file__, textures.__file__, trees.__file__, os.path.join(HERE, 'trees.yaml'),
                 os.path.join(HERE, 'relief', 'steppe.png')):
        with open(path, 'rb') as f:
            params += hashlib.sha1(f.read()).hexdigest()
    key = hashlib.sha1(params.encode()).hexdigest()[:10]
    out_dir = os.path.join(cache_dir, 'terrain_' + key)
    info_path = os.path.join(out_dir, 'terrain.yaml')
    if os.path.isfile(info_path):
        return info_path

    os.makedirs(out_dir, exist_ok=True)
    terrain = Terrain(relief_name, seed)
    x, y = grid()
    fields = terrain.fields(x, y)
    z = terrain.heights(x, y, fields)
    z_offset = z.mean()
    z -= z_offset
    # The height map is lower under the grounds with their own collision meshes.
    lab = labels(fields)
    own = np.isin(lab, [GARDEN_GROUNDS.index(g) for g in OWN_COLLISION])
    z_collision = z - FRICTION_GAP * own
    z_min, z_max = float(z_collision.min()), float(z_collision.max())
    heightmap = os.path.join(out_dir, f'heightmap_{key}.png')
    data = np.round((z_collision - z_min) / (z_max - z_min) * 65535)
    Image.fromarray(data.astype(np.uint16)).save(heightmap)

    grounds = []
    k = VISUAL_STEP
    meshes = ground_meshes(x[::k, ::k], y[::k, ::k], z[::k, ::k], grid_normals(z[::k, ::k], STEP * k),
                           [f[::k, ::k] for f in fields])
    for name, (p, n, f) in zip(GARDEN_GROUNDS, meshes):
        if not len(f):
            continue
        mesh = os.path.join(out_dir, f'ground_{name}_{key}.gltf')
        write_ground(mesh, p, n, f, macro_tile(name))
        grounds.append(dict(name=name, mesh=mesh, texture=ground_texture(cache_dir, name), color=ground_color(name),
                            collision=name in OWN_COLLISION, mu=GROUNDS[name]['mu']))

    def z_at(px, py):
        """Terrain heights at points, bilinear on the grid."""
        c = np.clip((px + SIZE / 2) / STEP, 0, SAMPLES - 1.001)
        r = np.clip((SIZE / 2 - py) / STEP, 0, SAMPLES - 1.001)
        c0, r0 = c.astype(int), r.astype(int)
        fc, fr = c - c0, r - r0
        return (z[r0, c0] * (1 - fc) * (1 - fr) + z[r0, c0 + 1] * fc * (1 - fr)
                + z[r0 + 1, c0] * (1 - fc) * fr + z[r0 + 1, c0 + 1] * fc * fr)

    placements = tufts(terrain, z_at, seed) if TUFTS_DENSITY + TUFTS_EDGE_DENSITY > 0 else []
    plants = write_tufts(out_dir, key, placements) if placements else []
    cards, card_count = write_cards(out_dir, key, terrain, z_at, seed)
    if card_count:
        plants.append(cards)
    orchard = trees.build(out_dir, key, z_at, seed, PLANTS['clusters'])
    water = os.path.join(out_dir, f'water_{key}.gltf')
    wet = [f for f in terrain.pits + terrain.puddles if f[4]]
    write_water(water, x, y, z, wet)
    water = water if os.path.isfile(water) else ''

    features = dict(water=water, water_color=WATER_COLOR, water_roughness=WATER_ROUGHNESS,
                    pits=[dict(x=round(float(px), 3), y=round(float(py), 3), radius=round(r * SCALE, 3),
                               depth=round(d * SCALE, 3), water=w) for px, py, r, d, w, *_ in terrain.pits],
                    puddles=[dict(x=round(float(px), 3), y=round(float(py), 3), length=round(2 * r * SCALE, 3),
                                  depth=round(d * SCALE, 3)) for px, py, r, d, *_ in terrain.puddles])
    starts = start_points(terrain, z_at)

    # Surrounding rings without the cells well inside the ring inside them;
    # the cells along its border are kept and lowered a little, under it.
    surroundings = []
    inner = SIZE
    for i, (ring_size, ring_step) in enumerate(RINGS):
        ox, oy = grid(ring_size, ring_step)
        oz = terrain.horizon(ox, oy) - z_offset
        inside = (np.abs(ox) < inner / 2 - 1e-6) & (np.abs(oy) < inner / 2 - 1e-6)
        # Lowered by more on coarse rings, as their straight cells cut across the curved relief.
        oz[inside] -= 0.025 * ring_step + 0.01
        cell_inside = inside[:-1, :-1] & inside[:-1, 1:] & inside[1:, :-1] & inside[1:, 1:]
        deep = (np.abs(ox[:-1, :-1] + ring_step / 2) < inner / 2 - ring_step)
        deep &= (np.abs(oy[:-1, :-1] - ring_step / 2) < inner / 2 - ring_step)
        used, faces = np.unique(grid_triangles(*oz.shape, keep=~(cell_inside & deep)), return_inverse=True)
        p = np.stack([ox, oy, oz], -1).reshape(-1, 3)[used]
        n = grid_normals(oz, ring_step).reshape(-1, 3)[used]
        mesh = os.path.join(out_dir, f'surroundings{i}_{key}.gltf')
        # Far rings are seen only from afar: a much larger texture repeat avoids ripples near the horizon.
        write_ground(mesh, p, n, faces.reshape(-1, 3), macro_tile(OUTSIDE) * (1 if i == 0 else FAR_TILE))
        surroundings.append(dict(name=f'ring{i}', mesh=mesh, texture=ground_texture(cache_dir, OUTSIDE),
                                 color=ground_color(OUTSIDE)))
        inner = ring_size

    slope = np.degrees(np.arctan(np.hypot(*np.gradient(z, STEP))))
    info = dict(
        relief=relief_name, seed=seed, scale=SCALE,
        size=SIZE, samples=SAMPLES, z_min=z_min, z_max=z_max,
        heightmap=heightmap, mu=GROUNDS[OUTSIDE]['mu'], grounds=grounds, plants=plants, trees=orchard,
        surroundings=surroundings, features=features, start_points=starts,
        tufts=len(placements), cards=card_count, shares={name: round(float((lab == g).mean()), 3) for g, name in enumerate(GARDEN_GROUNDS)},
        max_slope=float(slope.max()), p99_slope=float(np.percentile(slope, 99)),
    )
    with open(info_path, 'w') as f:
        yaml.safe_dump(info, f, sort_keys=False)
    return info_path
