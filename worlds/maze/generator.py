"""
Maze layouts for the maze world: fixed levels and a random maze generator.

A maze is a text grid of (2 * rows + 1) lines of (2 * cols + 1) characters.
Cells are at odd line and odd column indices, everything else is a wall
position between cells or a corner:
    '#'      - wall
    ' '      - open passage (in a wall position) or a cell
    'S', 'E' - entrance and exit, open passages in the outer wall
    letters  - openings from arches.yaml, e.g. 'A' arch or 'L' too low arch

Example:
    #######
    S  A  #
    ### # #
    #   L E
    #######

The generator turns a maze into a layout YAML with wall boxes, arches, start
points and the shortest route from the entrance to the exit for maze.sdf.xacro,
and scatters objects around the maze (see lib/scatter.py). The launch file calls prepare() before
processing the world; it can also be run directly to print or check a maze:
    python3 generator.py --maze random --seed 7 --difficulty hard
    python3 generator.py --maze level2 --layout /tmp/layout.yaml

Random mazes are perfect mazes (a spanning tree of passages) with a few extra
loops, so every cell is reachable. Passable arches replace passages on the
route from the entrance to the exit and impassable arches replace walls, so
the exit always stays reachable; the generator checks it anyway. Impassable
arches are put where they look like shortcuts. The same seed and difficulty
always give the same maze.
"""

import argparse
import math
import os
import random
import sys
import tempfile
from collections import deque

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lib'))
import scatter  # noqa: E402
import textured_box  # noqa: E402

INCLUDE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'include')

HERE = os.path.dirname(os.path.abspath(__file__))
LEVELS_DIR = os.path.join(HERE, 'levels')

CELL = 0.7              # cell size, m; corridors are CELL - WALL_THICKNESS wide
WALL_THICKNESS = 0.05
WALL_HEIGHT = 0.5       # not higher than two robot heights
MARGIN = 1.5            # free space between the maze and the warehouse walls
START_DISTANCE = 0.6    # distance of start points from the entrance and exit

# Random maze parameters for each difficulty: size in cells (cols, rows),
# number of extra loops, passable arches on the route and impassable arches.
DIFFICULTIES = {
    'easy': dict(size=(5, 4), loops=1, arches='DA', traps='N'),
    'medium': dict(size=(7, 5), loops=2, arches='DAT', traps='NLL'),
    'hard': dict(size=(10, 7), loops=3, arches='DAATT', traps='NNLLLL'),
}

DIRECTIONS = ((0, 1), (1, 0), (0, -1), (-1, 0))

# Free space kept around the maze and on the way from start points to the maze, m.
MAZE_MARGIN = 0.4
START_RADIUS = 0.6

# How the Gazebo GUI shows maze walls, see 'solid_box' in structures.xacro.
WALL_VIEWS = ('solid', 'transparent', 'hidden', 'marking')


def load_arches():
    with open(os.path.join(HERE, 'arches.yaml')) as f:
        return yaml.safe_load(f)


def available_levels():
    return sorted(f[:-4] for f in os.listdir(LEVELS_DIR) if f.endswith('.txt'))


class Maze:
    def __init__(self, grid):
        self.grid = [list(line) for line in grid]
        self.rows = (len(self.grid) - 1) // 2
        self.cols = (len(self.grid[0]) - 1) // 2

    @classmethod
    def parse(cls, text):
        lines = [line.rstrip('\n') for line in text.splitlines() if line.strip()]
        width = max(len(line) for line in lines)
        lines = [line.ljust(width) for line in lines]
        if len(lines) % 2 == 0 or width % 2 == 0:
            raise ValueError('Maze grid must have an odd number of lines and columns.')
        return cls(lines)

    def text(self):
        return '\n'.join(''.join(line) for line in self.grid)

    def cells(self):
        return [(r, c) for r in range(1, 2 * self.rows, 2) for c in range(1, 2 * self.cols, 2)]

    def find(self, char):
        found = [(r, c) for r, line in enumerate(self.grid) for c, ch in enumerate(line) if ch == char]
        if len(found) != 1:
            raise ValueError(f"Maze must have exactly one '{char}', found {len(found)}.")
        return found[0]

    def inner_cell(self, position):
        """Cell next to an entrance or exit in the outer wall."""
        r, c = position
        return (min(max(r, 1), 2 * self.rows - 1), min(max(c, 1), 2 * self.cols - 1))

    def is_boundary(self, r, c):
        return r in (0, 2 * self.rows) or c in (0, 2 * self.cols)

    def wall_positions(self, interior=True):
        """Positions between two cells (not corners), optionally only inside the outer wall."""
        return [(r, c) for r in range(len(self.grid)) for c in range(len(self.grid[0]))
                if (r + c) % 2 == 1 and not (interior and self.is_boundary(r, c))]

    def wall_cells(self, r, c):
        """The two cells separated by a wall position."""
        if r % 2 == 0:
            return (r - 1, c), (r + 1, c)
        return (r, c - 1), (r, c + 1)

    def distances(self, start, arches):
        """BFS distances and parents from a cell, through open passages and passable arches."""
        dist, parent = {start: 0}, {start: None}
        queue = deque([start])
        while queue:
            r, c = queue.popleft()
            for dr, dc in DIRECTIONS:
                wr, wc, nr, nc = r + dr, c + dc, r + 2 * dr, c + 2 * dc
                if not (0 < nr < 2 * self.rows and 0 < nc < 2 * self.cols) or (nr, nc) in dist:
                    continue
                ch = self.grid[wr][wc]
                if ch == ' ' or (ch in arches and arches[ch]['passable']):
                    dist[(nr, nc)] = dist[(r, c)] + 1
                    parent[(nr, nc)] = (wr, wc), (r, c)
                    queue.append((nr, nc))
        return dist, parent

    def check(self, arches):
        """Check the maze characters and that the exit is reachable from the entrance."""
        allowed = set('# SE') | set(arches)
        for r, line in enumerate(self.grid):
            for c, ch in enumerate(line):
                if ch not in allowed:
                    raise ValueError(f"Unknown maze character '{ch}' at line {r + 1}, column {c + 1}.")
                if r % 2 == 1 and c % 2 == 1 and ch != ' ':
                    raise ValueError(f"Cell at line {r + 1}, column {c + 1} must be ' '.")
                if r % 2 == 0 and c % 2 == 0 and ch not in '# ':
                    raise ValueError(f"Corner at line {r + 1}, column {c + 1} must be '#' or ' '.")
        entrance, exit_ = self.find('S'), self.find('E')
        for name, position in (('S', entrance), ('E', exit_)):
            if not self.is_boundary(*position) or sum(position) % 2 == 0:
                raise ValueError(f"'{name}' must be in the outer wall between corners.")
        dist, _ = self.distances(self.inner_cell(entrance), arches)
        if self.inner_cell(exit_) not in dist:
            raise ValueError('Exit is not reachable from the entrance.')


def generate(seed, difficulty):
    """Random maze for a seed and difficulty from DIFFICULTIES."""
    if difficulty not in DIFFICULTIES:
        raise ValueError(f"Unknown difficulty '{difficulty}'. Available: {', '.join(DIFFICULTIES)}.")
    cfg = DIFFICULTIES[difficulty]
    arches = load_arches()
    rng = random.Random(f'{seed}:{difficulty}')
    cols, rows = cfg['size']
    maze = Maze([['#'] * (2 * cols + 1) for _ in range(2 * rows + 1)])
    for r, c in maze.cells():
        maze.grid[r][c] = ' '

    # Perfect maze with an iterative recursive backtracker.
    start = rng.choice(maze.cells())
    visited, stack = {start}, [start]
    while stack:
        r, c = stack[-1]
        options = [(dr, dc) for dr, dc in DIRECTIONS
                   if 0 < r + 2 * dr < 2 * rows and 0 < c + 2 * dc < 2 * cols and (r + 2 * dr, c + 2 * dc) not in visited]
        if not options:
            stack.pop()
            continue
        dr, dc = rng.choice(options)
        maze.grid[r + dr][c + dc] = ' '
        visited.add((r + 2 * dr, c + 2 * dc))
        stack.append((r + 2 * dr, c + 2 * dc))

    # Entrance on the left side, exit on the right side.
    entrance = (2 * rng.randrange(rows) + 1, 0)
    exit_ = (2 * rng.randrange(rows) + 1, 2 * cols)
    maze.grid[entrance[0]][entrance[1]] = 'S'
    maze.grid[exit_[0]][exit_[1]] = 'E'

    # Extra loops give alternative routes.
    walls = [p for p in maze.wall_positions() if maze.grid[p[0]][p[1]] == '#']
    for r, c in rng.sample(walls, min(cfg['loops'], len(walls))):
        maze.grid[r][c] = ' '

    # Passable arches spread along the route from the entrance to the exit.
    start_cell, end_cell = maze.inner_cell(entrance), maze.inner_cell(exit_)
    dist, parent = maze.distances(start_cell, arches)
    route, cell = [], end_cell
    while parent[cell]:
        wall, cell = parent[cell]
        route.append(wall)
    route.reverse()
    letters = list(cfg['arches'])
    rng.shuffle(letters)
    for i, letter in enumerate(letters[:len(route)]):
        r, c = route[(2 * i + 1) * len(route) // (2 * len(letters))]
        maze.grid[r][c] = letter

    # Impassable arches in walls that look like shortcuts: between cells far apart along the passages.
    dist, _ = maze.distances(start_cell, arches)
    walls = [p for p in maze.wall_positions() if maze.grid[p[0]][p[1]] == '#']
    rng.shuffle(walls)
    walls.sort(key=lambda p: -abs(dist[maze.wall_cells(*p)[0]] - dist[maze.wall_cells(*p)[1]]))
    for (r, c), letter in zip(walls, cfg['traps']):
        maze.grid[r][c] = letter

    maze.check(arches)
    return maze


def load_level(name):
    path = os.path.join(LEVELS_DIR, name + '.txt')
    if not os.path.isfile(path):
        raise ValueError(f"Unknown maze '{name}'. Available: {', '.join(available_levels())}, random.")
    with open(path) as f:
        maze = Maze.parse(f.read())
    maze.check(load_arches())
    return maze


def layout(maze):
    """Geometry of a maze centered at the origin for maze.sdf.xacro."""
    arches = load_arches()
    t = WALL_THICKNESS

    def x(c):
        return round((c / 2 - maze.cols / 2) * CELL, 4)

    def y(r):
        return round((maze.rows / 2 - r / 2) * CELL, 4)

    def solid(r, c):
        return maze.grid[r][c] == '#'

    walls, covered = [], set()
    # Horizontal wall runs along even lines and vertical ones along even columns, each from corner to corner.
    for r in range(0, 2 * maze.rows + 1, 2):
        c = 1
        while c < 2 * maze.cols:
            if solid(r, c):
                end = c
                while end + 2 < 2 * maze.cols and solid(r, end + 2):
                    end += 2
                walls.append([round((x(c - 1) + x(end + 1)) / 2, 4), y(r), round(x(end + 1) - x(c - 1) + t, 4), t])
                covered.update((r, cc) for cc in range(c - 1, end + 2, 2))
                c = end + 2
            else:
                c += 2
    # Vertical runs stop at corners already covered by horizontal runs, so walls do not overlap.
    horizontal = set(covered)
    for c in range(0, 2 * maze.cols + 1, 2):
        r = 1
        while r < 2 * maze.rows:
            if solid(r, c):
                end = r
                while end + 2 < 2 * maze.rows and solid(end + 2, c):
                    end += 2
                top = y(r - 1) + (-t / 2 if (r - 1, c) in horizontal else t / 2)
                bottom = y(end + 1) - (-t / 2 if (end + 1, c) in horizontal else t / 2)
                walls.append([x(c), round((top + bottom) / 2, 4), t, round(top - bottom, 4)])
                covered.update((rr, c) for rr in range(r - 1, end + 2, 2))
                r = end + 2
            else:
                r += 2

    # Arches, and posts at corners next to arches that no wall run covers.
    openings = []
    for r, c in maze.wall_positions(interior=False):
        ch = maze.grid[r][c]
        if ch not in arches:
            continue
        arch = arches[ch]
        openings.append(dict(type=arch['name'], x=x(c), y=y(r), yaw=0.0 if r % 2 == 0 else round(math.pi / 2, 4),
                             width=arch['width'], height=arch['height'] or 0.0))
        corners = [(r, c - 1), (r, c + 1)] if r % 2 == 0 else [(r - 1, c), (r + 1, c)]
        for corner in corners:
            if corner not in covered:
                covered.add(corner)
                walls.append([x(corner[1]), y(corner[0]), t, t])

    # Start points outside the entrance and exit, facing into the maze.
    starts = {}
    for name, char in (('a', 'S'), ('b', 'E')):
        r, c = maze.find(char)
        nx = -1 if c == 0 else 1 if c == 2 * maze.cols else 0
        ny = 1 if r == 0 else -1 if r == 2 * maze.rows else 0
        starts[name] = [round(x(c) + nx * START_DISTANCE, 4), round(y(r) + ny * START_DISTANCE, 4),
                        round(math.atan2(-ny, -nx), 4)]

    # Shortest route through passable openings, from start point 'a' through the cell centers to start point 'b'.
    start_cell, end_cell = maze.inner_cell(maze.find('S')), maze.inner_cell(maze.find('E'))
    _, parent = maze.distances(start_cell, arches)
    cells = [end_cell]
    while parent[cells[-1]]:
        cells.append(parent[cells[-1]][1])
    points = [starts['a'][:2]] + [[x(c), y(r)] for r, c in reversed(cells)] + [starts['b'][:2]]
    corners = [points[0]]
    for point, following in zip(points[1:-1], points[2:]):
        previous = corners[-1]
        if abs((point[0] - previous[0]) * (following[1] - point[1]) - (point[1] - previous[1]) * (following[0] - point[0])) > 1e-9:
            corners.append(point)
    corners.append(points[-1])

    def turn(a, b, c):
        return 'left' if (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]) > 0 else 'right'

    route = []
    for i in range(len(corners) - 1):
        start = turn(corners[i - 1], corners[i], corners[i + 1]) if i > 0 else 'flat'
        end = turn(corners[i], corners[i + 1], corners[i + 2]) if i + 2 < len(corners) else 'flat'
        route.append(corners[i] + corners[i + 1] + [start, end])

    size = [round(maze.cols * CELL, 4), round(maze.rows * CELL, 4)]
    return dict(
        cell=CELL, wall_thickness=t, wall_height=WALL_HEIGHT, span=round(CELL - t, 4),
        size=size, warehouse=[size[0] + 2 * MARGIN, size[1] + 2 * MARGIN],
        walls=walls, arches=openings, starts=starts, route=route, grid=maze.text().splitlines())


def scatter_objects(lay, args):
    """Scatter objects around the maze, off it and the ways from the start points to it."""
    sx, sy = lay['size'][0] / 2 + WALL_THICKNESS, lay['size'][1] / 2 + WALL_THICKNESS
    wx, wy = lay['warehouse'][0] / 2, lay['warehouse'][1] / 2
    keep_out = [('rect', -sx - MAZE_MARGIN, sx + MAZE_MARGIN, -sy - MAZE_MARGIN, sy + MAZE_MARGIN)]
    for x, y, yaw in lay['starts'].values():
        keep_out.append(('circle', x, y, START_RADIUS))
        keep_out.append(('segment', x, y, x + START_DISTANCE * math.cos(yaw), y + START_DISTANCE * math.sin(yaw), START_RADIUS))
    return scatter.scatter_from_args(args, (-wx, wx, -wy, wy), keep_out, [('rect', -sx, sx, -sy, sy)], 'maze')


def wall_meshes(lay, wall_material, arch_material):
    """Textured box meshes of the maze walls, the perimeter walls and the arch parts (see lib/textured_box.py)."""
    with open(os.path.join(INCLUDE_DIR, 'walls.yaml')) as f:
        walls_cfg = yaml.safe_load(f)
    t, h = lay['wall_thickness'], lay['wall_height']
    walls = {(max(sx, sy), min(sx, sy), h, 0.0) for _, _, sx, sy in lay['walls']}
    walls.update(textured_box.perimeter_boxes(*lay['warehouse'], height=h))
    meshes = textured_box.wall_meshes(sorted(walls), wall_material, walls_cfg)
    if arch_material != 'contrast':
        parts = set()
        for arch in lay['arches']:
            post = (lay['span'] - arch['width']) / 2
            if post > 0:
                parts.add((post, t, h, 0.0))
            if arch['height'] > 0:
                parts.add((arch['width'], t, h - arch['height'], arch['height']))
        meshes.update(textured_box.wall_meshes(sorted(parts), arch_material, walls_cfg))
    return meshes


def make_maze(name, seed, difficulty):
    if name == 'random':
        return generate(int(seed), difficulty)
    return load_level(name)


def prepare(args):
    """
    Launch hook: build the maze selected by the world arguments ('maze',
    'seed', 'difficulty') and return the xacro mappings with 'layout' set to
    a temporary layout file.
    """
    if args.get('wall_view') not in WALL_VIEWS:
        raise ValueError(f"Unknown wall_view '{args.get('wall_view')}'. Available: {', '.join(WALL_VIEWS)}.")
    if args.get('layout'):
        return args
    maze = make_maze(args['maze'], args['seed'], args['difficulty'])
    lay = layout(maze)
    lay['objects'] = scatter_objects(lay, args)
    lay['meshes'] = wall_meshes(lay, args.get('wall_material', 'plain'), args.get('arch_material', 'contrast'))
    with tempfile.NamedTemporaryFile('w', prefix='kpi_rover_maze_', suffix='.yaml', delete=False) as f:
        yaml.safe_dump(lay, f, default_flow_style=None, sort_keys=False)
    return dict(args, layout=f.name)


def main():
    parser = argparse.ArgumentParser(description='Print a maze and optionally write its layout for maze.sdf.xacro.')
    parser.add_argument('--maze', default='level1', help=f"fixed level ({', '.join(available_levels())}) or 'random'")
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--difficulty', default='medium', choices=list(DIFFICULTIES))
    parser.add_argument('--objects', type=int, default=0, help='number of objects scattered around the maze')
    parser.add_argument('--objects-seed', default='', help='object placement seed; empty for a new random one')
    parser.add_argument('--layout', help='write the layout YAML to this file')
    args = parser.parse_args()

    maze = make_maze(args.maze, args.seed, args.difficulty)
    print(maze.text())
    lay = layout(maze)
    if args.objects:
        lay['objects'] = scatter_objects(lay, vars(args))
    if args.layout:
        with open(args.layout, 'w') as f:
            yaml.safe_dump(lay, f, default_flow_style=None, sort_keys=False)


if __name__ == '__main__':
    main()
