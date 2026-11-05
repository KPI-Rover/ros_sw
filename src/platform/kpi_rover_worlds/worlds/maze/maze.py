"""
Maze world: a warehouse with a maze of walls and arches, a fixed level or a random one.

A maze is a text grid of (2 * rows + 1) lines of (2 * cols + 1) characters.
Cells are at odd line and odd column indices, everything else is a wall
position between cells or a corner:
    '#'      - wall
    ' '      - open passage (in a wall position) or a cell
    'S', 'E' - entrance and exit, open passages in the outer wall
    letters  - openings from ARCHES, e.g. 'A' arch or 'L' too low arch

Example:
    #######
    S  A  #
    ### # #
    #   L E
    #######

Start points: 'a' outside the entrance and 'b' outside the exit, facing into
the maze; the shortest route between them is a blue line, GUI only. To print
a maze: python3 maze.py --maze random --seed 7 --difficulty hard

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
from collections import deque

from kpi_rover_worlds import scatter, sdf

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
ARCH_COLOR = (0.55, 0.42, 0.3)      # arch_material 'contrast'

# Openings by their letter in maze grids: clear width and height (None: a
# doorway without a lintel), m, and whether the robot (0.26 m wide, 0.345 m
# high, lidar at 0.245 m) fits through. T fits with about 3 cm to spare; the
# L lintel is above the lidar scan plane, so the opening looks free to the
# lidar, but the robot hits it.
ARCHES = {
    'D': dict(name='door', width=0.55, height=None, passable=True),
    'A': dict(name='arch', width=0.45, height=0.45, passable=True),
    'T': dict(name='tight_arch', width=0.32, height=0.38, passable=True),
    'N': dict(name='narrow_arch', width=0.20, height=0.45, passable=False),
    'L': dict(name='low_arch', width=0.45, height=0.30, passable=False),
}


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
    arches = ARCHES
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
    maze.check(ARCHES)
    return maze


def layout(maze):
    """Geometry of a maze centered at the origin."""
    arches = ARCHES
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


def make_maze(name, seed, difficulty):
    if name == 'random':
        return generate(int(seed), difficulty)
    return load_level(name)


def arch(parent, name, x, y, yaw, span, width, height, wall_height, thickness, view, material):
    """Posts leave a 'width' opening in a 'span' long wall, a lintel above 'height' (0: none) closes it."""
    look = dict(color=ARCH_COLOR) if material == 'contrast' else dict(wall_material=material)
    link = sdf.static_model(parent, name, (x, y, 0, 0, 0, yaw))
    post = (span - width) / 2
    if post > 0:
        for side, sign in (('left', -1), ('right', 1)):
            sdf.solid_box(link, f'post_{side}', sign * (width + post) / 2, 0, wall_height / 2, post, thickness,
                          wall_height, view=view, **look)
    if height > 0:
        sdf.solid_box(link, 'lintel', 0, 0, (height + wall_height) / 2, width, thickness, wall_height - height,
                      view=view, footprint_scale=0.3, **look)


def build(args):
    lay = layout(make_maze(args['maze'], args['seed'], args['difficulty']))
    view = args['wall_view']
    root, world = sdf.world('maze')
    sdf.floor(world, args['floor'], *lay['warehouse'])
    sdf.perimeter_walls(world, *lay['warehouse'], args['wall_material'])
    link = sdf.static_model(world, 'maze_walls')
    for i, (x, y, sx, sy) in enumerate(lay['walls']):
        sdf.wall(link, f'wall_{i}', x, y, 0 if sx >= sy else math.pi / 2, max(sx, sy), lay['wall_height'],
                 min(sx, sy), args['wall_material'], view)
    for i, a in enumerate(lay['arches']):
        arch(world, f"{a['type']}_{i}", a['x'], a['y'], a['yaw'], lay['span'], a['width'], a['height'],
             lay['wall_height'], lay['wall_thickness'], view, args['arch_material'])
    if args['route']:
        link = sdf.static_model(world, 'maze_route', (0, 0, 0.002, 0, 0, 0))    # above wall footprints
        for i, (x1, y1, x2, y2, start, end) in enumerate(lay['route']):
            sdf.marking_line(link, f'route_{i}', x1, y1, x2, y2, 'blue', width=0.03, start=start, end=end,
                             visibility=sdf.VISIBLE_GUI_ONLY)
    for i, (kind, x, y, yaw) in enumerate(scatter_objects(lay, args)):
        sdf.scatter_object(world, f'{kind}_{i}', kind, x, y, yaw)
    for name, (x, y, yaw) in lay['starts'].items():
        sdf.start_point(world, name, x, y, yaw)
    return root


def main():
    parser = argparse.ArgumentParser(description='Print a maze.')
    parser.add_argument('--maze', default='level1', help=f"fixed level ({', '.join(available_levels())}) or 'random'")
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--difficulty', default='medium', choices=list(DIFFICULTIES))
    args = parser.parse_args()
    print(make_maze(args.maze, args.seed, args.difficulty).text())


if __name__ == '__main__':
    main()
