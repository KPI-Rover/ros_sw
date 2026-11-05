"""1x1 m square marked on the floor, driven counterclockwise from start point a, to check motion accuracy."""

from kpi_rover_worlds import sdf


def build(args):
    root, world = sdf.world('test_square')
    sdf.floor(world, args['floor'], 10, 10)
    link = sdf.static_model(world, 'square_marking')
    corners = [(0, 0), (1, 0), (1, 1), (0, 1)]
    for i, side in enumerate(('bottom', 'right', 'top', 'left')):
        sdf.marking_line(link, f'edge_{side}', *corners[i], *corners[(i + 1) % 4], args['marking'],
                         start='left', end='left')
    sdf.start_point(world, 'a', 0, 0, 0)
    return root
