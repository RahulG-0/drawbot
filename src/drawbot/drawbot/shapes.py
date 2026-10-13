"""Shape generators. Each returns a list of strokes; a stroke is a list of
(x, y) points in metres, centred on the origin, fitting a box of side `size`."""

import math


def _polygon(points):
    return [points + [points[0]]]


def square(size):
    h = size / 2
    return _polygon([(-h, -h), (h, -h), (h, h), (-h, h)])


def triangle(size):
    r = size / 2
    return _polygon([(r * math.cos(a), r * math.sin(a))
                     for a in (0.0, 2 * math.pi / 3, 4 * math.pi / 3)])


def circle(size, n=90):
    r = size / 2
    return [[(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n))
             for i in range(n + 1)]]


def star(size):
    r = size / 2
    return _polygon([(r * math.cos(4 * math.pi * i / 5), r * math.sin(4 * math.pi * i / 5))
                     for i in range(5)])


def heart(size, n=120):
    s = size / 34.0
    pts = []
    for i in range(n + 1):
        t = 2 * math.pi * i / n
        along = 16 * math.sin(t) ** 3
        up = 13 * math.cos(t) - 5 * math.cos(2 * t) - 2 * math.cos(3 * t) - math.cos(4 * t)
        pts.append(((up + 2.5) * s, -along * s))   # the heart's "up" is +x, away from the base
    return [pts]


def spiral(size, turns=4, n=240):
    r = size / 2
    return [[(r * i / n * math.cos(2 * math.pi * turns * i / n),
              r * i / n * math.sin(2 * math.pi * turns * i / n)) for i in range(n + 1)]]


def house(size):
    h = size / 2
    walls = [(-h, -h), (-h, h), (h * 0.2, h), (h * 0.2, -h), (-h, -h)]
    roof = [(h * 0.2, -h), (h, 0.0), (h * 0.2, h)]
    door = [(-h, -h * 0.25), (-h * 0.2, -h * 0.25), (-h * 0.2, h * 0.25), (-h, h * 0.25)]
    return [walls, roof, door]


SHAPES = {
    'square': square,
    'triangle': triangle,
    'circle': circle,
    'star': star,
    'heart': heart,
    'spiral': spiral,
    'house': house,
}


def place(strokes, cx, cy):
    return [[(x + cx, y + cy) for x, y in stroke] for stroke in strokes]


def demo():
    """A sheet-filling sampler: (shape, size, centre x, centre y)."""
    layout = [
        ('house', 0.050, 0.200, 0.060),
        ('star', 0.055, 0.200, -0.055),
        ('heart', 0.045, 0.128, 0.060),
        ('spiral', 0.050, 0.128, -0.055),
        ('circle', 0.030, 0.164, 0.002),
    ]
    strokes = []
    for name, size, cx, cy in layout:
        strokes += place(SHAPES[name](size), cx, cy)
    return strokes
