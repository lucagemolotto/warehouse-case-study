#!/usr/bin/env python3
"""Compute which traffic-control zones connect, and write them into zones.yaml.

Two zones are connected when their rectangles share a boundary segment
along which there is a free run of at least MIN_GAP metres on the map
(checked on both sides of the boundary). Also reports which stations fall
in which zone, and points of free space not covered by any zone.

    python3 tools/zones_edges.py            # inside the sim image (/ws mounted)
"""
import sys

import numpy as np
import yaml
from PIL import Image

SHARE = '/ws/src/warehouse_sim/'
ZONES = SHARE + 'config/zones.yaml'
MIN_GAP = 0.7    # metres of free boundary for a passage (TB3 Waffle ~0.31 m wide)
PROBE = 0.15     # how far each side of the boundary must be free

m = yaml.safe_load(open(SHARE + 'maps/warehouse_sim.yaml'))
img = np.array(Image.open(SHARE + 'maps/' + m['image']).convert('L'))
H, W = img.shape
res = m['resolution']
ox, oy = m['origin'][:2]
free = img > 250


def is_free(x, y):
    c, r = int((x - ox) / res), H - 1 - int((y - oy) / res)
    return 0 <= c < W and 0 <= r < H and free[r, c]


def longest_free_run(points, normal):
    best = run = 0.0
    nx, ny = normal
    for x, y in points:
        if all(is_free(x + s * nx * PROBE, y + s * ny * PROBE) for s in (-1, 0, 1)):
            run += res
            best = max(best, run)
        else:
            run = 0.0
    return best


def shared_boundary(a, b):
    """Points along the shared edge of rects a and b, and the edge normal."""
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    for x in (ax0, ax1):
        if x in (bx0, bx1) and x != (bx0 if x == ax0 else bx1):
            lo, hi = max(ay0, by0), min(ay1, by1)
            if hi > lo:
                return [(x, y) for y in np.arange(lo, hi, res)], (1, 0)
    for y in (ay0, ay1):
        if y in (by0, by1) and y != (by0 if y == ay0 else by1):
            lo, hi = max(ax0, bx0), min(ax1, bx1)
            if hi > lo:
                return [(x, y) for x in np.arange(lo, hi, res)], (0, 1)
    return None, None


def main():
    doc = yaml.safe_load(open(ZONES))
    zones = doc['zones']
    edges = []
    for i, a in enumerate(zones):
        for b in zones[i + 1:]:
            pts, normal = shared_boundary(a['rect'], b['rect'])
            if pts is None:
                continue
            gap = longest_free_run(pts, normal)
            ok = gap >= MIN_GAP
            print(f"{a['name']}-{b['name']}: longest free passage {gap:4.2f} m {'' if ok else '-> NOT connected'}")
            if ok:
                edges.append([a['name'], b['name']])

    def zone_of(x, y):
        for z in zones:
            x0, y0, x1, y1 = z['rect']
            if x0 <= x < x1 and y0 <= y < y1:
                return z['name']
        return None

    for s in yaml.safe_load(open(SHARE + 'config/stations.yaml'))['stations']:
        print(f"station {s['name']} in zone {zone_of(s['x'], s['y'])}")
    ys, xs = np.nonzero(free)
    uncovered = [(ox + c * res, oy + (H - 1 - r) * res) for r, c in zip(ys[::50], xs[::50])
                 if zone_of(ox + c * res, oy + (H - 1 - r) * res) is None]
    print(f'free cells not in any zone (sampled): {len(uncovered)}',
          '' if not uncovered else f'e.g. {[(round(x, 1), round(y, 1)) for x, y in uncovered[:6]]}')

    # Rewrite only the edges line, keeping the hand-written part of the file.
    text = open(ZONES).read()
    head = text[:text.index('\nedges:')]
    with open(ZONES, 'w') as f:
        f.write(head + '\nedges:\n' + ''.join(f'  - [{a}, {b}]\n' for a, b in edges))
    print(f'{len(edges)} edges written to {ZONES}')


if __name__ == '__main__':
    sys.exit(main())
