#!/usr/bin/env python3
"""Find obstacles the robots can hit but the lidar map does not show.

The simulated lidar renders visual geometry, while the physics uses collision
geometry; where a model's two differ, robots plan through something solid.
This slices every model's collision geometry (meshes and boxes) at robot
heights and reports collision outlines that lie far from any occupied cell
of the map.

    python3 tools/collision_vs_map.py            # report only (inside the sim image)
    python3 tools/collision_vs_map.py --write    # also write maps/warehouse_nav.*

--write saves the navigation map: the lidar map (maps/warehouse_sim, built by
tools/map_from_sim.py) plus every collision outline the lidar does not see,
as occupied cells. Nav2 plans on it, so robots go round obstacles that exist
only in the physics (e.g. the solid side panels of the west-wall shelf,
whose visual is an open frame).
"""
import argparse
import math
import re

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt

SHARE = '/ws/src/warehouse_sim/'
HEIGHTS = (0.05, 0.10, 0.15, 0.19)   # robot body heights above the floor (m)
FLOOR = 0.034                        # floor surface (GroundB box top)
TOL = 0.15                           # an outline point this close to the map is "seen"
SKIP = ('GroundB_01',)               # the floor itself


def mesh_triangles(path, scale, cpose):
    d = open(path).read()
    unit = float(re.search(r'meter="([^"]+)"', d).group(1))
    tris = []
    for g in re.finditer(r'<geometry id="[^"]+".*?</geometry>', d, re.S):
        g = g.group(0)
        pos = re.search(r'<float_array[^>]*POSITION[^>]*>([^<]+)<', g, re.I)
        tri = re.search(r'<triangles[^>]*>(.*?)</triangles>', g, re.S)
        if not pos or not tri:
            continue
        v = [float(x) * unit for x in pos.group(1).split()]
        pts = [(v[i] * scale[0] + cpose[0], v[i + 1] * scale[1] + cpose[1], v[i + 2] * scale[2] + cpose[2])
               for i in range(0, len(v), 3)]
        stride = len(re.findall(r'<input[^>]*offset', tri.group(1)))
        idx = [int(x) for x in re.search(r'<p>([^<]+)</p>', tri.group(1)).group(1).split()][::stride]
        tris += [(pts[idx[i]], pts[idx[i + 1]], pts[idx[i + 2]]) for i in range(0, len(idx) - 2, 3)]
    return tris


def box_triangles(size, cpose):
    sx, sy, sz = size
    c = [(cpose[0] + a * sx / 2, cpose[1] + b * sy / 2, cpose[2] + e * sz / 2)
         for a in (-1, 1) for b in (-1, 1) for e in (-1, 1)]
    faces = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    return [t for f in faces for t in ((c[f[0]], c[f[1]], c[f[2]]), (c[f[0]], c[f[2]], c[f[3]]))]


def slice_tri(t, z):
    """Segment where triangle t crosses the plane at height z."""
    pts = []
    for a, b in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])):
        if (a[2] - z) * (b[2] - z) < 0:
            k = (z - a[2]) / (b[2] - a[2])
            pts.append((a[0] + k * (b[0] - a[0]), a[1] + k * (b[1] - a[1])))
    return pts if len(pts) == 2 else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--write', action='store_true', help='write maps/warehouse_nav.png/.yaml')
    a = ap.parse_args()
    m = yaml.safe_load(open(SHARE + 'maps/warehouse_sim.yaml'))
    img = np.array(Image.open(SHARE + 'maps/' + m['image']).convert('L'))
    H, W = img.shape
    res, (ox, oy) = m['resolution'], m['origin'][:2]
    dist = distance_transform_edt(img >= 100) * res

    def cell(x, y):
        return H - 1 - int((y - oy) / res), int((x - ox) / res)

    def seen(x, y):
        r, c = cell(x, y)
        return not (0 <= c < W and 0 <= r < H) or dist[r, c] <= TOL

    added = np.zeros_like(img, dtype=bool)

    world = re.sub(r'<!--.*?-->', '', open(SHARE + 'worlds/warehouse.sdf').read(), flags=re.S)
    for inc in re.finditer(r"<model name=['\"]([^'\"]+)['\"]>\s*<include>\s*<uri>model://(\w+)</uri>\s*</include>\s*"
                           r"<pose[^>]*>([^<]+)</pose>", world):
        inst, model, pose = inc.group(1), inc.group(2), [float(v) for v in inc.group(3).split()]
        if model.endswith(SKIP):
            continue
        px, py, pz, yaw = pose[0], pose[1], pose[2], pose[5]
        sdf = open(f'{SHARE}models/{model}/model.sdf').read()
        tris = []
        for c in re.finditer(r'<collision[^>]*>(.*?)</collision>', sdf, re.S):
            cb = c.group(1)
            cp = [float(v) for v in (re.search(r'<pose>([^<]+)</pose>', cb).group(1).split()
                                     if re.search(r'<pose>', cb) else [0] * 6)]
            sc = [float(v) for v in (re.search(r'<scale>([^<]+)</scale>', cb).group(1).split()
                                     if re.search(r'<scale>', cb) else [1, 1, 1])]
            uri = re.search(r'<uri>model://[^/]+/([^<]+)</uri>', cb)
            bx = re.search(r'<box>\s*<size>([^<]+)</size>', cb)
            if uri:
                tris += mesh_triangles(f'{SHARE}models/{model}/{uri.group(1)}', sc, cp)
            elif bx:
                tris += box_triangles([float(v) for v in bx.group(1).split()], cp)
        unseen = []
        for z in HEIGHTS:
            zl = FLOOR + z - pz
            for t in tris:
                seg = slice_tri(t, zl)
                if not seg:
                    continue
                (x0, y0), (x1, y1) = seg
                n = max(2, int(math.dist((x0, y0), (x1, y1)) / res))
                for k in range(n + 1):
                    lx, ly = x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n
                    wx = px + lx * math.cos(yaw) - ly * math.sin(yaw)
                    wy = py + lx * math.sin(yaw) + ly * math.cos(yaw)
                    if not seen(wx, wy):
                        unseen.append((round(wx, 1), round(wy, 1)))
                        added[cell(wx, wy)] = True
        if unseen:
            pts = sorted(set(unseen))
            xs, ys = [p[0] for p in pts], [p[1] for p in pts]
            print(f'{inst}: {len(pts)} outline points not on the map, '
                  f'x[{min(xs)}, {max(xs)}] y[{min(ys)}, {max(ys)}], e.g. {pts[:4]}')
    if a.write:
        from scipy.ndimage import binary_dilation
        nav = img.copy()
        nav[binary_dilation(added, iterations=1)] = 0   # 2-3 cells thick: no gaps at 45 degrees
        Image.fromarray(nav).save(SHARE + 'maps/warehouse_nav.png')
        with open(SHARE + 'maps/warehouse_nav.yaml', 'w') as f:
            f.write('# Navigation map: maps/warehouse_sim (lidar) plus collision outlines the lidar\n'
                    '# does not see, written by tools/collision_vs_map.py --write.\n'
                    f"image: warehouse_nav.png\nmode: trinary\nresolution: {res}\n"
                    f"origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
        print(f'wrote maps/warehouse_nav.png/.yaml ({int(added.sum())} cells added)')
    print('done')


if __name__ == '__main__':
    main()
