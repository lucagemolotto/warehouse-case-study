#!/usr/bin/env python3
"""Draw the navigation map with traffic-control zones, stations, parking
spots and (optionally) tasks as arrows, to docs/zones.png.

    python3 tools/render_zones.py                    # inside the sim image (/ws, /docs mounted)
    python3 tools/render_zones.py P1:D2 P5:D3 P2:D1  # with tasks
"""
import math
import re
import sys

import yaml
from PIL import Image, ImageDraw, ImageFont

SHARE = './ros2_ws/src/warehouse_sim/'
OUT = './docs/zones.png'
S = 4  # pixels per map cell (5 cm)
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
ZONE_COLOURS = {'W': (60, 120, 220), 'C': (230, 140, 30), 'A': (40, 160, 90)}
TASK_COLOURS = [(200, 30, 30), (130, 40, 200), (20, 140, 160), (180, 120, 0)]


def zone_colour(name):
    """Hand-drawn zones by area (W/C/A); grid cells (rRcC) as a checkerboard."""
    g = re.fullmatch(r'r(\d+)c(\d+)', name)
    if g:
        return [(60, 120, 220), (230, 140, 30)][(int(g.group(1)) + int(g.group(2))) % 2]
    return ZONE_COLOURS.get(name[0], (120, 120, 120))


def main():
    m = yaml.safe_load(open(SHARE + 'maps/warehouse_nav.yaml'))
    base = Image.open(SHARE + 'maps/' + m['image']).convert('RGB')
    res, (ox, oy) = m['resolution'], m['origin'][:2]
    W, H = base.size
    img = base.resize((W * S, H * S), Image.NEAREST)
    overlay = Image.new('RGBA', img.size, (0, 0, 0, 0))
    od, dr = ImageDraw.Draw(overlay), ImageDraw.Draw(img)
    big, small = ImageFont.truetype(FONT, 34), ImageFont.truetype(FONT, 18)

    def px(x, y):
        return (x - ox) / res * S, (H - (y - oy) / res) * S

    zones = yaml.safe_load(open(SHARE + 'config/zones.yaml'))
    for i, z in enumerate(zones['zones']):
        x0, y0, x1, y1 = z['rect']
        c = zone_colour(z['name'])
        a, b = px(x0, y1), px(x1, y0)
        od.rectangle([a, b], fill=c + (45,), outline=c + (255,), width=4)
    img = Image.alpha_composite(img.convert('RGBA'), overlay).convert('RGB')
    dr = ImageDraw.Draw(img)
    for i, z in enumerate(zones['zones']):
        x0, y0, x1, y1 = z['rect']
        c = zone_colour(z['name'])
        a = px(x0, y1)
        dr.text((a[0] + 6, a[1] + 4), f"{z['name']} ({i})", fill=c, font=small)

    st = {s['name']: s for s in yaml.safe_load(open(SHARE + 'config/stations.yaml'))['stations']}
    for s in st.values():
        if s['kind'] == 'home':
            continue
        x, y = px(s['x'], s['y'])
        c = (0, 130, 0) if s['kind'] == 'pick' else (190, 0, 160)
        dr.ellipse([x - 12, y - 12, x + 12, y + 12], fill=c, outline=(255, 255, 255), width=2)
        dr.text((x + 16, y - 20), s['name'], fill=c, font=big)
    for r in yaml.safe_load(open(SHARE + 'config/robots.yaml'))['robots'][:3]:
        x, y = px(r['x'], r['y'])
        dr.rectangle([x - 12, y - 12, x + 12, y + 12], fill=(40, 40, 40))
        dr.text((x + 16, y - 14), r['name'], fill=(40, 40, 40), font=small)

    for k, task in enumerate(sys.argv[1:]):
        a, b = task.split(':')
        (xa, ya), (xb, yb) = px(st[a]['x'], st[a]['y']), px(st[b]['x'], st[b]['y'])
        c = TASK_COLOURS[k % len(TASK_COLOURS)]
        dr.line([(xa, ya), (xb, yb)], fill=c, width=6)
        ang = math.atan2(yb - ya, xb - xa)
        tip = [(xb - 22 * math.cos(ang - s), yb - 22 * math.sin(ang - s)) for s in (0.45, -0.45)]
        dr.polygon([(xb, yb)] + tip, fill=c)
        lx, ly = px(-3.3, -0.8 - 0.6 * k)   # legend, in the empty top of zone C2
        dr.line([(lx, ly + 20), (lx + 50, ly + 20)], fill=c, width=6)
        dr.text((lx + 62, ly), f'T{k + 1}: {a} -> {b}', fill=c, font=big)

    a, b = px(-7.3, 10.8), px(7.3, -10.6)
    img.crop((int(a[0]), int(a[1]), int(b[0]), int(b[1]))).save(OUT)
    print('wrote', OUT)


if __name__ == '__main__':
    main()
