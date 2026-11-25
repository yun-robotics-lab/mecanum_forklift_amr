#!/usr/bin/env python3
"""Make the curated navigation map: SLAM map + objects the 2D lidar cannot see.

The lidar plane is at z = 0.224 m, so floor-level objects (mini pallets, boxes, cones, the low shelf
levels) never appear in the SLAM map but are real obstacles. This script paints their footprints,
read from the world file, as occupied cells into a copy of the SLAM map. AMCL keeps using the
unmodified SLAM map; only the Nav2 costmaps use the curated map.

usage: make_nav_map.py <slam_map.yaml> <world.sdf> <output_basename> [skip_name,...] [name:model:x:y:yaw,...]
The optional list names world objects (e.g. pallet_1,box_1) that are left out, for the map used after
that pallet has been picked up. The second optional list adds objects, e.g.
pallet_1_dropped:mini_pallet:2.7:-1.6:1.5708 (the map after a drop).
"""
import math
import re
import sys

# footprint (x, y) in metres in the model frame, for models that are below the lidar plane
FOOTPRINT = {
    'mini_pallet': (0.26, 0.20),
    'cargo_box': (0.15, 0.15),
    'safety_cone': (0.12, 0.12),
    'shelf': (0.58, 0.32),
    'pallet_rack': (0.65, 0.36),
}
# cargo_box placed on a shelf (z >= 0.2) is seen by the lidar and is skipped
MAX_PAINT_Z = 0.15


def read_pgm(path):
    data = open(path, 'rb').read()
    gap = rb'(?:\s|#[^\n]*\n)+'     # header fields may be separated by comments (GIMP)
    m = re.match(rb'P5' + gap + rb'(\d+)' + gap + rb'(\d+)' + gap + rb'(\d+)\s', data)
    w, h = int(m[1]), int(m[2])
    return w, h, bytearray(data[m.end():])


def objects(world, skip=()):
    text = open(world).read()
    for m in re.finditer(r'<include>(.*?)</include>', text, re.S):
        body = m.group(1)
        uri = re.search(r'model://(\w+)', body).group(1)
        pose = re.search(r'<pose>([^<]*)</pose>', body)
        name = re.search(r'<name>([^<]*)</name>', body)
        if uri in FOOTPRINT and pose and (name.group(1) if name else uri) not in skip:
            x, y, z, _, _, yaw = [float(v) for v in pose.group(1).split()]
            if z <= MAX_PAINT_Z or uri == 'shelf':
                yield (name.group(1) if name else uri), uri, x, y, yaw


def main():
    yaml_in, world, out = sys.argv[1:4]
    meta = open(yaml_in).read()
    res = float(re.search(r'resolution:\s*([\d.]+)', meta)[1])
    ox, oy = [float(v) for v in re.search(r'origin:\s*\[([^\]]*)\]', meta)[1].split(',')[:2]]
    pgm = re.search(r'image:\s*(\S+)', meta)[1]
    w, h, px = read_pgm(yaml_in.rsplit('/', 1)[0] + '/' + pgm)
    painted = 0
    skip = sys.argv[4].split(',') if len(sys.argv) > 4 else []
    extra = []
    for item in (sys.argv[5].split(',') if len(sys.argv) > 5 else []):
        n, uri, x, y, yw = item.split(':')
        extra.append((n, uri, float(x), float(y), float(yw)))
    for name, uri, cx, cy, yaw in list(objects(world, skip)) + extra:
        sx, sy = FOOTPRINT[uri]
        c, s = math.cos(yaw), math.sin(yaw)
        n0 = painted
        for j in range(h):
            for i in range(w):
                x = ox + (i + 0.5) * res
                y = oy + (h - 1 - j + 0.5) * res
                dx, dy = x - cx, y - cy
                lx, ly = c * dx + s * dy, -s * dx + c * dy
                if abs(lx) <= sx / 2 + res / 2 and abs(ly) <= sy / 2 + res / 2 and px[j * w + i] != 0:
                    px[j * w + i] = 0
                    painted += 1
        print(f'{name:22s} ({cx:+.2f}, {cy:+.2f}, yaw {yaw:+.2f}) -> {painted - n0} new cells')
    with open(out + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h) + bytes(px))
    name = out.rsplit('/', 1)[1]
    open(out + '.yaml', 'w').write(meta.replace(pgm, name + '.pgm'))
    print(f'painted {painted} cells -> {out}.pgm / .yaml')


if __name__ == '__main__':
    main()
