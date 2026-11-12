#!/usr/bin/env python3
"""Build the occupancy map from the running simulation, with exact poses.

Teleports one robot over a grid of poses covering the warehouse (the other
robots are moved out of the building first), takes one lidar scan at each
pose and ray-casts it into an occupancy grid using the true pose. The map
therefore matches what the simulated lidar sees, clutter included.

Run in the Jazzy sim image next to a running sim; restart the sim after
(the robots' AMCL is meaningless once they have been teleported):

    python3 tools/map_from_sim.py --out /ws/src/warehouse_sim/maps/warehouse_sim
"""
import argparse
import math
import subprocess
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

RES = 0.05
X0, Y0, X1, Y1 = -8.0, -12.0, 8.0, 12.0   # grid bounds, map frame
BUILDING = (-6.8, -10.2, 6.8, 10.4)        # scan poses only inside the walls
MAX_FREE = 10.0                            # trust free space along a beam up to this range
LIDAR_X = -0.064                           # base_scan offset from base_footprint (TB3 Waffle)


def teleport(name, x, y, yaw=0.0):
    req = (f'name: "{name}", position: {{x: {x}, y: {y}, z: 0.05}}, '
           f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
    out = subprocess.run(
        ['gz', 'service', '-s', '/world/default/set_pose', '--reqtype', 'gz.msgs.Pose',
         '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
        capture_output=True, text=True)
    return 'true' in out.stdout


class Grab(Node):
    def __init__(self, robot):
        super().__init__('map_from_sim', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.scan = self.truth = None
        self.create_subscription(LaserScan, f'/{robot}/scan',
                                 lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, f'/{robot}/ground_truth',
                                 lambda m: setattr(self, 'truth', m), 10)

    def fresh(self, after_s, timeout=5.0):
        """Next scan and truth both stamped after sim time after_s."""
        t_end = time.time() + timeout
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
            s, t = self.scan, self.truth
            if s and t and stamp(s) > after_s + 0.3 and stamp(t) > after_s + 0.3:
                return s, t
        return None, None


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot', default='robot1')
    ap.add_argument('--park', nargs='*', default=['robot2', 'robot3'])
    ap.add_argument('--step', type=float, default=1.0)
    ap.add_argument('--out', default='/ws/src/warehouse_sim/maps/warehouse_sim')
    ap.add_argument('--limit', type=int, default=0, help='only the first N poses (debugging)')
    ap.add_argument('--verbose', action='store_true', help='say why each pose is skipped')
    a = ap.parse_args()

    W, H = int((X1 - X0) / RES), int((Y1 - Y0) / RES)
    hits = np.zeros((H, W), np.int32)
    free = np.zeros((H, W), np.int32)

    def cells(px, py):
        c = ((px - X0) / RES).astype(int)
        r = (H - 1 - (py - Y0) / RES).astype(int)
        ok = (c >= 0) & (c < W) & (r >= 0) & (r < H)
        return r[ok], c[ok]

    for i, name in enumerate(a.park):
        teleport(name, 30.0 + 2 * i, 30.0)

    rclpy.init()
    node = Grab(a.robot)
    bx0, by0, bx1, by1 = BUILDING
    poses = [(x, y) for x in np.arange(bx0 + 0.3, bx1, a.step)
             for y in np.arange(by0 + 0.3, by1, a.step)]
    if a.limit:
        poses = poses[:a.limit]
    skip = (lambda k, why: print(f'pose {k}: skipped, {why}', flush=True)) if a.verbose else (lambda k, why: None)
    used = 0
    for k, (x, y) in enumerate(poses):
        rclpy.spin_once(node, timeout_sec=0.05)
        now = stamp(node.truth) if node.truth else 0.0
        if not teleport(a.robot, round(x, 3), round(y, 3)):
            skip(k, 'teleport failed')
            continue
        s, t = node.fresh(now)
        if s is None:
            skip(k, f'no fresh scan+truth after sim t={now:.1f} '
                    f'(scan {stamp(node.scan) if node.scan else None}, truth {stamp(node.truth) if node.truth else None})')
            continue
        p = t.pose.pose
        # Skip poses where the robot ended up somewhere else (spawned inside an object).
        if math.hypot(p.position.x - x, p.position.y - y) > 0.05 or abs(p.position.z) > 0.05:
            skip(k, f'robot at ({p.position.x:.2f},{p.position.y:.2f},{p.position.z:.2f}), wanted ({x:.2f},{y:.2f})')
            continue
        yaw = math.atan2(2 * p.orientation.w * p.orientation.z, 1 - 2 * p.orientation.z ** 2)
        rng = np.asarray(s.ranges, float)
        ang = yaw + s.angle_min + np.arange(len(rng)) * s.angle_increment
        lx, ly = p.position.x + LIDAR_X * math.cos(yaw), p.position.y + LIDAR_X * math.sin(yaw)
        if np.mean(rng < 0.15) > 0.2:   # lidar inside an object
            skip(k, 'lidar inside an object')
            continue
        used += 1
        hit = np.isfinite(rng) & (rng > s.range_min) & (rng < s.range_max)
        hr, hc = cells(lx + rng[hit] * np.cos(ang[hit]), ly + rng[hit] * np.sin(ang[hit]))
        np.add.at(hits, (hr, hc), 1)
        reach = np.where(hit, rng - RES, MAX_FREE).clip(0, MAX_FREE)
        steps = np.arange(0, MAX_FREE, RES / 2)
        d = steps[None, :]
        m = d < reach[:, None]
        fx = (lx + d * np.cos(ang)[:, None])[m]
        fy = (ly + d * np.sin(ang)[:, None])[m]
        fr, fc = cells(fx, fy)
        lin = np.unique(fr * W + fc)
        free.flat[lin] += 1
        if k % 25 == 0:
            print(f'{k}/{len(poses)} poses, {used} used', flush=True)

    occ = (hits >= 2) & (hits * 4 >= free)
    img = np.full((H, W), 205, np.uint8)          # unknown
    img[(free > 0) & ~occ] = 254                    # free
    img[occ] = 0                                    # occupied
    Image.fromarray(img).save(a.out + '.png')
    with open(a.out + '.yaml', 'w') as f:
        f.write(f'# Built by tools/map_from_sim.py from the simulated world ({used} scans, '
                f'true poses).\nimage: {a.out.rsplit("/", 1)[-1]}.png\nmode: trinary\n'
                f'resolution: {RES}\norigin: [{X0}, {Y0}, 0.0]\nnegate: 0\n'
                f'occupied_thresh: 0.65\nfree_thresh: 0.196\n')
    print(f'done: {used} scans used of {len(poses)} poses -> {a.out}.png/.yaml')


if __name__ == '__main__':
    main()
