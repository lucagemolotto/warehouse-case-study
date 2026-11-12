#!/usr/bin/env python3
"""Drive robots around a station route and measure AMCL error vs Gazebo truth.

Run in the Jazzy sim image, next to a running sim (same ROS domain, and
GZ_IP/GZ_PARTITION as the sim, which the image sets):

    python3 tools/eval_amcl.py robot1=P1,D1,P4,D4,H robot2=P3,D2,P5,D3,H

Each robot gets its own route: robots sent to the same station at the same
time block each other (Nav2 has no multi-robot coordination). `home` in a
route is the robot's own spawn pose (config/robots.yaml).

Also reports each robot's clearance: the smallest distance from its true
position (base_footprint) to an occupied cell of the map. The TB3 Waffle's
body reaches ~0.24 m from base_footprint, so values below that mean the robot
touched something.

Ground truth: /robotN/ground_truth (Gazebo OdometryPublisher, map frame,
started by warehouse_multi.launch.py). Every amcl_pose message is compared
with the true pose nearest to its timestamp.
"""
import argparse
import bisect
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry

STATIONS = '/ws/src/warehouse_sim/config/stations.yaml'
ROBOTS = '/ws/src/warehouse_sim/config/robots.yaml'
MAP = '/ws/src/warehouse_sim/maps/warehouse_nav.yaml'


class Clearance:
    """Distance from a point to the nearest occupied map cell."""

    def __init__(self):
        m = yaml.safe_load(open(MAP))
        img = np.array(Image.open(MAP.rsplit('/', 1)[0] + '/' + m['image']).convert('L'))
        self.res, (self.ox, self.oy) = m['resolution'], m['origin'][:2]
        self.h, self.w = img.shape
        self.dist = distance_transform_edt(img >= 100) * self.res

    def __call__(self, x, y):
        c, r = int((x - self.ox) / self.res), self.h - 1 - int((y - self.oy) / self.res)
        if 0 <= c < self.w and 0 <= r < self.h:
            return float(self.dist[r, c])
        return float('nan')


def setattr_dict(d, k, v):
    d[k] = v


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Eval(Node):
    def __init__(self, robots):
        super().__init__('eval_amcl', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.robots = robots
        self.truth = {r: ([], []) for r in robots}  # times, (x, y, yaw)
        self.errors = {r: [] for r in robots}       # (t, dxy, dyaw, true_x, true_y, odom_dxy)
        self.odom = {r: None for r in robots}
        spawn = {x['name']: x for x in yaml.safe_load(open(ROBOTS))['robots']}
        self.spawn = {r: spawn[r] for r in robots}
        self.clearance = Clearance()
        self.min_clear = {r: (float('inf'), None) for r in robots}
        self.lock = threading.Lock()
        for r in robots:
            self.create_subscription(Odometry, f'/{r}/ground_truth',
                                     lambda m, r=r: self.on_truth(r, m), 50)
            self.create_subscription(Odometry, f'/{r}/odom',
                                     lambda m, r=r: setattr_dict(self.odom, r, m), 50)
            self.create_subscription(PoseWithCovarianceStamped, f'/{r}/amcl_pose',
                                     lambda m, r=r: self.on_amcl(r, m), 10)

    def on_truth(self, r, msg):
        p = msg.pose.pose
        with self.lock:
            ts, ps = self.truth[r]
            ts.append(stamp_s(msg.header))
            ps.append((p.position.x, p.position.y, yaw_of(p.orientation)))
            c = self.clearance(p.position.x, p.position.y)
            if c < self.min_clear[r][0]:
                self.min_clear[r] = (c, (round(p.position.x, 2), round(p.position.y, 2)))

    def on_amcl(self, r, msg):
        t = stamp_s(msg.header)
        with self.lock:
            ts, ps = self.truth[r]
            if not ts:
                return
            i = min(bisect.bisect_left(ts, t), len(ts) - 1)
            if i > 0 and abs(ts[i - 1] - t) < abs(ts[i] - t):
                i -= 1
            if abs(ts[i] - t) > 0.1:
                return
            x, y, yaw = ps[i]
        p = msg.pose.pose
        dyaw = math.atan2(math.sin(yaw_of(p.orientation) - yaw), math.cos(yaw_of(p.orientation) - yaw))
        # Wheel odometry in the map frame: odom starts at the spawn pose.
        od = float('nan')
        if self.odom[r] is not None:
            o, sp = self.odom[r].pose.pose.position, self.spawn[r]
            c, s_ = math.cos(sp['yaw']), math.sin(sp['yaw'])
            od = math.hypot(sp['x'] + c * o.x - s_ * o.y - x, sp['y'] + s_ * o.x + c * o.y - y)
        self.errors[r].append((t, math.hypot(p.position.x - x, p.position.y - y), abs(dyaw), x, y, od))


GOAL_TIMEOUT_S = 180.0


def drive(node, robot, route, stations, results):
    stations = dict(stations, home={'x': node.spawn[robot]['x'], 'y': node.spawn[robot]['y']})
    out = []
    results[robot] = out
    ac = node.action_clients[robot]
    if not ac.wait_for_server(timeout_sec=30.0):
        out.append(('-', 'NO NAV2 SERVER', 0.0))
        return
    for name in route:
        s = stations[name]
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y = float(s['x']), float(s['y'])
        g.pose.pose.orientation.w = 1.0
        t0 = time.time()
        gh = ac.send_goal_async(g)
        while not gh.done():
            time.sleep(0.1)
        res = gh.result().get_result_async()
        while not res.done() and time.time() - t0 < GOAL_TIMEOUT_S:
            time.sleep(0.1)
        if not res.done():
            gh.result().cancel_goal_async()
            out.append((name, 'TIMEOUT', time.time() - t0))
            continue
        out.append((name, res.result().status, time.time() - t0))


def pct(v, q):
    v = sorted(v)
    return v[min(int(q * len(v)), len(v) - 1)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('routes', nargs='*', default=[
        'robot1=P1,D1,P4,D4,H', 'robot2=P3,D2,P5,D3,H', 'robot3=P5,D3,P2,D2,H'])
    a = ap.parse_args()
    routes = dict((k, v.split(',')) for k, v in (r.split('=') for r in a.routes))
    a.robots = list(routes)
    stations = {s['name']: s for s in yaml.safe_load(open(STATIONS))['stations']}

    rclpy.init()
    node = Eval(a.robots)
    # Create the action clients here: creating them from the driving threads
    # while the executor spins races inside rclpy.
    node.action_clients = {r: ActionClient(node, NavigateToPose, f'/{r}/navigate_to_pose')
                           for r in a.robots}
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(3)

    results = {}
    threads = [threading.Thread(target=drive, args=(node, r, routes[r], stations, results))
               for r in a.robots]
    for th in threads:
        th.start()
    for th in threads:
        th.join()

    STATUS = {4: 'ok', 5: 'canceled', 6: 'ABORTED'}
    allxy = []
    for r in a.robots:
        legs = ' '.join(f"{n}:{STATUS.get(st, st)}({dt:.0f}s)" for n, st, dt in results[r])
        e = node.errors[r]
        if not e:
            print(f'{r}: no AMCL samples | {legs}')
            continue
        xy = [d for _, d, *_ in e]
        yw = [math.degrees(d) for _, _, d, *_ in e]
        allxy += xy
        od = [row[5] for row in e if row[5] == row[5]]
        odtxt = f' | odom err max {max(od):.3f} m' if od else ''
        mc, where = node.min_clear[r]
        odtxt += f' | min clearance {mc:.2f} m at {where}'
        print(f'{r}: {len(e)} samples | xy mean {sum(xy)/len(xy):.3f} p95 {pct(xy, .95):.3f} '
              f'max {max(xy):.3f} m | yaw mean {sum(yw)/len(yw):.1f} max {max(yw):.1f} deg{odtxt} | {legs}')
    if os.environ.get('EVAL_CSV'):
        with open(os.environ['EVAL_CSV'], 'w') as f:
            f.write('robot,t,err_xy,err_yaw,true_x,true_y,odom_err\n')
            for r in a.robots:
                for row in node.errors[r]:
                    f.write(r + ',' + ','.join(f'{v:.3f}' for v in row) + '\n')
    if allxy:
        print(f'ALL: xy mean {sum(allxy)/len(allxy):.3f} p95 {pct(allxy, .95):.3f} max {max(allxy):.3f} m')
    sys.stdout.flush()
    os._exit(0)  # skip rclpy teardown with the spinning executor thread (it aborts)


if __name__ == '__main__':
    main()
