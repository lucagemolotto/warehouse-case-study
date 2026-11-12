#!/usr/bin/env python3
"""Where does the robots' time go? Breakdown from the AbU-ROS logs.

Reads agents/logs/abu/app_robotN_*.log (debug level: one "applied update"
line per rule firing, with timestamp and action chain) and splits each
task a robot executed into:

  election   task broadcast (slot active)        -> robot takes it (Take)
  wait_from  standing still on the way to L_from: first route request ->
             first grant, plus each stop at a window end -> next grant
  drive_from the rest of the leg, up to arriving at L_from
  loading    arrived at L_from                   -> Loaded
  wait_to    route request to L_to               -> route granted
  drive_to   granted                             -> arrived at L_to
  unloading  arrived at L_to                     -> Unloaded (task done)

plus the trip home afterwards (wait_home, drive_home, not part of the task).

    python3 tools/timeline.py [agents/logs/abu]
"""
import glob
import json
import os
import re
import subprocess
import sys
from datetime import datetime

KEYS = ('this.t1_active=true', 'this.t2_active=true', 'this.t3_active=true', 'state="busy"',
        'req_open=true', 'route=req_mask', 'arrivals=arrivals+1', 'phase="to_to"',
        'finished=busy_task', 'phase="loading"', 'phase="unloading"', 'stopped=true')


def ts(s):
    return datetime.fromisoformat(s.replace('Z', '+00:00')[:26] + '+00:00').timestamp()


def events(path):
    # grep first: the logs are hundreds of MB
    pat = 'applied update resulting in modified values: map\\[[a-z]'
    out = subprocess.run(['grep', '-a', pat, path], capture_output=True, text=True).stdout
    for line in out.splitlines():
        try:
            d = json.loads(line)
        except ValueError:
            continue
        a = d.get('actionChain', '')
        if any(k in a for k in KEYS):
            yield ts(d['time']), a


def analyse(path):
    tasks, cur, dispatched = [], None, {}
    home = {}
    leg_t = stop_t = None   # start of the current leg; start of the current stop
    leg_wait = 0.0          # time standing still in the current leg
    for t, a in events(path):
        m = re.search(r'this\.t(\d)_active=true', a)
        if m:
            dispatched[m.group(1)] = t
            continue
        if 'state="busy"' in a and 'phase="to_from"' in a:
            slot = re.search(r'busy_task=(\d)', a).group(1)
            cur = {'slot': slot, 'election': t - dispatched.get(slot, t), 'leg': 'from'}
            continue
        leg = cur['leg'] if cur else 'home'
        acc = cur if cur else home
        if 'req_open=true' in a:
            if leg_t is None:       # first request of a leg: standing still from here
                leg_t = stop_t = t
                leg_wait = 0.0
        elif 'stopped=true' in a and leg_t is not None and stop_t is None:
            stop_t = t              # reached a window end without the next window
        elif 'route=req_mask' in a and stop_t is not None:
            leg_wait += t - stop_t
            stop_t = None
        elif 'arrivals=arrivals+1' in a and leg_t is not None:
            acc[f'wait_{leg}'] = acc.get(f'wait_{leg}', 0.0) + leg_wait
            acc[f'drive_{leg}'] = acc.get(f'drive_{leg}', 0.0) + (t - leg_t) - leg_wait
            if cur:
                cur['arrived'] = t
            leg_t = stop_t = None
        elif 'phase="to_to"' in a and cur:
            cur['loading'] = t - cur.get('arrived', t)
            cur['leg'] = 'to'
        elif 'finished=busy_task' in a and cur:
            cur['unloading'] = t - cur.get('arrived', t)
            tasks.append(cur)
            cur = None
    return tasks, home


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else 'agents/logs/abu'
    cols = ['election', 'wait_from', 'drive_from', 'loading', 'wait_to', 'drive_to', 'unloading']
    totals = {c: 0.0 for c in cols}
    print(f"{'robot':7s} {'slot':4s} " + ' '.join(f'{c:>10s}' for c in cols) + f" {'task total':>10s}")
    for path in sorted(glob.glob(os.path.join(d, 'app_robot*_*.log'))):
        robot = re.search(r'app_(robot\d)_', path).group(1)
        tasks, home = analyse(path)
        for tk in tasks:
            vals = [tk.get(c, 0.0) for c in cols]
            for c, v in zip(cols, vals):
                totals[c] += v
            print(f"{robot:7s} T{tk['slot']:3s} " + ' '.join(f'{v:10.1f}' for v in vals) + f' {sum(vals):10.1f}')
        print(f"{robot:7s} home: waiting {home.get('wait_home', 0.0):.1f} s, driving {home.get('drive_home', 0.0):.1f} s")
    s = sum(totals.values())
    print('\nshare of task time: ' + ', '.join(f'{c} {100 * v / s:.0f}%' for c, v in totals.items() if s))


if __name__ == '__main__':
    main()
