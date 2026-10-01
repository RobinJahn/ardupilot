#!/usr/bin/env python3
# Infer rate-thread iterations per main loop from mix decrements inside slew-limited ramp-downs.
# per-iteration decrement = 0.5*dt_step, dt_step = main-loop dt (baseline) or 1/999 (fixed).
import sys
from collections import Counter
from pymavlink import mavutil
path, mode = sys.argv[1], sys.argv[2]   # mode: base|fix
m = mavutil.mavlink_connection(path)
rows = []
while True:
    x = m.recv_match(type='DMIX')
    if x is None: break
    rows.append((x.TimeUS, x.Mix, x.Des, x.dt))
c = Counter()
for k in range(1, len(rows)-1):
    t, mix, des, dt = rows[k]
    nmix = rows[k+1][1]
    if des < 0.2 and rows[k+1][2] == des and mix > des + 0.02 and nmix > des + 0.01:  # inside a ramp-down, away from its end
        step = 0.5 * (dt if mode == 'base' else 1.0/999)
        c[round((mix - nmix)/step)] += 1
tot = sum(c.values())
mean = sum(k*v for k, v in c.items())/tot if tot else float('nan')
print(f'{path}: loops={tot} iterations/main-loop histogram {dict(sorted(c.items()))} mean={mean:.2f}')
