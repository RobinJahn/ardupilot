#!/usr/bin/env python3
# For consecutive ATSC rows (rate-thread, logged every D rate iterations) that straddle a main-loop
# timestamp T (DMIX.TimeUS == reset loop time, sim clock frozen during the loop) strictly
# (t_a < T < t_b), an effective reset at T bounds p_b by the iterations since T. If no logging slot was
# skipped between a and b, that is <= D. p_b > D therefore indicates a lost/overwritten reset.
import sys, math, bisect
from pymavlink import mavutil
path, D = sys.argv[1], int(sys.argv[2])
m = mavutil.mavlink_connection(path)
at, dm = [], []
while True:
    x = m.recv_match(type=['ATSC', 'DMIX'])
    if x is None: break
    if x.get_type() == 'ATSC': at.append((x.TimeUS, round(math.log(x.PDScX)/math.log(1.4)) if x.PDScX > 1.001 else 0))
    else: dm.append(x.TimeUS)
viol = []; straddle = 0
for (ta, pa), (tb, pb) in zip(at, at[1:]):
    i = bisect.bisect_right(dm, ta)
    if i < len(dm) and dm[i] < tb:
        straddle += 1
        if pb > D: viol.append((ta, pa, tb, pb, dm[i]))
print(f'{path}: D={D} straddling pairs={straddle}, p_b>D: {len(viol)}')
for v in viol[:8]: print('  ', v)
