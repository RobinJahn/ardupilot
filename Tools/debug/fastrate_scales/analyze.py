#!/usr/bin/env python3
# Summarise ATSC, DAPS, DMIX and rate thread timing from a log written by the debug build.
# usage: analyze.py <workdir or .BIN>
import sys, glob, os, math, re
from collections import Counter
from pymavlink import mavutil

arg = sys.argv[1]
if arg.endswith('.BIN'):
    path = arg
else:
    path = sorted(glob.glob(os.path.join(arg, 'logs', '*.BIN')), key=os.path.getmtime)[-1]
m = mavutil.mavlink_connection(path)
atsc, daps, dmix, msgs, modes, rtdt, arm = [], [], [], [], [], [], []
parms = {}
want = ('SCHED_LOOP_RATE', 'FSTRATE_ENABLE', 'FSTRATE_DIV', 'ATC_THR_G_BOOST', 'INS_GYRO_RATE',
        'INS_FAST_SAMPLE', 'ATC_THR_MIX_MAN', 'ATC_THR_MIX_MIN', 'ATC_THR_MIX_MAX', 'LOG_BITMASK')
while True:
    x = m.recv_match(type=['ATSC', 'DAPS', 'DMIX', 'MSG', 'MODE', 'RTDT', 'PARM', 'ARM'])
    if x is None:
        break
    t = x.get_type()
    if t == 'ATSC': atsc.append((x.TimeUS, x.AngPScX, x.PDScX))
    elif t == 'DAPS': daps.append((x.TimeUS, x.X, x.Y, x.PDX))
    elif t == 'DMIX': dmix.append((x.TimeUS, x.Mix, x.Des, x.ThrIn, x.ThrOut, x.ThrH, x.Slew, x.dt))
    elif t == 'MSG': msgs.append((x.TimeUS, x.Message))
    elif t == 'MODE': modes.append((x.TimeUS, x.Mode))
    elif t == 'RTDT': rtdt.append(x.to_dict())
    elif t == 'ARM': arm.append((x.TimeUS, x.ArmState))
    elif t == 'PARM' and x.Name in want: parms[x.Name] = x.Value

def pw(v, b):
    return math.log(v)/math.log(b) if v > 0 else float('nan')

print('log', path)
print('parms', {k: parms.get(k) for k in want})
rate_hz = None
for tu, s in msgs:
    if 'rate set to' in s:
        print(f'MSG @{tu*1e-6:.1f}s: {s}')
        rate_hz = int(re.search(r'(\d+)Hz', s).group(1))
# main loop rate from DMIX timestamps (one DMIX per main loop)
if len(dmix) > 10:
    ints = sorted(dmix[i+1][0]-dmix[i][0] for i in range(len(dmix)-1))
    med = ints[len(ints)//2]
    dts = [d[7] for d in dmix]
    loop_hz = 1e6/med
    print(f'main loop: median DMIX interval {med} us -> {loop_hz:.1f} Hz; mean _dt_s {sum(dts)/len(dts)*1e3:.3f} ms; '
          f'DMIX intervals p1/p99 {ints[len(ints)//100]}/{ints[len(ints)*99//100]} us')
    if rate_hz and int(parms.get('FSTRATE_ENABLE', 0)) != 0:
        print(f'N = rate-thread {rate_hz} Hz / main loop {loop_hz:.1f} Hz = {rate_hz/loop_hz:.2f}')
if rtdt:
    dta = [r['dtAvg'] for r in rtdt]; dtm = [r['dtMax'] for r in rtdt]
    print(f'RTDT n={len(rtdt)} sensor dt(avg field) {sum(dta)/len(dta)*1e3:.3f} ms; wall dt max over log {max(dtm)*1e3:.2f} ms')

# ---- ATSC
print('--- ATSC (logged scale used by rate controller)')
if atsc:
    mx_pd = max(a[2] for a in atsc); mx_ap = max(a[1] for a in atsc)
    print(f'ATSC n={len(atsc)} max PDScX={mx_pd:.4g} (1.4^{pw(mx_pd,1.4):.3f})  max AngPScX={mx_ap:.4g} (1.96^{pw(mx_ap,1.96):.3f})')
    boosted = [a for a in atsc if a[2] > 1.001]
    pows = [pw(a[2], 1.4) for a in boosted]
    nonint = sum(1 for p in pows if abs(p-round(p)) > 0.02)
    print(f'  PDScX>1: {len(boosted)} rows; power-of-1.4 histogram {dict(sorted(Counter(round(p) for p in pows).items()))}; non-integer powers: {nonint}')
    apows = [pw(a[1], 1.96) for a in atsc if a[1] > 1.001]
    print(f'  AngPScX power-of-1.96 histogram {dict(sorted(Counter(round(p) for p in apows).items()))}')
    below = sum(1 for a in atsc if a[2] < 0.999)
    print(f'  rows with PDScX<1 (landed gain reduction): {below}')
else:
    print('ATSC: none')

# ---- DAPS
print('--- DAPS (what update_ang_vel_target_from_att_error actually reads)')
nb = [d for d in daps if abs(d[1]-1.0) > 1e-4]
print(f'DAPS n={len(daps)}; X!=1: {len(nb)}')
if nb:
    print('  X as power of 1.96:', dict(sorted(Counter(round(pw(d[1],1.96),2) for d in nb).items())), f' max X {max(d[1] for d in nb):.4g}')
    lt = [d for d in nb if d[1] < 1]
    print(f'  of which X<1 (landed gain reduction): {len(lt)}')
pd = [d for d in daps if abs(d[3]-1.0) > 1e-4]
print(f'  PDX!=1: {len(pd)}', dict(sorted(Counter(round(pw(d[3],1.4),2) for d in pd if d[3] > 0).items())))

# Pair DAPS with main loops in which the rate controller logged a boosted scale.
# ATSC.TimeUS = _rate_gyro_time_us (time of the rate run). DAPS in the same main loop has TimeUS >= that
# and < the next DMIX after it. Use DMIX rows as loop delimiters.
if atsc and dmix:
    import bisect
    dmt = [d[0] for d in dmix]
    dat = [d[0] for d in daps]
    boost_loops = 0; seen_one = 0; seen_boost = 0; vals = Counter()
    used = set()
    for a in atsc:
        if a[1] <= 1.001: continue
        # end of this main loop: first DMIX with time >= ATSC time
        j = bisect.bisect_left(dmt, a[0])
        if j >= len(dmt): continue
        if j in used: continue
        used.add(j)
        t_end = dmt[j]
        t_beg = dmt[j-1] if j > 0 else 0
        # DAPS samples written in this main loop (between previous DMIX and this DMIX)
        k0 = bisect.bisect_right(dat, t_beg); k1 = bisect.bisect_right(dat, t_end)
        for k in range(k0, k1):
            boost_loops += 1
            v = daps[k][1]
            vals[round(v, 4)] += 1
            if abs(v-1) < 1e-4: seen_one += 1
            else: seen_boost += 1
    print(f'  DAPS samples in main loops where ATSC logged AngPScX>1: {boost_loops}; saw X=1.0: {seen_one}; saw X!=1: {seen_boost}; values {dict(sorted(vals.items())[:12])}')

# ---- DMIX
print('--- DMIX (throttle-rpy mix slew; design up 2.0/s, down 0.5/s)')
if dmix:
    hi = [d for d in dmix if d[6] > 1.0]
    print(f'main loops with throttle slew > 1.0 (boost condition): {len(hi)}; max slew {max(d[6] for d in dmix):.2f}')
    i = 1
    events = []
    while i < len(dmix):
        if abs(dmix[i][2] - dmix[i-1][2]) > 0.05:
            events.append(i)
        i += 1
    ups, downs = [], []
    for i0 in events:
        up = dmix[i0][2] > dmix[i0-1][2]
        tgt = dmix[i0][2]
        ms, ts = dmix[i0-1][1], dmix[i0-1][0]
        j = i0
        rows = [dmix[i0-1]]
        reached = None
        while j < len(dmix) and abs(dmix[j][2]-tgt) < 1e-6:
            rows.append(dmix[j])
            if abs(dmix[j][1]-tgt) < 1e-4:
                reached = j; break
            j += 1
        if reached is None:
            print(f'  {"UP  " if up else "DOWN"} @{ts*1e-6:7.2f}s {ms:.3f}->{tgt:.3f}: des changed again / log end before reaching target (rows {len(rows)-1}, mix {rows[-1][1]:.3f})')
            continue
        steps = [(rows[k][1]-rows[k+1][1] if not up else rows[k+1][1]-rows[k][1], (rows[k+1][0]-rows[k][0])*1e-6) for k in range(len(rows)-1)]
        total_t = (rows[-1][0]-ts)*1e-6
        if abs(ms - tgt) < 1e-4:
            print(f'  {"UP  " if up else "DOWN"} @{ts*1e-6:7.2f}s mix already at target {tgt:.3f}'); continue
        full = steps[:-1]  # last step may be partial (MIN(...)) or a snap
        if full:
            dm = sum(s[0] for s in full); dt = sum(s[1] for s in full)
            slope = dm/dt if dt > 0 else float('nan')
        else:
            slope = float('nan')
        last = steps[-1]
        med = sorted(s[0] for s in full)[len(full)//2] if full else float('nan')
        # snap = last step much larger than the regular steps (or a single step larger than any design step)
        snap = (last[0] > 3*max(s[0] for s in full) + 1e-6) if full else (last[0] > 0.05)
        # code: throttle_out = MAX(get_throttle_out(), get_throttle()); shortcut can only cut when that is < hover.
        # rows[0] (row before the des change) approximates the throttle seen by the first slew step.
        thr_ok = all(max(r[4], r[3]) >= r[5] for r in rows)
        print(f'  {"UP  " if up else "DOWN"} @{ts*1e-6:7.2f}s {ms:.3f}->{tgt:.3f} in {len(steps)} loops / {total_t*1e3:.1f} ms; '
              f'slope over full steps {slope:.3f}/s (n={len(full)}); median step {med:.5f}; last step {last[0]:.4f}'
              f'{"  <-- SNAP (mix_used shortcut)" if snap else ""}{"" if up else ("  max(thrOut,thrIn)>=hover throughout" if thr_ok else "  max(thrOut,thrIn)<hover at some point")}')
        (ups if up else downs).append((slope, len(full), snap))

    def fmt(lst):
        return [f'{a:.3f}' for a, n, sn in lst if n >= 3 and not sn]
    print(f'  SUMMARY ramp-up slopes (>=3 full steps): {fmt(ups)}  (design 2.0/s)')
    print(f'  SUMMARY slew-limited ramp-down slopes (>=3 full steps, no snap): {fmt(downs)}  (design 0.5/s); '
          f'snapped ramp-downs: {sum(1 for a, n, sn in downs if sn)}')
