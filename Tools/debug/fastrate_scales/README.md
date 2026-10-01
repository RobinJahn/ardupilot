# Fast rate thread: throttle gain boost and throttle mix slew

Debug build and SITL scripts to reproduce three issues with `AC_AttitudeControl_Multi::rate_controller_run_dt()`
when the fast rate thread (`FSTRATE_ENABLE` != 0) is used:

1. `update_throttle_gain_boost()` multiplies `_pd_scale` on every rate thread iteration, but the scale is only reset
   once per main loop, so it compounds to (1+`ATC_THR_G_BOOST`)^n.
2. `_angle_P_scale` is boosted in the rate controller and reset in `rate_controller_target_reset()` before
   `update_flight_mode()` runs the angle controller. Without the fast rate thread the angle controller never sees the
   boost while ATSC logs it. With the fast rate thread the boost compounds.
3. `update_throttle_rpy_mix()` slews by the main loop `_dt_s` on every rate thread iteration, so the mix slews
   (rate thread rate / main loop rate) times too fast.

## Debug log messages

The two `DEBUG` commits on this branch add:

| Message | Written | Fields |
|---|---|---|
| `DAPS` | in `update_ang_vel_target_from_att_error()`, where the angle controller reads the scale | `X`, `Y`: `_angle_P_scale.x/y`; `PDX`: `_pd_scale.x` |
| `DMIX` | end of `Copter::update_flight_mode()`, once per main loop | `Mix`, `Des`: current and desired throttle mix; `ThrIn`, `ThrOut`, `ThrH`; `Slew`: throttle slew rate; `dt`: attitude controller dt |

## Running

Needs a working SITL build environment and pymavlink.

```
Tools/debug/fastrate_scales/run.sh
```

This builds SITL copter, then flies `fly.py` for four configurations and runs `analyze.py` on each log:

| Name | Parameters | Rate thread iterations per main loop |
|---|---|---|
| `nofast400` | `FSTRATE_ENABLE 0`, `SCHED_LOOP_RATE 400` | 1 |
| `fast400` | `FSTRATE_ENABLE 2`, `FSTRATE_DIV 1`, `SCHED_LOOP_RATE 400` | 2.5 |
| `fast200` | as above, `SCHED_LOOP_RATE 200` | 5 |
| `fast100` | as above, `SCHED_LOOP_RATE 100` | 10 |

All use `ATC_THR_G_BOOST 0.4`. The SITL gyro runs at a fixed 1 kHz (`INS_SITL_SENSOR_A`), so the rate thread cannot run
faster than that. More iterations per main loop are obtained by lowering `SCHED_LOOP_RATE` instead.

A single configuration: `run.sh fast400:common.parm,fast.parm`. Each run takes about 75 s (speedup 1).

## Profile

`fly.py` arms in STABILIZE, climbs, then:

- ALT_HOLD descend/hold cycles,
- three cycles of ALT_HOLD climb (desired mix = `THR_MIX_MAX`) to STABILIZE above hover throttle
  (desired mix = `THR_MIX_MAN`), which gives throttle mix ramp-downs limited by the 0.5/s slew,
- eight throttle punches with roll in STABILIZE, which drive the throttle slew above 1.0 and trigger the boost,
- LAND.

## Reading the output

- `ATSC max PDScX` and its power of 1.4: compounding of the PD boost (issue 1).
- `DAPS ... saw X=1.0 / saw X!=1`: what the angle controller read in main loops where ATSC logged a boosted angle P
  scale (issue 2).
- `SUMMARY ramp-up slopes` / `slew-limited ramp-down slopes`: throttle mix slew rate, design 2.0/s up and 0.5/s down
  (issue 3).

## Results on master 755258d

| | N | max ATSC.PDScX | angle P scale read by the angle controller while boosted | mix slope up / down [1/s] |
|---|---|---|---|---|
| `nofast400` | 1 | 1.4 | 1.0 in every boosted loop, ATSC logs 1.96 | 2.0 / 0.5 |
| `fast400` | 2.5 | 1.4^8 | 1.96^1 to 1.96^5 | 5.0 / 1.25 |
| `fast200` | 5 | 1.4^9 | 1.96^1 to 1.96^5 | 10 / 2.5 |
| `fast100` | 10 | 1.4^16 | 1.96^1 to 1.96^6 | 20 / 5.0 |

With the fix applied: max PDScX 1.4, the angle controller reads 1.96 whenever boosted, and the mix slews at 2.0 / 0.5
at every N.

## Caveats

- In SITL only the main thread advances simulated time, while the rate thread consumes gyro samples asynchronously.
  The number of rate thread iterations between two main loop resets therefore varies more than on hardware, and the
  exponents above (e.g. 1.4^16 at N = 10) are not representative of a real flight controller. The scaling with N is.
- ATSC is written from the rate thread at a decimated rate, so it samples the scales rather than logging every value.

## Helpers

- `iters_per_loop.py <BIN> base|fix`: infers rate thread iterations per main loop from the mix decrements.
- `lost_reset_check.py <BIN> <decimation>`: looks for ATSC rows that show a reset being overwritten by the rate
  thread.
