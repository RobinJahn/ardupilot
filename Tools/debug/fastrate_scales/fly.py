#!/usr/bin/env python3
# Fly a SITL copter through a fixed profile that exercises the throttle gain boost and throttle mix slew.
# usage: fly.py <arducopter binary> <workdir> <comma separated parm files> <label>
#  A) ALT_HOLD descend/hold cycles (mix ramp-down usually cut short by the mix_used check)
#  B) ALT_HOLD climb (des=MAX 0.5) <-> STABILIZE at throttle above hover (des=MAN 0.1):
#     gives des up/down steps while throttle_out >= hover, so the ramp-down is slew limited
#  C) STABILIZE throttle punches with roll (throttle slew > 1.0 -> gain boost)
import sys, time, subprocess, os, signal
from pymavlink import mavutil

binary, workdir, extra_parm, label = sys.argv[1:5]
os.makedirs(workdir, exist_ok=True)
here = os.path.dirname(os.path.abspath(__file__))
ap = os.path.abspath(os.path.join(here, '..', '..', '..'))
extra_abs = ",".join(f if os.path.isabs(f) else os.path.join(here, f) for f in extra_parm.split(",") if f)
defaults = f"{ap}/Tools/autotest/default_params/copter.parm,{extra_abs}"
print(f"[{label}] defaults={defaults}", flush=True)
log = open(os.path.join(workdir, 'sitl.out'), 'w')
proc = subprocess.Popen([binary, '--model', '+', '--speedup', '1', '-w', '-I0',
                         '--defaults', defaults, '--home', '49.0134,12.1016,340,0'],
                        cwd=workdir, stdout=log, stderr=subprocess.STDOUT)
try:
    time.sleep(3)
    mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255, retries=60)
    mav.wait_heartbeat(timeout=60)
    t0 = time.time()

    def drain(dur):
        end = time.time() + dur
        while time.time() < end:
            m = mav.recv_match(blocking=True, timeout=0.05)
            if m is None:
                continue
            if m.get_type() == 'STATUSTEXT':
                print(f"[{label}] {time.time()-t0:6.1f} {m.text}", flush=True)

    def rc(thr, roll=1500, pitch=1500, yaw=1500):
        vals = [roll, pitch, thr, yaw] + [0]*14
        mav.mav.rc_channels_override_send(mav.target_system, mav.target_component, *vals)

    def hold(thr, dur, **kw):
        end = time.time() + dur
        while time.time() < end:
            rc(thr, **kw)
            drain(0.04)

    def set_mode(name):
        mav.set_mode(mav.mode_mapping()[name])
        drain(0.05)

    hold(1000, 25)
    set_mode('STABILIZE')
    for attempt in range(20):
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                  mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        hold(1000, 1.5)
        if mav.motors_armed():
            break
    print(f"[{label}] armed={mav.motors_armed()}", flush=True)
    hold(1700, 3)                 # climb
    set_mode('ALT_HOLD')
    hold(1500, 3)
    # A) original descend/hold cycles
    for i in range(2):
        hold(1150, 2.0)
        hold(1500, 2.5)
    # B) des MAX<->MAN steps with throttle above MOT_THST_HOVER at the switch instant
    for i in range(3):
        rc(1900); set_mode('ALT_HOLD')
        hold(1900, 0.8)           # accelerating climb -> des = MAX (0.5), throttle > hover
        rc(1600); set_mode('STABILIZE')
        hold(1600, 1.3)           # manual throttle (~0.49) above hover -> des = MAN (0.1), slew-limited ramp-down
        hold(1400, 1.0)           # slow the climb again (mix already at MAN)
    # C) throttle punches in STABILIZE with roll
    set_mode('STABILIZE')
    for i in range(8):
        hold(1850, 0.35, roll=1650)
        hold(1350, 0.35, roll=1350)
    hold(1550, 1.0)
    set_mode('ALT_HOLD')
    hold(1500, 1.5)
    set_mode('LAND')
    hold(1500, 8)
    print(f"[{label}] done armed={mav.motors_armed()}", flush=True)
finally:
    proc.send_signal(signal.SIGINT)
    time.sleep(2)
    proc.kill()
