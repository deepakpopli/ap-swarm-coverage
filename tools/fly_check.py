#!/usr/bin/env python3
"""End-to-end check of sim/launch.py + swarm/mav_vehicle.py on one backend.

  fly_check.py --backend sitl|gazebo [--out results/spike/fly_check_sitl.json]

1. Fleet.start(); every MavVehicle connects; before takeoff its GLOBAL_POSITION_INT converted with
   config.lla_to_enu is compared with config.start_xy (position-frame check).
2. All vehicles take off to config.ALT_NOMINAL (in parallel threads).
3. ~20 s of send_setpoint at 10 Hz from ONE loop for all vehicles:
   10 s north at V_CRUISE, 4 s zero, 6 s south at V_CRUISE / 2.
4. Fleet.kill_vehicle(middle id) (SIGKILL). For 10 s more the survivors fly north at 5 m/s then
   stop; we record when the victim's link_ok turns False and how long its poll() calls take.
5. Prints a JSON summary (also written to --out).
"""
import argparse
import json
import math
import os
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from sim.launch import Fleet                 # noqa: E402
from swarm import config                     # noqa: E402
from swarm.mav_vehicle import MavVehicle     # noqa: E402

DT = 1.0 / config.CONTROL_HZ


def pct(values, q):
    s = sorted(values)
    return s[min(len(s) - 1, int(q * (len(s) - 1) + 0.5))]


def connect_all(vehicles):
    """Connect every vehicle and wait for its first position. Returns nothing; raises on failure."""
    for v in vehicles:
        v.connect()
    deadline = time.time() + 30
    while any(v.pos is None for v in vehicles):
        if time.time() > deadline:
            raise RuntimeError("no GLOBAL_POSITION_INT")
        for v in vehicles:
            v.poll()
        time.sleep(0.05)


def frame_check(fleet, vehicles, wait_s=20.0):
    """Compare reported ENU position with the configured start, once the EKF has settled.

    Returns {id: {"expected": [x, y], "measured": [x, y, z], "err_m": horizontal error}}.
    """
    t_end = time.time() + wait_s
    while time.time() < t_end:          # GPS/EKF needs a few seconds after boot
        for v in vehicles:
            v.poll()
        time.sleep(0.05)
    out = {}
    for v in vehicles:
        ex, ey = fleet.start_xy(v.agent_id)
        x, y, z = v.pos
        out[v.agent_id] = {"expected": [round(ex, 2), round(ey, 2)], "measured": [round(x, 2), round(y, 2), round(z, 2)],
                           "err_m": round(math.hypot(x - ex, y - ey), 2)}
    return out


def takeoff_all(vehicles, alt):
    """arm_and_takeoff for all vehicles in parallel. Returns {id: wall time at altitude}."""
    done, errors = {}, {}

    def one(v):
        try:
            v.arm_and_takeoff(alt)
            done[v.agent_id] = time.time()
        except Exception as e:     # noqa: BLE001
            errors[v.agent_id] = repr(e)

    threads = [threading.Thread(target=one, args=(v,), daemon=True) for v in vehicles]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    if errors:
        raise RuntimeError(f"takeoff failed: {errors}")
    return done


def command_at(el):
    """(vx, vy) for the 20 s flight phase at elapsed time el (s)."""
    if el < 10.0:
        return 0.0, config.V_CRUISE
    if el < 14.0:
        return 0.0, 0.0
    return 0.0, -config.V_CRUISE / 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", choices=["sitl", "gazebo"], default="sitl")
    ap.add_argument("-n", type=int, default=3)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    run_dir = os.path.join(ROOT, "run", f"flycheck_{a.backend}")
    res = {"backend": a.backend, "n": a.n}
    with Fleet(a.backend, a.n, run_dir) as fleet:
        fleet.start()
        vehicles = [MavVehicle(i, port=fleet.port(i)) for i in range(1, a.n + 1)]
        connect_all(vehicles)
        res["frame_check_on_ground"] = frame_check(fleet, vehicles)
        done = takeoff_all(vehicles, config.ALT_NOMINAL)
        res["launch_to_all_airborne_s"] = round(max(done.values()) - fleet.t_start, 1)
        print("all airborne after", res["launch_to_all_airborne_s"], "s", flush=True)

        # ---- 20 s flight, one 10 Hz loop for everybody
        periods, speeds, alt_err = [], [], []
        start_pos = {v.agent_id: v.pos for v in vehicles}
        t0 = last = time.time()
        nxt = t0
        while time.time() - t0 < 20.0:
            now = time.time()
            periods.append(now - last)
            last = now
            vx, vy = command_at(now - t0)
            for v in vehicles:
                v.poll()
                v.send_setpoint(vx, vy, config.ALT_NOMINAL)
                speeds.append(math.hypot(v.vel[0], v.vel[1]))
                alt_err.append(abs(v.pos[2] - config.ALT_NOMINAL))
            nxt += DT
            time.sleep(max(0.0, nxt - time.time()))
        periods = periods[1:]
        res["flight"] = {
            "loop_period_ms_mean": round(1000 * sum(periods) / len(periods), 2),
            "loop_period_ms_p99": round(1000 * pct(periods, 0.99), 2),
            "loop_period_ms_max": round(1000 * max(periods), 2),
            "max_hspeed_ms": round(max(speeds), 2),
            "alt_err_max_m": round(max(alt_err), 2),
            "alt_err_mean_m": round(sum(alt_err) / len(alt_err), 3),
            "north_travel_m": {v.agent_id: round(v.pos[1] - start_pos[v.agent_id][1], 1) for v in vehicles},
        }

        # ---- SIGKILL one autopilot, keep flying the others
        victim_id = a.n // 2 + 1
        victim = vehicles[victim_id - 1]
        survivors = [v for v in vehicles if v is not victim]
        t_kill = fleet.kill_vehicle(victim_id)
        before = {v.agent_id: v.pos for v in survivors}
        t_link_false, victim_poll_ms, survivor_link_false = None, [], 0
        nxt = time.time()
        while time.time() - t_kill < 10.0:
            el = time.time() - t_kill
            vy = 5.0 if el < 6.0 else 0.0
            for v in survivors:
                v.poll()
                v.send_setpoint(0.0, vy, config.ALT_NOMINAL)
                survivor_link_false += 0 if v.link_ok() else 1
            tp = time.time()
            victim.poll()
            victim.send_setpoint(0.0, vy, config.ALT_NOMINAL)    # must not raise or block
            victim_poll_ms.append(1000 * (time.time() - tp))
            st = victim.state()
            if t_link_false is None and st is not None and not st.link_ok:
                t_link_false = time.time() - t_kill
            nxt += DT
            time.sleep(max(0.0, nxt - time.time()))
        res["kill"] = {
            "victim_id": victim_id,
            "victim_link_ok_false_after_s": round(t_link_false, 2) if t_link_false is not None else None,
            "victim_poll_plus_send_ms_max": round(max(victim_poll_ms), 3),
            "survivor_link_false_samples": survivor_link_false,
            "survivor_travel_after_kill_m": {v.agent_id: round(math.hypot(v.pos[0] - before[v.agent_id][0],
                                                                          v.pos[1] - before[v.agent_id][1]), 1)
                                             for v in survivors},
            "survivor_speed_at_end_ms": {v.agent_id: round(math.hypot(v.vel[0], v.vel[1]), 2) for v in survivors},
            "survivor_alt_m": {v.agent_id: round(v.pos[2], 2) for v in survivors},
        }
        for v in vehicles:
            v.close()
    print(json.dumps(res, indent=1))
    if a.out:
        json.dump(res, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
