#!/usr/bin/env python3
"""Measure which GUIDED setpoint style ArduCopter itself caps at WP_SPD (= config.V_MAX = 12 m/s).

Three vehicles, each flies a different setpoint style while we deliberately ask for ~18 m/s north
(or give a far-away target). Every vehicle gets swarm.mav_vehicle.SPEED_PARAMS (WP_SPD=12,
ATC_ANGLE_MAX=45 so the airframe CAN exceed 12 m/s, otherwise the test would prove nothing).

  run "a":  1 vel      velocity-only target 18 m/s            (SET_POSITION_TARGET_LOCAL_NED, pos ignored)
            2 pos_far  position-only target 1000 m north      (SET_POSITION_TARGET_GLOBAL_INT, vel ignored)
            3 posvel   position moving at 18 m/s + 18 m/s velocity feed-forward
  run "b":  1 rabbit18 MavVehicle.send_setpoint(0, 18) with the code clamp OFF (pure autopilot cap)
            2 wpnav    GUID_OPTIONS bit 6 (WPNav for position targets), position target 1000 m north
            3 rabbit10 MavVehicle.send_setpoint(0, 10)  (normal cruise)

Then (run "b" only) all three fly a cruise test with send_setpoint: 10 m/s south for 20 s while
the altitude target steps 60 -> 70 -> 50 m, then a zero command (stopping distance).

Usage: speed_cap.py --backend sitl|gazebo --run a|b   (appends to results/spike/speed_cap.json)
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

from sim.launch import Fleet                         # noqa: E402
from swarm import config                             # noqa: E402
from swarm.mav_vehicle import MavVehicle, SPEED_PARAMS  # noqa: E402

DT = 1.0 / config.CONTROL_HZ
OVERSPEED_CMD = 18.0
OVERSPEED_S = 25.0
CRUISE_S = 20.0
STOP_S = 10.0
FAR_M = 1000.0

STYLES = {"a": ["vel", "pos_far", "posvel"], "b": ["rabbit18", "wpnav", "rabbit10"]}


def bring_up(vehicles, alt):
    """Connect + arm + take off all vehicles in parallel threads. Returns {id: seconds} or raises."""
    errors, done = {}, {}

    def one(v):
        try:
            v.connect()
            v.arm_and_takeoff(alt)
            done[v.agent_id] = time.time()
        except Exception as e:     # noqa: BLE001  (report, do not hide)
            errors[v.agent_id] = repr(e)

    threads = [threading.Thread(target=one, args=(v,), daemon=True) for v in vehicles]
    for t in threads:
        t.start()
    while any(t.is_alive() for t in threads):
        time.sleep(0.2)
    if errors:
        raise RuntimeError(f"bring-up failed: {errors}")
    return done


class Ticker:
    """Fixed-rate loop helper: wait() sleeps until the next DT boundary."""

    def __init__(self, dt):
        self.dt = dt
        self.next = time.time()

    def wait(self):
        self.next += self.dt
        delay = self.next - time.time()
        if delay > 0:
            time.sleep(delay)
        else:
            self.next = time.time()


def hspeed(v):
    return math.hypot(v.vel[0], v.vel[1])


def overspeed_phase(vehicles, styles, log):
    """Everybody asks for > 12 m/s north in its own style for OVERSPEED_S. Logs samples."""
    start = {v.agent_id: v.pos for v in vehicles}
    t0 = time.time()
    tick = Ticker(DT)
    while time.time() - t0 < OVERSPEED_S:
        el = time.time() - t0
        for v in vehicles:
            v.poll()
            s = styles[v.agent_id]
            x0, y0, _ = start[v.agent_id]
            if s == "vel":
                v.send_velocity_target(0.0, OVERSPEED_CMD)
            elif s == "pos_far":
                v.send_position_target(x0, y0 + FAR_M, config.ALT_NOMINAL)
            elif s == "wpnav" and (v.last_target is None or el < 1.0):
                # WPNav re-plans on every new destination, so send it only during the first second
                v.send_position_target(x0, y0 + FAR_M, config.ALT_NOMINAL)
            elif s == "posvel":
                v.send_pos_vel_target(x0, y0 + OVERSPEED_CMD * el, config.ALT_NOMINAL, 0.0, OVERSPEED_CMD)
            elif s == "rabbit18":
                v.send_setpoint(0.0, OVERSPEED_CMD, config.ALT_NOMINAL)
            elif s == "rabbit10":
                v.send_setpoint(0.0, config.V_CRUISE, config.ALT_NOMINAL)
            log.append({"ph": "over", "id": v.agent_id, "t": round(el, 2), "pos": v.pos, "vel": v.vel,
                        "tgt": v.last_target, "link": v.link_ok()})
        tick.wait()


def hold_phase(vehicles, seconds, log, tag):
    """Zero-velocity setpoints for `seconds` (rabbit style; the rabbit restarts at the vehicle)."""
    for v in vehicles:
        v.reset_rabbit()
    t0 = time.time()
    tick = Ticker(DT)
    while time.time() - t0 < seconds:
        for v in vehicles:
            v.poll()
            v.send_setpoint(0.0, 0.0, config.ALT_NOMINAL)
            log.append({"ph": tag, "id": v.agent_id, "t": round(time.time() - t0, 2), "pos": v.pos, "vel": v.vel,
                        "tgt": v.last_target, "link": v.link_ok()})
        tick.wait()


def cruise_phase(vehicles, log):
    """send_setpoint(0, -10) for CRUISE_S with altitude steps, then zero for STOP_S."""
    t0 = time.time()
    tick = Ticker(DT)
    while time.time() - t0 < CRUISE_S + STOP_S:
        el = time.time() - t0
        z = 70.0 if 5.0 <= el < 12.0 else (50.0 if el >= 12.0 else config.ALT_NOMINAL)
        vy = -config.V_CRUISE if el < CRUISE_S else 0.0
        for v in vehicles:
            v.poll()
            v.send_setpoint(0.0, vy, z)
            log.append({"ph": "cruise" if el < CRUISE_S else "stop", "id": v.agent_id, "t": round(el, 2),
                        "pos": v.pos, "vel": v.vel, "tgt": v.last_target, "zcmd": z, "vcmd": abs(vy),
                        "rabbit": v.rabbit, "link": v.link_ok()})
        tick.wait()


def summarize_overspeed(log, vid):
    rows = [r for r in log if r["ph"] == "over" and r["id"] == vid]
    sp = [math.hypot(r["vel"][0], r["vel"][1]) for r in rows]
    late = [s for r, s in zip(rows, sp) if r["t"] > OVERSPEED_S - 10]
    return {"max_hspeed_ms": round(max(sp), 2), "mean_hspeed_last10s_ms": round(sum(late) / len(late), 2),
            "max_alt_err_m": round(max(abs(r["pos"][2] - config.ALT_NOMINAL) for r in rows if r["t"] > 3), 2)}


def summarize_cruise(log, vid):
    """Speed tracking, altitude response and stopping distance of the cruise test."""
    cr = [r for r in log if r["ph"] == "cruise" and r["id"] == vid]
    st = [r for r in log if r["ph"] == "stop" and r["id"] == vid]
    sp = [math.hypot(r["vel"][0], r["vel"][1]) for r in cr]
    t_reach = next((r["t"] for r, s in zip(cr, sp) if s >= 0.95 * config.V_CRUISE), None)
    steady = [s for r, s in zip(cr, sp) if t_reach is not None and r["t"] >= t_reach + 1.0]
    x_lane = cr[0]["pos"][0]
    cross = [abs(r["pos"][0] - x_lane) for r in cr]
    lags = [math.hypot(r["rabbit"][0] - r["pos"][0], r["rabbit"][1] - r["pos"][1]) for r in cr
            if r["rabbit"] and t_reach is not None and r["t"] >= t_reach + 1.0]
    # stopping: from the first zero-command sample to rest
    y_cmd0 = st[0]["pos"][1]
    v_cmd0 = math.hypot(st[0]["vel"][0], st[0]["vel"][1])
    y_rest = st[-1]["pos"][1]
    y_min = min(r["pos"][1] for r in st)          # flying south: the furthest point is the minimum y
    t_stop = next((r["t"] - st[0]["t"] for r in st if math.hypot(r["vel"][0], r["vel"][1]) < 0.3), None)

    def alt_settle(z_target, t_from, t_to):
        rows = [r for r in cr if t_from <= r["t"] < t_to]
        ok = next((r["t"] - t_from for r in rows if abs(r["pos"][2] - z_target) < 1.0), None)
        return {"target_m": z_target, "time_to_within_1m_s": round(ok, 1) if ok is not None else None,
                "hspeed_min_during_ms": round(min(math.hypot(r["vel"][0], r["vel"][1]) for r in rows), 2)}

    return {
        "time_to_95pct_speed_s": t_reach,
        "steady_speed_mean_ms": round(sum(steady) / len(steady), 2) if steady else None,
        "steady_speed_min_ms": round(min(steady), 2) if steady else None,
        "steady_speed_max_ms": round(max(steady), 2) if steady else None,
        "cross_track_max_m": round(max(cross), 2),
        "rabbit_lag_mean_m": round(sum(lags) / len(lags), 2) if lags else None,
        "rabbit_lag_max_m": round(max(lags), 2) if lags else None,
        "alt_step_up": alt_settle(70.0, 5.0, 12.0),
        "alt_step_down": alt_settle(50.0, 12.0, CRUISE_S),
        "speed_at_stop_cmd_ms": round(v_cmd0, 2),
        "stopping_distance_m": round(y_cmd0 - y_min, 2),
        "overshoot_beyond_rest_m": round(y_rest - y_min, 2),
        "time_to_below_0p3ms_s": round(t_stop, 2) if t_stop is not None else None,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", choices=["sitl", "gazebo"], default="sitl")
    ap.add_argument("--run", choices=["a", "b"], required=True)
    a = ap.parse_args()
    styles = {i + 1: s for i, s in enumerate(STYLES[a.run])}
    run_dir = os.path.join(ROOT, "run", f"speedcap_{a.backend}_{a.run}")
    log = []
    with Fleet(a.backend, 3, run_dir) as fleet:
        fleet.start()
        vehicles = []
        for vid, s in styles.items():
            params = dict(SPEED_PARAMS)
            if s == "wpnav":
                params["GUID_OPTIONS"] = 64
            vehicles.append(MavVehicle(vid, port=fleet.port(vid), params=params, clamp_in_code=(s != "rabbit18")))
        done = bring_up(vehicles, config.ALT_NOMINAL)
        airborne_s = round(max(done.values()) - fleet.t_start, 1)
        print("all airborne after", airborne_s, "s", flush=True)
        hold_phase(vehicles, 2.0, log, "pre")
        overspeed_phase(vehicles, styles, log)
        # rabbit / position targets stop by themselves; the vel style needs an explicit zero
        if a.run == "b":
            for v in vehicles:
                if styles[v.agent_id] == "wpnav":
                    v.set_param("GUID_OPTIONS", 0)
        hold_phase(vehicles, 12.0, log, "hold")
        if a.run == "b":
            cruise_phase(vehicles, log)
        out = {"backend": a.backend, "run": a.run, "params": SPEED_PARAMS, "airborne_after_launch_s": airborne_s,
               "overspeed_cmd_ms": OVERSPEED_CMD, "overspeed_s": OVERSPEED_S, "styles": {}}
        for v in vehicles:
            s = styles[v.agent_id]
            res = {"id": v.agent_id, "params_echoed": {k: v.param_values.get(k) for k in v.params}}
            res.update(summarize_overspeed(log, v.agent_id))
            if a.run == "b":
                res["cruise_10ms"] = summarize_cruise(log, v.agent_id)
            out["styles"][s] = res
    json.dump(log, open(os.path.join(run_dir, "log.json"), "w"))
    path = os.path.join(ROOT, "results", "spike", "speed_cap.json")
    allres = json.load(open(path)) if os.path.exists(path) else {}
    allres[f"{a.backend}_{a.run}"] = out
    json.dump(allres, open(path, "w"), indent=1)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
