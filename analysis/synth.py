"""Synthetic run directory generator, for testing the analysis WITHOUT the simulator.

NOT measured data: numbers computed from these runs say nothing about the real system.

Model: N agents start on the south edge (config.start_xy) already at 60 m, then fly north-south
lawnmower lanes at V_CRUISE inside equal-width vertical strips of the area, yaw held north.
At a seeded random time in [60, 90] s one agent stops logging (its last CSV line is truncated,
as after a SIGKILL). The ground declares it lost DETECT_S later and re-splits the area among
the survivors, who fly straight to their new lanes.

Usage: python -m analysis.synth run/synth_demo [--seed 1]
"""
import argparse
import json
import math
import os

import numpy as np

from swarm import config, sensor
from analysis.logs import AGENT_COLUMNS

DETECT_S = 12.0
SUSPECT_S = 2.0
LANE_MARGIN_Y = 20.0     # lanes run from y=20 to y=280 (footprint half-length 29 m covers the edge)
DT = 1.0 / config.CONTROL_HZ


def share_bounds(k, n):
    xmin, xmax, ymin, ymax = config.AREA
    w = (xmax - xmin) / n
    return [xmin + k * w, xmin + (k + 1) * w, ymin, ymax]


def lane_waypoints(share, alt=config.ALT_NOMINAL):
    """Boustrophedon loop over north-south lanes covering the share. Lane count: enough lanes
    that the across-track footprint (with 10 % overlap) spans the share width."""
    xmin, xmax, ymin, ymax = share
    swath = sensor.footprint_size(alt)[0] * 0.9
    n_lanes = max(1, math.ceil((xmax - xmin) / swath))
    xs = [xmin + (i + 0.5) * (xmax - xmin) / n_lanes for i in range(n_lanes)]
    y0, y1 = ymin + LANE_MARGIN_Y, ymax - LANE_MARGIN_Y
    wps = []
    for i, x in enumerate(xs):
        wps += [(x, y0), (x, y1)] if i % 2 == 0 else [(x, y1), (x, y0)]
    if n_lanes == 1 or n_lanes % 2 == 1:
        wps += [(x, y) for x, y in reversed(wps)][1:-1]   # odd lane count: retrace to close the loop
    return wps


class Mover:
    """Point mass flying waypoint to waypoint at constant speed, looping over the list."""

    def __init__(self, pos, wps, speed=config.V_CRUISE):
        self.pos = np.array(pos, dtype=float)
        self.set_path(wps)
        self.speed = speed

    def set_path(self, wps):
        self.wps = [np.array(w, dtype=float) for w in wps]
        self.k = 0

    def step(self, dt):
        """Advance dt seconds; returns velocity (vx, vy)."""
        start = self.pos.copy()
        budget = self.speed * dt
        while budget > 1e-9:
            d = self.wps[self.k] - self.pos
            dist = float(np.hypot(*d))
            if dist <= budget:
                self.pos = self.wps[self.k].copy()
                budget -= dist
                self.k = (self.k + 1) % len(self.wps)
                if dist < 1e-9 and len(self.wps) == 1:
                    break
            else:
                self.pos += d / dist * budget
                budget = 0.0
        return (self.pos - start) / dt


def generate(out_dir, seed=1, n=3, mission_s=config.MISSION_S, t0_wall=1_700_000_000.0):
    rng = np.random.default_rng(seed)
    os.makedirs(out_dir, exist_ok=True)
    victim = int(rng.integers(1, n + 1))
    t_kill = float(np.round(rng.uniform(60.0, 90.0), 1))
    t_detect = t_kill + DETECT_S
    ids = list(range(1, n + 1))
    alt = config.ALT_NOMINAL

    movers = {}
    for i, aid in enumerate(ids):
        sx, sy = config.start_xy(i, n)
        movers[aid] = Mover((sx, sy), lane_waypoints(share_bounds(i, n)))
    rows = {aid: [] for aid in ids}
    epoch = 1
    replanned = False
    steps = int(round((mission_s + 2.0) / DT))
    for s in range(-20, steps):                     # 2 s of pre-start rows (t_mission empty)
        t = s * DT
        if not replanned and t >= t_detect:
            replanned, epoch = True, 2
            alive = [a for a in ids if a != victim]
            for k, aid in enumerate(alive):
                wps = lane_waypoints(share_bounds(k, len(alive)))
                near = min(range(len(wps)), key=lambda j: np.hypot(*(np.array(wps[j]) - movers[aid].pos)))
                movers[aid].set_path(wps[near:] + wps[:near])
        live = [a for a in ids if not (a == victim and t >= t_kill)]
        vel = {aid: (movers[aid].step(DT) if t >= 0 else np.zeros(2)) for aid in live}
        for aid in live:
            p = movers[aid].pos
            others = [np.hypot(*(movers[o].pos - p)) for o in live if o != aid]
            z = alt + rng.normal(0, 0.3)
            rows[aid].append({
                "t_wall": f"{t0_wall + t + rng.normal(0, 0.002):.3f}",
                "t_mission": f"{t:.2f}" if t >= 0 else "",
                "x": f"{p[0] + rng.normal(0, 0.3):.2f}", "y": f"{p[1] + rng.normal(0, 0.3):.2f}",
                "z": f"{z:.2f}", "vx": f"{vel[aid][0]:.2f}", "vy": f"{vel[aid][1]:.2f}", "vz": "0.00",
                "yaw": f"{rng.normal(0, 0.02):.4f}", "status": "cover" if t >= 0 else "ready",
                "epoch": epoch, "slot": movers[aid].k,
                "cmd_vx": f"{vel[aid][0]:.2f}", "cmd_vy": f"{vel[aid][1]:.2f}", "cmd_z": f"{alt:.1f}",
                "nom_vx": f"{vel[aid][0]:.2f}", "nom_vy": f"{vel[aid][1]:.2f}",
                "min_nbr_dist": f"{min(others):.2f}" if others else "", "min_nbr_age": "0.20",
                "flags": ""})

    for aid in ids:
        with open(os.path.join(out_dir, f"agent_{aid}.csv"), "w") as f:
            f.write(",".join(AGENT_COLUMNS) + "\n")
            for r in rows[aid]:
                f.write(",".join(str(r[c]) for c in AGENT_COLUMNS) + "\n")
            if aid == victim:
                r = rows[aid][-1]
                f.write(",".join(str(r[c]) for c in AGENT_COLUMNS)[:23])   # SIGKILL mid-write

    def assign_body(alive, ep):
        return {k: {"share": share_bounds(i, len(alive)),
                    "slots": [list(w) for w in lane_waypoints(share_bounds(i, len(alive)))],
                    "loop_from": 0} for i, k in enumerate(alive)}

    def ev(t, event, **kw):
        return dict({"t_wall": t0_wall + t, "t_mission": t, "event": event}, **kw)

    alive = [a for a in ids if a != victim]
    ground = [ev(0.0, "start"),
              ev(0.0, "assign", epoch=1, trigger="start", assignments=assign_body(ids, 1), changed=ids),
              ev(t_kill + SUSPECT_S, "liveness", agent=victim, **{"from": "alive", "to": "suspect"}),
              ev(t_detect, "liveness", agent=victim, **{"from": "suspect", "to": "lost"}),
              ev(t_detect, "assign", epoch=2, trigger="lost", assignments=assign_body(alive, 2),
                 changed=alive)]
    with open(os.path.join(out_dir, "ground.jsonl"), "w") as f:
        for e in ground:
            f.write(json.dumps(e) + "\n")
    meta = {"seed": seed, "n": n, "backend": "synthetic", "t0_wall": t0_wall,
            "kill_agent": victim, "kill_t_mission": t_kill, "mission_s": mission_s,
            "radio": {"loss": config.RADIO_LOSS, "delay_s": config.RADIO_DELAY_S},
            "note": "SYNTHETIC test data from analysis/synth.py, not a measurement"}
    with open(os.path.join(out_dir, "meta.json"), "w") as f:
        json.dump(meta, f, indent=1)
    with open(os.path.join(out_dir, "radio.jsonl"), "w") as f:
        f.write(json.dumps({"t_wall": t0_wall + mission_s, "sent": 0, "dropped": 0, "delivered": 0,
                            "note": "synthetic placeholder"}) + "\n")
    return meta


def main():
    ap = argparse.ArgumentParser(description="write a SYNTHETIC run directory")
    ap.add_argument("out_dir")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    m = generate(a.out_dir, a.seed)
    print(f"synthetic run -> {a.out_dir}: victim {m['kill_agent']} at t={m['kill_t_mission']} s")


if __name__ == "__main__":
    main()
