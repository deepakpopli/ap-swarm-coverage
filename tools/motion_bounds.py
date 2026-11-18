#!/usr/bin/env python3
"""Motion bounds for the safety filter, from the speed-cap logs (tools/speed_cap.py --run b).

    python3 tools/motion_bounds.py results/spike/speedcap_gazebo_b_log.json \
        results/spike/speedcap_sitl_b_log.json \
        --out results/spike/motion_bounds.json

For every vehicle and phase of the run-b log (cruise + stop = normal send_setpoint flight with a stop
command at 20 s; hold = zero command right after a 12 m/s overspeed with the rabbit reset, the
hardest stop the code can produce) it reports:
  * max over start times t0 of |p(t0+h) - p(t0) - v(t0)*h|, the deviation from the constant-velocity
    prediction, for horizons h = 0.3 .. 4.4 s (what swarm/safety.py's keep-out disc must cover);
  * peak |dv/dt| over 0.5 s windows;
  * reaction: time from the stop command (cruise -> stop at 20.0 s) until the speed has dropped by
    0.1 m/s.
The model columns are SafetyParams().deviation(h) for comparison.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from swarm.safety import SafetyParams  # noqa: E402

HORIZONS = (0.3, 0.5, 0.8, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.4)
GROUPS = {"cruise_stop": ("cruise", "stop"), "hold_after_overspeed": ("hold",)}


def series(log, vid, phases):
    r = [x for x in log if x["id"] == vid and x["ph"] in phases]
    t = np.array([x["t"] for x in r])
    return t, np.array([x["pos"][:2] for x in r]), np.array([x["vel"][:2] for x in r])


def deviation(t, p, v):
    out = {}
    for h in HORIZONS:
        best = 0.0
        for i in range(len(t)):
            j = np.searchsorted(t, t[i] + h - 1e-3)
            if j < len(t) and abs(t[j] - t[i] - h) < 0.02:
                best = max(best, float(np.linalg.norm(p[j] - p[i] - v[i] * (t[j] - t[i]))))
        out[str(h)] = round(best, 2)
    return out


def peak_accel(t, v, w=0.5):
    best = 0.0
    for i in range(len(t)):
        j = np.searchsorted(t, t[i] + w - 1e-3)
        if j < len(t) and 0.3 < t[j] - t[i] < 0.7:
            best = max(best, float(np.linalg.norm(v[j] - v[i]) / (t[j] - t[i])))
    return round(best, 2)


def reaction(t, v, t_cmd=20.0):
    s = np.hypot(v[:, 0], v[:, 1])
    i0 = np.searchsorted(t, t_cmd)
    for j in range(i0, len(t)):
        if s[j] < s[i0] - 0.1:
            return round(float(t[j] - t_cmd), 2)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--out")
    a = ap.parse_args()
    p = SafetyParams()
    res = {"model": {"a_max": p.a_max, "dv_max": p.dv_max, "t_react": p.t_react,
                     "deviation_m": {str(h): round(p.deviation(h), 2) for h in HORIZONS}},
           "logs": {}}
    for path in a.logs:
        log = json.load(open(path))
        per = {}
        for vid in sorted({x["id"] for x in log}):
            d = {}
            for g, phases in GROUPS.items():
                t, pos, vel = series(log, vid, phases)
                if len(t) < 10:
                    continue
                d[g] = {"max_deviation_m": deviation(t, pos, vel), "peak_accel_0p5s": peak_accel(t, vel)}
                if g == "cruise_stop":
                    d[g]["reaction_s"] = reaction(t, vel)
            per[str(vid)] = d
        res["logs"][path] = per
    txt = json.dumps(res, indent=1)
    if a.out:
        with open(a.out, "w") as f:
            f.write(txt + "\n")
    print(txt)


if __name__ == "__main__":
    main()
