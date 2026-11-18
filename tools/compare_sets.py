#!/usr/bin/env python3
"""Side-by-side table of several evaluation sets, from their aggregate results.json.

    python tools/compare_sets.py "label=results_dir:runs_dir" ... > results/comparison.md

Every cell is median [min, max] over the set's runs, n stated in the header. Attempts per seed
= 1 + number of failed attempts kept under <runs_dir>/failed/ (run_eval.sh moves them there).
"""
import json
import os
import re
import sys

ROWS = [  # (label, kind, metric name in results.json, phase or None)
    ("Coverage before the loss (60 s window), %", "paired", "Coverage (60 s window)", "before"),
    ("Coverage after recovery (60 s window), %", "paired", "Coverage (60 s window)", "after"),
    ("Coverage minimum after loss (dip), %", "single", "Coverage minimum after loss (dip)", None),
    ("Recovery (a), s", "single", "Recovery (a): back within 10 % of pre-loss coverage", None),
    ("Recovery (b), s", "single", "Recovery (b): back within 10 % of (N-1)/N x pre-loss", None),
    ("Post-recovery coverage / ideal (N-1)/N", "single",
     "Post-recovery coverage / ideal (N-1)/N share", None),
    ("Worst revisit gap before, s", "paired", "Worst revisit gap, clipped to phase", "before"),
    ("Worst revisit gap after, s", "paired", "Worst revisit gap, clipped to phase", "after"),
    ("Worst staleness after (unclipped age), s", "paired", "Worst staleness (unclipped age)",
     "after"),
    ("Cells never seen after recovery, %", "paired", "Cells never seen in phase", "after"),
    ("Ground 'lost' declared after the kill, s", "single",
     "Ground 'lost' declaration after the kill", None),
    ("Min true 3D separation, m", "single", "Min true 3D separation, live agents", None),
    ("Time below 8 m, s", "single", "Time below 8 m", None),
    ("Infeasible reports", "single", "Infeasible reports", None),
    ("Churn excl. initial + loss re-plan, /agent/min", "single",
     "Assignment churn overall, excl. initial + loss re-plan", None),
]


def fmt(s, digits):
    if s is None or s.get("median") is None:
        return "n/a"
    f = f"{{:.{digits}f}}"
    out = f"{f.format(s['median'])} [{f.format(s['min'])}, {f.format(s['max'])}]"
    if s.get("n_none"):
        out += f" ({s['n_none']} none)"
    return out


def cell(res, kind, metric, phase):
    for r in res[kind]:
        if r["metric"] == metric:
            d = r.get("digits", 1)
            if metric == "Infeasible reports":
                d = 0
            return fmt(r[phase] if phase else r["value"], d)
    return "n/a"


def attempts(runs_dir, seeds):
    failed = os.path.join(runs_dir, "failed")
    names = os.listdir(failed) if os.path.isdir(failed) else []
    a = [1 + sum(1 for n in names if re.match(rf"seed_{s}_attempt", n)) for s in seeds]
    return f"{sorted(a)[len(a) // 2]} [{min(a)}, {max(a)}]"


def main(argv):
    sets = []
    for arg in argv:
        label, dirs = arg.split("=", 1)
        res_dir, runs_dir = dirs.split(":")
        res = json.load(open(os.path.join(res_dir, "results.json")))
        seeds = [r["seed"] for r in res["runs"]]
        sets.append((label, res, runs_dir, seeds))
    print("| Metric | " + " | ".join(f"{l} (n = {r['n_runs']})" for l, r, _, _ in sets) + " |")
    print("|---|" + "---|" * len(sets))
    for label, kind, metric, phase in ROWS:
        print(f"| {label} | " + " | ".join(cell(r, kind, metric, phase) for _, r, _, _ in sets) + " |")
    print("| Attempts per seed | " + " | ".join(attempts(d, s) for _, _, d, s in sets) + " |")


if __name__ == "__main__":
    main(sys.argv[1:])
