"""Aggregate several analysed runs into a results table.

Usage: python -m analysis.aggregate runs/a runs/b ... [--out results]
Reads (or computes, if missing) each run's metrics.json; writes <out>/results.md and
<out>/results.json. Summary cells are "median [min, max]" over the runs that have a value;
None values are never averaged in, they are counted ("not recovered in k of n runs").
Runs whose meta.json says backend "synthetic" make the whole table carry a SYNTHETIC banner.
"""
import argparse
import json
import os

import numpy as np

from analysis import metrics


REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(REPO, "results")


def check_out_dir(out_dir, synthetic):
    """results/ is reserved for measured data: refuse to write synthetic output there."""
    p = os.path.abspath(out_dir)
    if synthetic and (p == RESULTS_DIR or p.startswith(RESULTS_DIR + os.sep)):
        raise SystemExit("refusing to write SYNTHETIC runs into results/ (use e.g. run/synth_demo/)")


def load_metrics(run_dir, recompute=False):
    p = os.path.join(run_dir, "metrics.json")
    if recompute or not os.path.exists(p):
        metrics.analyse(run_dir)
    with open(p) as f:
        return json.load(f)


def get(d, *path):
    for k in path:
        if not isinstance(d, dict) or d.get(k) is None:
            return None
        d = d[k]
    return d


def summary(values):
    """{"median","min","max","n","n_none"} over a list that may contain None."""
    v = [float(x) for x in values if x is not None]
    out = {"n": len(v), "n_none": len(values) - len(v)}
    if v:
        out.update(median=float(np.median(v)), min=float(np.min(v)), max=float(np.max(v)))
    return out


def fmt(s, digits=1, none_label="n/a"):
    if s["n"] == 0:
        return f"{none_label} (all {s['n_none']})" if s["n_none"] else "n/a"
    txt = f"{s['median']:.{digits}f} [{s['min']:.{digits}f}, {s['max']:.{digits}f}]"
    if s["n_none"]:
        txt += f"; {none_label} in {s['n_none']} of {s['n'] + s['n_none']}"
    return txt


# (label, unit, path before, path after, digits)
PAIRED = [
    ("Coverage (60 s window)", "%", ("loss", "pre_loss_pct"), ("loss", "post_coverage_pct"), 1),
    ("Worst revisit gap, clipped to phase", "s", ("revisit_gap", "pre", "worst_s"), ("revisit_gap", "post", "worst_s"), 1),
    ("Worst revisit gap, seen cells only", "s", ("revisit_gap", "pre", "worst_seen_cells_s"), ("revisit_gap", "post", "worst_seen_cells_s"), 1),
    ("Worst staleness (unclipped age)", "s", ("revisit_gap", "pre", "worst_age_s"), ("revisit_gap", "post", "worst_age_s"), 1),
    ("Cells never seen in phase", "%", ("revisit_gap", "pre", "never_seen_cells_pct"), ("revisit_gap", "post", "never_seen_cells_pct"), 1),
    ("Assignment churn, all events", "/agent/min", ("churn", "pre", "per_agent_min_all"), ("churn", "post", "per_agent_min_all"), 2),
    ("Assignment churn, excl. initial + loss re-plan", "/agent/min", ("churn", "pre", "per_agent_min_excl"), ("churn", "post", "per_agent_min_excl"), 2),
]

SINGLE = [
    ("Recovery (a): back within 10 % of pre-loss coverage", "s", ("loss", "recovery_a_s"), "not recovered", 1),
    ("Recovery (b): back within 10 % of (N-1)/N x pre-loss", "s", ("loss", "recovery_b_s"), "not recovered", 1),
    ("Ground 'lost' declaration after the kill", "s", ("loss", "detection_delay_s"), "no event", 1),
    ("Coverage minimum after loss (dip)", "%", ("loss", "dip_min_pct"), "n/a", 1),
    ("Post-recovery coverage / ideal (N-1)/N share", "ratio", ("loss", "post_vs_ideal_ratio"), "n/a", 2),
    ("Post-recovery coverage minus ideal", "% points", ("loss", "post_minus_ideal_pct_points"), "n/a", 1),
    ("Min true 3D separation, live agents", "m", ("safety", "min_separation_m"), "n/a", 1),
    ("Time below 8 m", "s", ("safety", "time_below_d_min_s"), "n/a", 1),
    ("Infeasible reports", "count", ("safety", "infeasible_reports"), "n/a", 0),
    ("Assignment churn overall, all events", "/agent/min", ("churn", "overall", "per_agent_min_all"), "n/a", 2),
    ("Assignment churn overall, excl. initial + loss re-plan", "/agent/min", ("churn", "overall", "per_agent_min_excl"), "n/a", 2),
]


def per_agent_extremes(m):
    """Worst-agent control rate numbers and total fallback/deaf seconds for one run."""
    cr = [v for v in (m.get("control_rate") or {}).values() if v]
    ft = (get(m, "safety", "flag_time_s") or {}).values()
    return {"min_rate_hz": min((v["rate_hz"] for v in cr if v["rate_hz"]), default=None),
            "worst_p99_period_s": max((v["p99_period_s"] for v in cr), default=None),
            "fallback_s": sum(v["fallback_s"] for v in ft) if ft else None,
            "deaf_s": sum(v["deaf_s"] for v in ft) if ft else None}


def aggregate(run_dirs, out_dir, recompute=False):
    ms = [load_metrics(d, recompute) for d in run_dirs]
    n = len(ms)
    synthetic = any(m.get("backend") == "synthetic" for m in ms)
    check_out_dir(out_dir, synthetic)
    res ={"n_runs": n, "synthetic": synthetic, "runs": [], "paired": [], "single": []}
    for m in ms:
        res["runs"].append(dict({"run": m["run"], "seed": m.get("seed"), "victim": m.get("victim"),
                                 "t_loss_s": m.get("t_loss_s"),
                                 "lost_declared_t_s": get(m, "loss", "lost_declared_t_s"),
                                 "recovery_a_s": get(m, "loss", "recovery_a_s"),
                                 "recovery_b_s": get(m, "loss", "recovery_b_s"),
                                 "warmup_incomplete": get(m, "loss", "warmup_incomplete"),
                                 "min_separation_m": get(m, "safety", "min_separation_m")},
                                **per_agent_extremes(m)))
    for label, unit, pa, pb, dg in PAIRED:
        res["paired"].append({"metric": label, "unit": unit, "digits": dg,
                              "before": summary([get(m, *pa) for m in ms]),
                              "after": summary([get(m, *pb) for m in ms])})
    for label, unit, p, none_label, dg in SINGLE:
        res["single"].append({"metric": label, "unit": unit, "digits": dg, "none_label": none_label,
                              "value": summary([get(m, *p) for m in ms])})
    for key, label, unit, dg in (("min_rate_hz", "Log/setpoint rate, slowest agent", "Hz", 2),
                                 ("worst_p99_period_s", "Log period p99, worst agent", "s", 3),
                                 ("fallback_s", "Fallback time, all agents", "s", 1),
                                 ("deaf_s", "Deaf time, all agents", "s", 1)):
        res["single"].append({"metric": label, "unit": unit, "digits": dg, "none_label": "n/a",
                              "value": summary([r[key] for r in res["runs"]])})

    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "results.json"), "w") as f:
        json.dump(res, f, indent=1)
    with open(os.path.join(out_dir, "results.md"), "w") as f:
        f.write(render_md(res))
    return res


def _cell(x, digits=1, none_label="not recovered"):
    return none_label if x is None else (f"{x:.{digits}f}" if isinstance(x, float) else str(x))


def render_md(res):
    L = []
    if res["synthetic"]:
        L += ["> **SYNTHETIC DATA from analysis/synth.py: pipeline check only, NOT measured results.**", ""]
    L += [f"# Graceful degradation: {res['n_runs']} run(s)", "",
          "Cells are median [min, max] across runs. Definitions: analysis/metrics.py docstring.",
          "Before = pre-loss phase [0, kill]; after = post-recovery phase "
          "[kill + max(60 s, recovery b), end].", "",
          "| Metric | Unit | Before the dropout | After recovery |", "|---|---|---|---|"]
    for r in res["paired"]:
        L.append(f"| {r['metric']} | {r['unit']} | {fmt(r['before'], r['digits'])} | {fmt(r['after'], r['digits'])} |")
    L += ["", "| Metric | Unit | Value |", "|---|---|---|"]
    for r in res["single"]:
        L.append(f"| {r['metric']} | {r['unit']} | {fmt(r['value'], r['digits'], r['none_label'])} |")
    L += ["", "## Per run", "",
          "| Run | Seed | Victim | Kill t (s) | Lost declared t (s) | Recovery a (s) | Recovery b (s) | Min sep (m) |",
          "|---|---|---|---|---|---|---|---|"]
    for r in res["runs"]:
        decl = "no event" if r["lost_declared_t_s"] is None else f"{r['lost_declared_t_s']:.1f}"
        sep = "n/a" if r["min_separation_m"] is None else f"{r['min_separation_m']:.1f}"
        warm = " (pre-loss in warm-up)" if r["warmup_incomplete"] else ""
        L.append(f"| {r['run']} | {r['seed']} | {r['victim']} | {_cell(r['t_loss_s'], none_label='n/a')}{warm} | {decl} | "
                 f"{_cell(r['recovery_a_s'])} | {_cell(r['recovery_b_s'])} | {sep} |")
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser(description="aggregate run metrics")
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--out", default="results")
    ap.add_argument("--recompute", action="store_true", help="re-run metrics even if metrics.json exists")
    a = ap.parse_args()
    res = aggregate(a.runs, a.out, a.recompute)
    print(f"{res['n_runs']} runs -> {a.out}/results.md" + (" (SYNTHETIC)" if res["synthetic"] else ""))


if __name__ == "__main__":
    main()
