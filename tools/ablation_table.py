#!/usr/bin/env python3
"""Pair the no-re-plan runs with the evaluation runs (same seed, victim, kill time) and print
the per-seed numbers as JSON. Reads metrics.json and meta.json only.

  python3 tools/ablation_table.py ~/Documents/Projects/ap-swarm-eval-snapshot/runs 1 6
"""
import json
import statistics
import sys
from pathlib import Path

ROOT = Path(sys.argv[1]).expanduser()
SEEDS = range(int(sys.argv[2]), int(sys.argv[3]) + 1)
SETS = {"with": "gazebo_50m", "without": "gazebo_50m_noreplan"}


def kill(meta):
    """(victim, kill t) from the fault list: the one drone kill."""
    f = [f for f in meta["faults"] if f["target"].startswith("sitl:")]
    assert len(f) == 1, meta["faults"]
    return int(f[0]["target"].split(":")[1]), f[0]["t_mission"]


def row(run):
    m = json.loads((run / "metrics.json").read_text())
    loss = m["loss"]
    return {"after_mean_pct": loss["after_mean_pct"], "after_min_pct": loss["after_min_pct"],
            "dip_pct": loss["dip_pct"], "end_of_run_pct": m["coverage"]["end_of_run_pct"],
            "unseen_after_s": m["unseen"]["after"]["worst_s"],
            "recovered": loss["recovery_s"] is not None}


seeds = []
for s in SEEDS:
    runs = {k: ROOT / v / f"seed_{s}" for k, v in SETS.items()}
    metas = {k: json.loads((r / "meta.json").read_text()) for k, r in runs.items()}
    pair = {k: (m["seed"],) + kill(m) for k, m in metas.items()}
    assert pair["with"] == pair["without"], pair
    assert metas["without"]["faults"][0]["target"] == "ground", metas["without"]["faults"]
    seeds.append({"seed": s, "victim": pair["with"][1], "kill_t_s": pair["with"][2],
                  "ground_killed_t_s": metas["without"]["faults"][0]["t_mission"],
                  **{k: row(r) for k, r in runs.items()}})

summary = {}
for k in SETS:
    summary[k] = {key: {"median": statistics.median(v), "min": min(v), "max": max(v)}
                  for key in seeds[0][k] if key != "recovered"
                  for v in [[x[k][key] for x in seeds]]}
    summary[k]["recovered_runs"] = sum(x[k]["recovered"] for x in seeds)
print(json.dumps({"sets": SETS, "seeds": seeds, "summary": summary, "n": len(seeds)}, indent=1))
