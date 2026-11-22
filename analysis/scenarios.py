"""One-table summary of the scenario runs (run_scenarios.sh): what each should show, and what
was measured. It does not judge; the reader compares the two columns.

python -m analysis.scenarios RUNS_DIR OUT_DIR
Reads RUNS_DIR/<name>/metrics.json (and meta.json); writes OUT_DIR/summary.md and copies each
metrics.json to OUT_DIR/runs/<name>.json.
"""
import json
import os
import shutil
import sys

from swarm import config

# run name -> expectation. The names are the ones run_scenarios.sh uses.
EXPECTED = {
    "B_middle": "zone owner (agent 2) keeps the middle zone seen within 30 s; the other two "
                "fly 2+2 lane groups; a non-owner is lost: the zone is not affected",
    "B_middle_owner": "the zone owner is lost: the nearest survivor takes the zone lane, the "
                      "other flies all four lanes; the zone exceeds 30 s once, around the loss",
    "B_corner": "as B_middle with the zone in the south-west corner (owner agent 1)",
    "B_corner_owner": "as B_middle_owner with the zone in the corner",
    "C_s1": "10 s blackout: agent not declared lost (12 s rule), no re-plan, it goes deaf "
            "and holds, then resumes",
    "C_s2": "as C_s1, another seed",
    "C_s3": "as C_s1, another seed",
    "blackout13": "13 s blackout: declared lost after 12 s, re-plan; rejoins after 3 s of "
                  "steady contact, second re-plan",
    "mute": "one agent's transmitter dead from 60 s: declared lost after 12 s, re-plan; it "
            "still hears, so it gets an empty plan and holds at its private hold altitude, "
            "out of the cruise layer, to the end",
    "agentdeath": "the agent process is killed, the autopilot lands it in failsafe: declared "
                  "lost after 12 s, re-plan as in Mission A",
    "groundrestart": "ground down 40-55 s: agents keep flying their plans; the new ground "
                     "jumps the epoch, keeps t0; then the Mission A loss is handled",
    "groundloss": "ground down from 40 s for good: agents keep flying their plans; nobody "
                  "re-plans the Mission A loss: the dead agent's lanes stay unflown",
    "radiooutage": "radio down 60-70 s: everyone deaf after 3 s, holds at its hold altitude, "
                   "nobody declared lost (12 s), resumes",
    "obstacle": "an unknown obstacle on a loop slot: one INFEASIBLE accepted after 15 s, one "
                "re-plan of that agent only, no re-issue within the 60 s blacklist",
    "swap": "groups handed out in reverse at 30 s: agents cross; >= 8 m always; a stand-off "
            "is reported by the 15 s slot timer",
    "A_n5": "N = 5: four agents on one lane each, the spare holds at its hold altitude and "
            "takes the lane of the lost agent",
    "A_n4": "N = 4: one lane each; after the loss 1+1+2",
}


def fmt(value, digits=1):
    return "-" if value is None else f"{value:.{digits}f}"


def measured(m, meta):
    """The numbers that matter for every scenario, plus the scenario's own."""
    if meta["outcome"] != "ok":
        return f"INVALID: {meta['outcome']}"
    safety, loss = m["safety"], m["loss"] or {}
    replans = ", ".join(f"{k} {v}" for k, v in m["churn"]["replans_by_trigger"].items())
    warmup = ", ".join(f"{k} {v}" for k, v in m["churn"]["replans_warmup_by_trigger"].items())
    parts = [f"min sep {fmt(safety['min_separation_from_10s_m'])} m from 10 s, "
             f"{fmt(safety['below_d_min_s'])} s < 8 m",
             f"max speed {fmt(m['control']['max_ground_speed_ms'], 2)} m/s (EKF), "
             f"rate {fmt(m['control']['min_rate_hz'])} Hz",
             f"re-plans: {replans or 'none'} (warm-up: {warmup or 'none'}); infeasible "
             f"{safety['infeasible_accepted']} accepted / {safety['infeasible_ignored']} "
             "ignored (warm-up included)",
             f"brake {fmt(safety['brake_s'])} s, deaf {fmt(safety['deaf_s'])} s (summed over "
             "agents)",
             f"coverage mean {fmt(m['coverage']['mean_pct'])} %, longest unseen "
             f"{fmt(m['unseen']['whole']['worst_s'])} s"]
    if loss:
        parts.append(f"victim {m['victim']} at {fmt(loss['loss_s'])} s, declared "
                     f"{fmt(loss['lost_declared_s'])} s; coverage {fmt(loss['before_pct'])} -> "
                     f"after (loss + 60 s to end) {fmt(loss['after_mean_pct'])} % (min "
                     f"{fmt(loss['after_min_pct'])}), dip (loss to end) {fmt(loss['dip_pct'])} "
                     f"%, recovery {fmt(loss['recovery_s'])} s")
    if m["mission_b"]:
        b = m["mission_b"]
        parts.append(f"zone longest unseen {fmt(b['longest_unseen_s'])} s (limit 30), whole "
                     f"zone fresh {fmt(b['whole_zone_fresh_pct'])} % of the time")
    if m["mission_c"]:
        c = m["mission_c"]
        parts.append(f"agent {c['agent']}: declared lost {c['declared_lost']}, re-plans "
                     f"{c['replans_from_blackout']}, deaf {fmt(c['deaf_s'])} s, back in "
                     f"cover {fmt(c['back_in_cover_after_s'])} s after")
    if meta["mission_s"] > config.MISSION_S:
        parts.append(f"run {fmt(meta['mission_s'], 0)} s, longer than {fmt(config.MISSION_S, 0)}")
    return "; ".join(parts)


def main(runs_dir, out_dir):
    os.makedirs(os.path.join(out_dir, "runs"), exist_ok=True)
    lines = ["# Scenario runs, one each", "",
             "Expectation written before the runs; measured from the logs (analysis/metrics.py, "
             "definitions in docs/metrics.md). Max speed here is the autopilot's estimate.", "",
             "| Run | Command | Expected | Measured |", "|---|---|---|---|"]
    for name in EXPECTED:
        run_dir = os.path.join(runs_dir, name)
        if not os.path.exists(os.path.join(run_dir, "meta.json")):
            lines.append(f"| {name} | - | {EXPECTED[name]} | not run |")
            continue
        with open(os.path.join(run_dir, "meta.json")) as f:
            meta = json.load(f)
        m = None
        if meta["outcome"] == "ok":
            with open(os.path.join(run_dir, "metrics.json")) as f:
                m = json.load(f)
            shutil.copy(os.path.join(run_dir, "metrics.json"),
                        os.path.join(out_dir, "runs", name + ".json"))
        command = (f"--mission {meta['mission']} --n {meta['n']} --seed {meta['seed']}"
                   + (f" --scenario {meta['scenario']}" if meta["scenario"] else "")
                   + (f" --zone {','.join(str(v) for v in meta['priority_zone'])}"
                      if meta["priority_zone"] else ""))
        lines.append(f"| {name} (commit {meta['git_commit'][:7]}) | `{command}` | "
                     f"{EXPECTED[name]} | {measured(m, meta)} |")
    with open(os.path.join(out_dir, "summary.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print(os.path.join(out_dir, "summary.md"))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
