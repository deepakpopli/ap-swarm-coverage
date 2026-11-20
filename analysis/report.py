"""Results of a set of runs (results.md, results.json, runs/*.json) and comparison of sets.

python -m analysis.report --out results/<set> --name "<label>" RUN_DIR...
                          [--ref RUN_DIR...] [--baseline RUN_DIR...]
python -m analysis.report --compare results/a results/b ...

--ref: no-loss runs with N-1 agents (the attainable coverage of the survivors).
--baseline: no-loss runs with N agents. Their mean coverage is the fixed level that recovery and
deficit are also measured against (besides each run's own P, which flatters a low start).
Every cell is median [min, max] over the runs. Definitions: docs/metrics.md.
"""
import argparse
import json
import os
import shutil

import numpy as np

from analysis import metrics

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

METRICS = [   # (key = path in metrics.json, label, unit, digits)
    # the task's "worst revisit gap": longest time any cell went unseen (docs/metrics.md)
    ("unseen.whole.worst_s", "Worst revisit gap (longest unseen), whole mission [0, end]",
     "s", 1),
    ("unseen.before.worst_s", "Worst revisit gap (longest unseen), before the loss [0, L)",
     "s", 1),
    ("unseen.after.worst_s", "Worst revisit gap (longest unseen), from the loss [L, end]",
     "s", 1),
    ("unseen.whole.cells_over_60s_pct", "Cells unseen > 60 s at some time, whole mission",
     "%", 1),
    ("unseen.whole.cells_over_90s_pct", "Cells unseen > 90 s at some time, whole mission",
     "%", 1),
    ("unseen.after.cells_over_60s_pct", "Cells unseen > 60 s, gap ending from the loss on",
     "%", 1),
    ("unseen.after.cells_over_90s_pct", "Cells unseen > 90 s, gap ending from the loss on",
     "%", 1),
    ("coverage.mean_pct", "Coverage over the mission, mean", "%", 1),
    ("loss.before_pct", "Coverage before the loss, mean (P)", "%", 1),
    ("loss.before_min_pct", "Coverage before the loss, min", "%", 1),
    ("loss.after_mean_pct", "Coverage after the loss (from L + 60 s), mean", "%", 1),
    ("loss.after_min_pct", "Coverage after the loss (from L + 60 s), min", "%", 1),
    ("coverage.end_of_run_pct", "Coverage at the end of the run (last 30 s), mean", "%", 1),
    ("coverage.end_of_run_min_pct", "Coverage at the end of the run (last 30 s), min", "%", 1),
    ("loss.share_pct", "(N-1)/N x P", "%", 1),
    ("attainable_pct", "Attainable with N-1: no-loss N-1 runs, mean", "%", 1),
    ("baseline_pct", "No-loss N runs, mean", "%", 1),
    ("loss.after_over_share", "After / ((N-1)/N x P)", "ratio", 2),
    ("loss.after_over_attainable", "After / attainable with N-1", "ratio", 2),
    ("loss.recovery_s", "Recovery to 0.9 x P", "s", 1),
    ("loss.recovery_to_share_s", "Recovery to 0.9 x (N-1)/N x P (weaker reading)", "s", 1),
    ("loss.vs_baseline.recovery_s", "Recovery to 0.9 x no-loss N baseline", "s", 1),
    ("loss.dip_pct", "Dip: lowest coverage after the loss", "%", 1),
    ("loss.dip_after_loss_s", "Dip: time after the loss", "s", 1),
    ("loss.deficit_pct_s", "Coverage deficit after the loss, against P", "%.s", 0),
    ("loss.vs_baseline.deficit_pct_s", "Coverage deficit after the loss, against the no-loss N "
     "baseline", "%.s", 0),
    ("loss.milestones_s.lost_declared", "Loss -> declared lost", "s", 1),
    ("loss.milestones_s.assignment_received", "Loss -> new assignment held by every survivor",
     "s", 1),
    ("loss.milestones_s.on_new_loop", "Loss -> first survivor on its new loop", "s", 1),
    ("revisit.before.worst_completed_s",
     "Steady revisit gap (both sightings inside the phase), before [0, L)", "s", 1),
    ("revisit.after.worst_completed_s",
     "Steady revisit gap (both sightings inside the phase), after [R, end]", "s", 1),
    ("revisit.before.worst_open_s", "Largest open gap at L", "s", 1),
    ("revisit.after.worst_open_s", "Largest open gap at the end", "s", 1),
    ("revisit.before.no_completed_gap_pct", "Cells with no completed gap, before", "%", 1),
    ("revisit.after.no_completed_gap_pct", "Cells with no completed gap, after", "%", 1),
    ("churn.before_per_agent_min", "Churn before the loss", "/agent/min", 2),
    ("churn.after_per_agent_min", "Churn from the loss", "/agent/min", 2),
    ("churn.whole_per_agent_min", "Churn, whole mission", "/agent/min", 2),
    ("churn.extra_changes_after_loss", "Changes beyond the one forced by the loss", "count", 0),
    ("safety.min_separation_m", "Min 3D separation, whole flight", "m", 1),
    ("safety.min_separation_from_10s_m", "Min 3D separation, from t = 10 s", "m", 1),
    ("safety.below_d_min_s", "Time below 8 m", "s", 1),
    ("safety.brake_s", "Time braking, all agents", "s", 1),
    ("safety.deaf_s", "Time deaf, all agents", "s", 1),
    ("safety.infeasible_accepted", "Infeasible reports accepted", "count", 0),
    ("safety.infeasible_ignored", "Infeasible reports ignored", "count", 0),
    ("control.min_rate_hz", "Setpoint rate, slowest agent", "Hz", 1),
    ("control.max_p99_period_s", "Setpoint period p99, worst agent", "s", 3),
    ("control.max_period_s", "Setpoint period max, worst agent", "s", 3),
    ("control.max_ground_speed_ms", "Max ground speed", "m/s", 2),
    ("radio.loss_pct", "Radio: loss rate", "%", 1),
    ("radio.delay_mean_s", "Radio: mean delay", "s", 3),
    ("radio.delay_max_s", "Radio: max delay", "s", 3),
    ("mission_b.longest_unseen_s", "Priority zone: longest unseen (limit 30 s)", "s", 1),
    ("mission_b.whole_zone_fresh_pct", "Priority zone: time all seen within 30 s", "%", 1),
    ("mission_c.declared_lost", "Blackout agent declared lost (1 = yes)", "", 0),
    ("mission_c.replans_from_blackout", "Re-plans from the blackout start", "count", 0),
    ("mission_c.deaf_s", "Blackout agent: time deaf", "s", 1),
    ("mission_c.back_in_cover_after_s", "Blackout end -> back in cover", "s", 1),
]
LABELS = {key: (label, unit, digits) for key, label, unit, digits in METRICS}
PHASE_TABLE_KEYS = {"loss.before_pct", "loss.before_min_pct", "loss.after_mean_pct",
                    "loss.after_min_pct", "coverage.end_of_run_pct", "coverage.end_of_run_min_pct",
                    "loss.share_pct", "attainable_pct", "loss.recovery_s",
                    "loss.recovery_to_share_s", "loss.vs_baseline.recovery_s",
                    "unseen.before.worst_s", "unseen.after.worst_s",
                    "revisit.before.worst_completed_s",
                    "revisit.after.worst_completed_s", "churn.before_per_agent_min",
                    "churn.after_per_agent_min"}


def refuse_fake_in_results(out_path, backends):
    """results/ holds measurements only; the kinematic fake vehicle is not one."""
    inside = os.path.abspath(out_path).startswith(os.path.join(REPO, "results") + os.sep)
    if inside and "fake" in backends:
        raise SystemExit("refusing to write runs with backend 'fake' under results/")


def lookup(m, key):
    """Value at a dotted path in a metrics dict, or None."""
    value = m
    for part in key.split("."):
        if value is None:
            return None
        value = value.get(part)
    return value


def summarise(values):
    present = [float(v) for v in values if v is not None]
    out = {"n": len(present), "n_none": len(values) - len(present)}
    if present:
        out.update(median=float(np.median(present)), min=min(present), max=max(present))
    return out


def cell(summary, digits):
    if summary["n"] == 0:
        return "-"
    return (f"{summary['median']:.{digits}f} "
            f"[{summary['min']:.{digits}f}, {summary['max']:.{digits}f}]")


def recovered_text(summary, digits=1):
    """"k of n recovered; median over those" - runs that never recovered are counted, not hidden."""
    k = summary["n"]
    text = f"{k} of {k + summary['n_none']} runs recovered"
    if k:
        text += f"; over the recovered runs: {cell(summary, digits)} s"
    return text


def load_metrics(run_dir):
    path = os.path.join(run_dir, "metrics.json")
    if not os.path.exists(path):
        metrics.analyse(run_dir)
    with open(path) as f:
        m = json.load(f)
    m["dir"] = run_dir
    return m


def summarise_set(runs, refs, baselines):
    """{key: summary} for every key in METRICS, over the runs. Needs coverage.csv beside each
    run's metrics.json when baselines are given (m["dir"])."""
    if baselines:
        level = float(np.median([m["coverage"]["mean_pct"] for m in baselines]))
        for m in runs:
            if m["loss"] is not None:
                ticks, coverage = metrics.read_coverage_csv(m["dir"])
                m["loss"]["vs_baseline"] = metrics.against_level(ticks, coverage,
                                                                 m["loss"]["loss_s"], level)
    attainable = summarise([m["coverage"]["mean_pct"] for m in refs])
    for m in runs:
        if m["loss"] is not None and attainable["n"]:
            m["loss"]["after_over_attainable"] = m["loss"]["after_mean_pct"] / attainable["median"]
    summary = {key: summarise([lookup(m, key) for m in runs]) for key, *_ in METRICS}
    summary["attainable_pct"] = attainable
    summary["baseline_pct"] = summarise([m["coverage"]["mean_pct"] for m in baselines])
    return summary


def phase_table(s):
    def c(key):
        return cell(s[key], LABELS[key][2])

    head = ["| Metric | Before the loss | After the loss (from L + 60 s) | End of run (last 30 s) "
            "| Attainable with N-1 | (N-1)/N x before | n |", "|---|---|---|---|---|---|---|"]
    rows = [
        f"| Worst revisit gap (longest unseen) (s) | {c('unseen.before.worst_s')} | "
        f"{c('unseen.after.worst_s')} | - | - | - | {s['unseen.after.worst_s']['n']} |",
        f"| Coverage, mean (%) | {c('loss.before_pct')} | {c('loss.after_mean_pct')} | "
        f"{c('coverage.end_of_run_pct')} | {c('attainable_pct')} | {c('loss.share_pct')} | "
        f"{s['loss.before_pct']['n']} |",
        f"| Coverage, min (%) | {c('loss.before_min_pct')} | {c('loss.after_min_pct')} | "
        f"{c('coverage.end_of_run_min_pct')} | - | - | {s['loss.before_min_pct']['n']} |",
        f"| Steady revisit gap (both sightings inside the phase) (s) | "
        f"{c('revisit.before.worst_completed_s')} | "
        f"{c('revisit.after.worst_completed_s')} | - | - | - | "
        f"{s['revisit.after.worst_completed_s']['n']} |",
        f"| Assignment churn (/agent/min) | {c('churn.before_per_agent_min')} | "
        f"{c('churn.after_per_agent_min')} | - | - | - | {s['churn.after_per_agent_min']['n']} |"]
    notes = ["", "Revisit gap phases differ from the coverage phases. Longest unseen: before = "
             "[0, L), after = [L, end]; a gap belongs to the phase where it ends, a gap still "
             "open at the end counts as its age then. Steady gap: before = [0, L), after = "
             "[R, end] with R = lost declared, survivors only.", "",
             f"- Recovery to 0.9 x P: {recovered_text(s['loss.recovery_s'])}.",
             f"- Recovery to 0.9 x the no-loss N baseline ({c('baseline_pct')} %): "
             f"{recovered_text(s['loss.vs_baseline.recovery_s'])}.",
             f"- Recovery to 0.9 x (N-1)/N x P (weaker reading): "
             f"{recovered_text(s['loss.recovery_to_share_s'])}."]
    return head + rows + notes


def per_run_table(runs):
    def number(value, digits=1):
        return "-" if value is None else f"{value:.{digits}f}"

    lines = ["| Run | Seed | Victim | Kill t (s) | Lost declared t (s) | Recovery (s) "
             "| After, mean (%) | Longest unseen, whole (s) | Min separation from 10 s (m) |",
             "|---" * 9 + "|"]
    for m in runs:
        loss = m["loss"] or {}
        recovery = number(loss.get("recovery_s"))
        if m["loss"] and loss["recovery_s"] is None:
            recovery = "not recovered"
        lines.append(f"| {m['run']} | {m['seed']} | {m['victim']} | {number(loss.get('loss_s'))} "
                     f"| {number(loss.get('lost_declared_s'))} | {recovery} "
                     f"| {number(loss.get('after_mean_pct'))} "
                     f"| {number((m.get('unseen') or {}).get('whole', {}).get('worst_s'))} "
                     f"| {number(m['safety']['min_separation_from_10s_m'])} |")
    return lines


def render(name, runs, summary):
    backends = sorted({m["backend"] for m in runs})
    lines = [f"# {name}", "",
             f"{len(runs)} runs, backend {', '.join(backends)}, N = {runs[0]['n']}. "
             "Cells: median [min, max] over runs. Definitions: docs/metrics.md.", ""]
    if summary["loss.before_pct"]["n"]:
        lines += phase_table(summary) + [""]
    lines += ["| Metric | Unit | Median [min, max] | n |", "|---|---|---|---|"]
    for key, label, unit, digits in METRICS:
        if key not in PHASE_TABLE_KEYS and summary[key]["n"]:
            lines.append(f"| {label} | {unit} | {cell(summary[key], digits)} "
                         f"| {summary[key]['n']} |")
    triggers = {}
    for m in runs:
        for trigger, count in m["churn"]["replans_by_trigger"].items():
            triggers[trigger] = triggers.get(trigger, 0) + count
    mismatched = [m["run"] for m in runs if m["churn"]["epoch_mismatch"]]
    lines += ["", "Re-plans by trigger, all runs: "
              + (", ".join(f"{k} {v}" for k, v in sorted(triggers.items())) or "none") + ".",
              "Agents' epoch columns vs ground re-plans: "
              + (f"MISMATCH in {', '.join(mismatched)}" if mismatched else "no mismatch") + ".",
              "", "## Per run", ""] + per_run_table(runs)
    return "\n".join(lines) + "\n"


def write_set(out_dir, name, run_dirs, ref_dirs, baseline_dirs):
    runs = [load_metrics(d) for d in run_dirs]
    refs = [load_metrics(d) for d in ref_dirs]
    baselines = [load_metrics(d) for d in baseline_dirs]
    refuse_fake_in_results(out_dir, {m["backend"] for m in runs + refs + baselines})
    summary = summarise_set(runs, refs, baselines)
    os.makedirs(os.path.join(out_dir, "runs"), exist_ok=True)
    for prefix, dirs in (("", run_dirs), ("ref_", ref_dirs), ("baseline_", baseline_dirs)):
        for d in dirs:
            copy_name = prefix + os.path.basename(os.path.normpath(d)) + ".json"
            shutil.copy(os.path.join(d, "metrics.json"), os.path.join(out_dir, "runs", copy_name))
    with open(os.path.join(out_dir, "results.json"), "w") as f:
        json.dump({"name": name, "runs": [m["run"] for m in runs], "summary": summary}, f, indent=1)
    with open(os.path.join(out_dir, "results.md"), "w") as f:
        f.write(render(name, runs, summary))


def runs_with_value(summary):
    """" (n)", or " (k of n)" when some runs have no value (e.g. did not recover)."""
    if summary["n"] == 0:
        return ""
    if summary.get("n_none"):
        return f" ({summary['n']} of {summary['n'] + summary['n_none']})"
    return f" ({summary['n']})"


def write_comparison(set_dirs):
    """results/comparison.md: the sets side by side, matched by metric key."""
    sets = []
    for d in set_dirs:
        with open(os.path.join(d, "results.json")) as f:
            sets.append(json.load(f))
    lines = ["# Comparison", "", "Cells: median [min, max] (n runs). Definitions: docs/metrics.md.",
             "", "| Metric | Unit | " + " | ".join(s["name"] for s in sets) + " |",
             "|---|---|" + "---|" * len(sets)]
    for key, label, unit, digits in METRICS:
        summaries = [s["summary"].get(key, {"n": 0}) for s in sets]
        if any(x["n"] for x in summaries):
            cells = [cell(x, digits) + runs_with_value(x) for x in summaries]
            lines.append(f"| {label} | {unit} | " + " | ".join(cells) + " |")
    out = os.path.join(os.path.dirname(os.path.abspath(set_dirs[0])), "comparison.md")
    with open(out, "w") as f:
        f.write("\n".join(lines) + "\n")
    return out


def main():
    parser = argparse.ArgumentParser(description="results table of a set of runs")
    parser.add_argument("runs", nargs="*")
    parser.add_argument("--out")
    parser.add_argument("--name")
    parser.add_argument("--ref", nargs="*", default=[])
    parser.add_argument("--baseline", nargs="*", default=[])
    parser.add_argument("--compare", nargs="+")
    args = parser.parse_args()
    if args.compare:
        print(write_comparison(args.compare))
    else:
        write_set(args.out, args.name, args.runs, args.ref, args.baseline)
        print(os.path.join(args.out, "results.md"))


if __name__ == "__main__":
    main()
