"""Results of a set of runs (results.md, results.json, runs/*.json).

python -m analysis.report --out results/<set> --name "<label>" RUN_DIR...
                          [--ref RUN_DIR...] [--baseline RUN_DIR...]

--ref: no-loss runs with N-1 agents (the attainable coverage of the survivors).
--baseline: no-loss runs with N agents. Their mean coverage is the fixed level that recovery and
deficit are also measured against (besides each run's own P, which flatters a low start).
Every cell is median [min, max] over the runs; a reference set of one run prints its single
value. Definitions: how_to_read() and analysis/metrics.py.
"""
import argparse
import csv
import glob
import json
import os
import shutil

import numpy as np

from analysis import metrics

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

METRICS = [   # (key = path in metrics.json, label, unit, digits)
    # the task's "worst revisit gap": longest time any cell went unseen
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
    ("loss.after_mean_pct", "Coverage from L + 60 s, mean", "%", 1),
    ("loss.after_min_pct", "Coverage from L + 60 s, min", "%", 1),
    ("extra.after_recovery_mean_pct", "Coverage after recovery [L + recovery, end], mean", "%",
     1),
    ("extra.after_recovery_min_pct", "Coverage after recovery [L + recovery, end], min", "%", 1),
    ("coverage.end_of_run_pct", "Coverage at the end of the run (last 30 s), mean", "%", 1),
    ("coverage.end_of_run_min_pct", "Coverage at the end of the run (last 30 s), min", "%", 1),
    ("loss.share_pct", "(N-1)/N x P", "%", 1),
    ("attainable_pct", "Attainable with N-1: no-loss N-1 runs, mean", "%", 1),
    ("baseline_pct", "No-loss N runs, mean", "%", 1),
    ("loss.after_over_share", "After / ((N-1)/N x P)", "ratio", 2),
    ("loss.after_over_attainable", "After / attainable with N-1", "ratio", 2),
    ("loss.recovery_s", "Recovery to 0.9 x P (0 = never fell below)", "s", 1),
    ("extra.recovery_dipped_s", "Recovery to 0.9 x P, runs that fell below only", "s", 1),
    ("extra.baseline_recovery_dipped_s",
     "Recovery to 0.9 x no-loss N baseline, runs that fell below only", "s", 1),
    ("loss.recovery_to_share_s", "Recovery to 0.9 x (N-1)/N x P (weaker reading)", "s", 1),
    ("loss.vs_baseline.recovery_s", "Recovery to 0.9 x no-loss N baseline", "s", 1),
    ("loss.dip_pct", "Dip: lowest coverage from L on", "%", 1),
    ("loss.dip_after_loss_s", "Dip: when, after L", "s", 1),
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
    ("control.max_ground_speed_ms",
     "Max ground speed, logged velocity (vx, vy; autopilot EKF estimate)", "m/s", 2),
    ("extra.max_speed_position_ms",
     "Max ground speed, position differences over 1 s (EKF positions)", "m/s", 2),
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
                    "loss.after_min_pct", "extra.after_recovery_mean_pct",
                    "extra.after_recovery_min_pct", "loss.dip_pct", "loss.dip_after_loss_s",
                    "extra.recovery_dipped_s", "extra.baseline_recovery_dipped_s",
                    "baseline_pct", "coverage.end_of_run_pct", "coverage.end_of_run_min_pct",
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


# ------------------------------------------------------- fields metrics.json does not hold

POSITION_STEP_S = 1.0   # speed check against the logged velocity: distance flown over 1 s


def read_positions(path, t0_wall):
    """(t mission s, x m, y m) of one agent log. A SIGKILL can cut the last line: dropped."""
    with open(path) as f:
        rows = list(csv.DictReader(f.read().split("\n")[:-1]))

    def column(name):
        return np.array([float(r[name]) if r[name] else np.nan for r in rows])

    return column("t_wall") - t0_wall, column("x"), column("y")


def max_speed_over_step(t_s, x_m, y_m, step_s=POSITION_STEP_S):
    """Largest distance between p(t) and p(t + step), / step; p(t + step) interpolated."""
    starts = t_s + step_s <= t_s[-1] if len(t_s) else np.zeros(0, dtype=bool)
    if not starts.any():
        return None
    later_x = np.interp(t_s[starts] + step_s, t_s, x_m)
    later_y = np.interp(t_s[starts] + step_s, t_s, y_m)
    return float(np.nanmax(np.hypot(later_x - x_m[starts], later_y - y_m[starts])) / step_s)


def position_speed_ms(run_dir, meta):
    """Max ground speed from EKF position differences, over the rows metrics.py counts:
    [0, end], the victim until the loss."""
    loss_s = None if meta["kill_agent"] is None else meta["kill_wall"] - meta["t0_wall"]
    speeds = []
    for path in sorted(glob.glob(os.path.join(run_dir, "agent_*.csv"))):
        agent_id = int(os.path.basename(path)[len("agent_"):-len(".csv")])
        t, x, y = read_positions(path, meta["t0_wall"])
        until = loss_s if agent_id == meta["kill_agent"] else meta["mission_s"]
        rows = (t >= 0.0) & (t <= until)
        speeds.append(max_speed_over_step(t[rows], x[rows], y[rows]))
    speeds = [s for s in speeds if s is not None]
    return max(speeds) if speeds else None


def first_plan_lanes(ground_records, agent_id):
    """East positions (m) of the agent's loop lanes in the first plan, or None."""
    first = next((e for e in ground_records if e["event"] == "assign"), None)
    if first is None or str(agent_id) not in first["assignments"]:
        return None
    plan = first["assignments"][str(agent_id)]
    return sorted({round(x, 1) for x, _ in plan["slots"][plan["loop_from"]:]})


def coverage_after(ticks, coverage, from_s):
    """(mean, min) of coverage(t) for t >= from_s, or (None, None)."""
    if from_s is None or not np.any(ticks >= from_s):
        return None, None
    window = coverage[ticks >= from_s]
    return float(np.mean(window)), float(np.min(window))


def recovery_if_dipped(recovery_s, dipped):
    """The recovery time of a run that fell below the threshold; None for one that never did."""
    return recovery_s if dipped else None


def add_extras(m, baseline_level_pct=None):
    """m["extra"]: after-recovery coverage, whether the dip crossed each threshold, the victim's
    first-plan lanes, the position-difference speed. Reads the run's files beside metrics.json."""
    run_dir = m["dir"]
    with open(os.path.join(run_dir, "meta.json")) as f:
        meta = json.load(f)
    extra = {"max_speed_position_ms": position_speed_ms(run_dir, meta)}
    loss = m["loss"]
    if loss is not None:
        ticks, coverage = metrics.read_coverage_csv(run_dir)
        recovery_s = loss["recovery_s"]
        from_s = None if recovery_s is None else loss["loss_s"] + recovery_s
        mean_pct, min_pct = coverage_after(ticks, coverage, from_s)
        dipped = loss["dip_pct"] < loss["recovery_threshold_pct"]
        with open(os.path.join(run_dir, "ground.jsonl")) as f:
            ground = [json.loads(line) for line in f if line.strip()]
        extra.update(after_recovery_from_s=from_s, after_recovery_mean_pct=mean_pct,
                     after_recovery_min_pct=min_pct, dipped_below_0_9p=dipped,
                     recovery_dipped_s=recovery_if_dipped(recovery_s, dipped),
                     victim_lanes_x_m=first_plan_lanes(ground, m["victim"]))
        if baseline_level_pct is not None and "vs_baseline" in loss:
            below = loss["dip_pct"] < metrics.RECOVERED * baseline_level_pct
            extra["dipped_below_0_9b"] = below
            extra["baseline_recovery_dipped_s"] = recovery_if_dipped(
                loss["vs_baseline"]["recovery_s"], below)
    m["extra"] = extra
    return m


def summarise_set(runs, refs, baselines):
    """{key: summary} for every key in METRICS, over the runs. Needs the run files beside each
    run's metrics.json (m["dir"])."""
    level = None
    if baselines:
        level = float(np.median([m["coverage"]["mean_pct"] for m in baselines]))
        for m in runs:
            if m["loss"] is not None:
                ticks, coverage = metrics.read_coverage_csv(m["dir"])
                m["loss"]["vs_baseline"] = metrics.against_level(ticks, coverage,
                                                                 m["loss"]["loss_s"], level)
    for m in runs:
        add_extras(m, level)
    attainable = summarise([m["coverage"]["mean_pct"] for m in refs])
    for m in runs:
        if m["loss"] is not None and attainable["n"]:
            m["loss"]["after_over_attainable"] = m["loss"]["after_mean_pct"] / attainable["median"]
    summary = {key: summarise([lookup(m, key) for m in runs]) for key, *_ in METRICS}
    summary["attainable_pct"] = attainable
    summary["baseline_pct"] = summarise([m["coverage"]["mean_pct"] for m in baselines])
    summary["dipped_runs"] = sum(1 for m in runs if lookup(m, "extra.dipped_below_0_9p"))
    summary["dipped_runs_baseline"] = sum(1 for m in runs
                                          if lookup(m, "extra.dipped_below_0_9b"))
    return summary


# -------------------------------------------------------------------------- results.md

def reference_cell(summary, digits=1, unit=""):
    """A reference level: one run is a single value, not "x [x, x]"."""
    if summary["n"] == 0:
        return "-"
    if summary["n"] == 1:
        return f"{summary['median']:.{digits}f}{unit} (1 run)"
    return f"{cell(summary, digits)}{unit} ({summary['n']} runs)"


def how_to_read(s, refs, baselines):
    share = s["loss.share_pct"]
    dip = s["loss.dip_pct"]
    lines = ["## How to read this", "",
             "- Mission t = 0 is 60 s after every drone is on its coverage loop, so the 60 s "
             "coverage window is already full at t = 0 (no start-up ramp).",
             "- The agent is killed at a seeded time L in [60, 90] s mission time; the mission "
             "ends at 180 s.",
             "- coverage(t) = % of 2 m cells seen in (t - 60 s, t], every 1 s. Windows: "
             "before = [L - 60 s, L); dip = lowest coverage(t) on [L, end]; from L + 60 s = "
             "[L + 60 s, end] (the victim's last sightings have left the window); after "
             "recovery = [L + recovery, end] (a run that never fell below 0.9 P counts as "
             "recovered at L); end of run = the last 30 s; mission mean and the no-loss "
             "references = [0, end].",
             "- Longest unseen: before = [0, L), from L on = [L, end] (a gap counts where it "
             "ends). Steady revisit gap: before = [0, L), from L on = [R, end], R = declared "
             "lost. Churn: [0, L) and [L, end].",
             "- Recovery = first time from which coverage stays >= 0.9 x P to the end. 0.0 s "
             "means it never fell below 0.9 x P, not an instant recovery."]
    if share["n"] and dip["n"]:
        lines.append(
            f"- The task's (N-1)/N x P check (about {share['max']:.0f} %) is passed trivially "
            f"at this altitude: the lowest dip of any run is {dip['min']:.1f} %. The stronger "
            "comparison is against the no-loss N-1 reference (\"attainable with N-1\").")
    names = ", ".join(os.path.basename(os.path.normpath(m["dir"])) for m in baselines + refs)
    lines.append(f"- Reference runs used: {names or 'none'}.")
    return lines + [""]


def recovery_line(label, all_runs, dipped, dipped_count):
    """Separates the runs that never fell below (0.0 s) from those that did."""
    total = all_runs["n"] + all_runs["n_none"]
    text = (f"- {label}: {total - dipped_count} of {total} runs never fell below it "
            f"(recovery 0.0 s); {dipped_count} fell below")
    if dipped_count:
        back = dipped["n"]
        text += f", {back} of them back within the run"
        if back:
            text += f", over those {back}: {cell(dipped, 1)} s"
    return text + "."


def phase_table(s):
    def c(key):
        return cell(s[key], LABELS[key][2])

    head = ["| Metric | Before [L - 60 s, L) | From L on [L, end] | From L + 60 s "
            "| After recovery [L + rec, end] | End of run (last 30 s) "
            "| Attainable with N-1 (no-loss ref) | (N-1)/N x P | n |", "|---" * 9 + "|"]
    n = s["loss.before_pct"]["n"]
    rows = [
        f"| Coverage, mean (%) | {c('loss.before_pct')} | - | {c('loss.after_mean_pct')} "
        f"| {c('extra.after_recovery_mean_pct')} | {c('coverage.end_of_run_pct')} "
        f"| {reference_cell(s['attainable_pct'])} | {c('loss.share_pct')} | {n} |",
        f"| Coverage, min (%); from L on: the dip | {c('loss.before_min_pct')} "
        f"| {c('loss.dip_pct')} | {c('loss.after_min_pct')} "
        f"| {c('extra.after_recovery_min_pct')} | {c('coverage.end_of_run_min_pct')} "
        f"| - | - | {n} |",
        f"| Dip: when, after L (s) | - | {c('loss.dip_after_loss_s')} | - | - | - | - | - "
        f"| {n} |",
        f"| Worst revisit gap (longest unseen) (s) | {c('unseen.before.worst_s')} "
        f"| {c('unseen.after.worst_s')} | - | - | - | - | - | {s['unseen.after.worst_s']['n']} |",
        f"| Steady revisit gap (s); from R | {c('revisit.before.worst_completed_s')} "
        f"| {c('revisit.after.worst_completed_s')} | - | - | - | - | - "
        f"| {s['revisit.after.worst_completed_s']['n']} |",
        f"| Assignment churn (/agent/min) | {c('churn.before_per_agent_min')} "
        f"| {c('churn.after_per_agent_min')} | - | - | - | - | - "
        f"| {s['churn.after_per_agent_min']['n']} |"]
    notes = ["", "- No-loss N baseline (B), mean coverage: "
             f"{reference_cell(s['baseline_pct'], unit=' %')}.",
             f"- No-loss N-1 reference (attainable with N-1), mean coverage: "
             f"{reference_cell(s['attainable_pct'], unit=' %')}.",
             recovery_line("Recovery to 0.9 x P", s["loss.recovery_s"],
                           s["extra.recovery_dipped_s"], s["dipped_runs"])]
    if s["baseline_pct"]["n"]:
        notes.append(recovery_line("Recovery to 0.9 x B", s["loss.vs_baseline.recovery_s"],
                                   s["extra.baseline_recovery_dipped_s"],
                                   s["dipped_runs_baseline"]))
    notes.append(f"- Recovery to 0.9 x (N-1)/N x P (weaker reading): "
                 f"{recovered_text(s['loss.recovery_to_share_s'])}.")
    return head + rows + notes


def victim_groups(runs):
    groups = {}
    for m in runs:
        if m["loss"] is not None:
            groups.setdefault(m["victim"], []).append(m)
    return dict(sorted(groups.items()))


def mean_cell(values, digits=1):
    values = [v for v in values if v is not None]
    if not values:
        return "-"
    return f"{np.mean(values):.{digits}f} [{min(values):.{digits}f}, {max(values):.{digits}f}]"


def lanes_text(group):
    lane_sets = sorted({tuple(m["extra"]["victim_lanes_x_m"] or ()) for m in group})
    return "; ".join(f"{len(lanes)} (x = {', '.join(f'{x:g}' for x in lanes)} m)"
                     for lanes in lane_sets)


def victim_recovery_text(group):
    """Never-below count, then the recovery times of the runs that fell below."""
    dipped = [m for m in group if m["extra"]["dipped_below_0_9p"]]
    times = ", ".join("not back" if m["loss"]["recovery_s"] is None
                      else f"{m['loss']['recovery_s']:.1f}" for m in dipped)
    text = f"{len(group) - len(dipped)} never below"
    return text + (f"; {len(dipped)} below: {times} s" if dipped else "")


def per_victim_table(runs):
    lines = ["## By victim", "",
             "Cells: mean [min, max] over that victim's runs. Lanes: the victim's loop lanes "
             "in the first plan.", "",
             "| Victim | Lanes before the loss | n runs | Coverage from L + 60 s, mean (%) "
             "| Recovery to 0.9 x P | Dip (%) | Longest unseen, whole (s) |", "|---" * 7 + "|"]
    for victim, group in victim_groups(runs).items():
        lines.append(
            f"| {victim} | {lanes_text(group)} | {len(group)} "
            f"| {mean_cell([m['loss']['after_mean_pct'] for m in group])} "
            f"| {victim_recovery_text(group)} "
            f"| {mean_cell([m['loss']['dip_pct'] for m in group])} "
            f"| {mean_cell([lookup(m, 'unseen.whole.worst_s') for m in group])} |")
    return lines


def per_victim_json(runs):
    return {str(victim): {"lanes_x_m": [m["extra"]["victim_lanes_x_m"] for m in group],
                          "runs": [m["run"] for m in group],
                          "after_mean_pct": [m["loss"]["after_mean_pct"] for m in group],
                          "recovery_s": [m["loss"]["recovery_s"] for m in group],
                          "dipped_below_0_9p": [m["extra"]["dipped_below_0_9p"] for m in group],
                          "dip_pct": [m["loss"]["dip_pct"] for m in group],
                          "longest_unseen_whole_s": [lookup(m, "unseen.whole.worst_s")
                                                     for m in group]}
            for victim, group in victim_groups(runs).items()}


def per_run_table(runs):
    def number(value, digits=1):
        return "-" if value is None else f"{value:.{digits}f}"

    lines = ["| Run | Seed | Victim | Kill t (s) | Lost declared t (s) | Recovery (s) "
             "| Dip (%) | Dip at, after L (s) | From L + 60 s, mean (%) "
             "| After recovery, mean (%) | Longest unseen, whole (s) "
             "| Min separation from 10 s (m) |", "|---" * 12 + "|"]
    for m in runs:
        loss = m["loss"] or {}
        extra = m.get("extra") or {}
        recovery = number(loss.get("recovery_s"))
        if m["loss"] and loss["recovery_s"] is None:
            recovery = "not recovered"
        elif m["loss"] and not extra.get("dipped_below_0_9p"):
            recovery = "0.0 (never below)"
        lines.append(f"| {m['run']} | {m['seed']} | {m['victim']} | {number(loss.get('loss_s'))} "
                     f"| {number(loss.get('lost_declared_s'))} | {recovery} "
                     f"| {number(loss.get('dip_pct'))} | {number(loss.get('dip_after_loss_s'))} "
                     f"| {number(loss.get('after_mean_pct'))} "
                     f"| {number(extra.get('after_recovery_mean_pct'))} "
                     f"| {number((m.get('unseen') or {}).get('whole', {}).get('worst_s'))} "
                     f"| {number(m['safety']['min_separation_from_10s_m'])} |")
    return lines


def render(name, runs, summary, refs=(), baselines=()):
    backends = sorted({m["backend"] for m in runs})
    has_loss = bool(summary["loss.before_pct"]["n"])
    lines = [f"# {name}", "",
             f"{len(runs)} runs, backend {', '.join(backends)}, N = {runs[0]['n']}. "
             "Cells: median [min, max] over runs; a reference of one run is a single value. "
             "Definitions: " + ("\"How to read this\" below and " if has_loss else "")
             + "analysis/metrics.py.", ""]
    commits = sorted({m.get("git_commit") or "not recorded" for m in runs + list(refs)
                      + list(baselines)})
    short = [c[:12] + ("-dirty" if c.endswith("-dirty") else "") for c in commits]
    lines += [f"Code: commit {', '.join(short)} (from each run's meta.json).", ""]
    if has_loss:
        lines += how_to_read(summary, refs, baselines) + phase_table(summary) + [""]
        lines += per_victim_table(runs) + [""]
    lines += ["## All metrics", "", "| Metric | Unit | Median [min, max] | n |",
              "|---|---|---|---|"]
    for key, label, unit, digits in METRICS:
        if key not in PHASE_TABLE_KEYS and summary[key]["n"]:
            lines.append(f"| {label} | {unit} | {cell(summary[key], digits)} "
                         f"| {summary[key]['n']} |")
    mismatched = [m["run"] for m in runs if m["churn"]["epoch_mismatch"]]
    lines += ["", "Both speeds are the autopilots' EKF estimates (logged velocity and logged "
              "position), not simulator truth.",
              "", "Re-plans by trigger, all runs: " + trigger_sum(runs, "replans_by_trigger")
              + "; in the warm-up: " + trigger_sum(runs, "replans_warmup_by_trigger") + ".",
              "Agents' epoch columns vs ground re-plans: "
              + (f"MISMATCH in {', '.join(mismatched)}" if mismatched else "no mismatch") + ".",
              "", "## Per run", ""] + per_run_table(runs)
    return "\n".join(lines) + "\n"


def trigger_sum(runs, key):
    triggers = {}
    for m in runs:
        for trigger, count in m["churn"].get(key, {}).items():
            triggers[trigger] = triggers.get(trigger, 0) + count
    return ", ".join(f"{k} {v}" for k, v in sorted(triggers.items())) or "none"


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
        json.dump({"name": name, "runs": [m["run"] for m in runs], "summary": summary,
                   "references": {"baseline": [m["run"] for m in baselines],
                                  "ref": [m["run"] for m in refs]},
                   "run_extras": {m["run"]: m["extra"] for m in runs},
                   "per_victim": per_victim_json(runs)}, f, indent=1)
    with open(os.path.join(out_dir, "results.md"), "w") as f:
        f.write(render(name, runs, summary, refs, baselines))


def main():
    parser = argparse.ArgumentParser(description="results table of a set of runs")
    parser.add_argument("runs", nargs="*")
    parser.add_argument("--out")
    parser.add_argument("--name")
    parser.add_argument("--ref", nargs="*", default=[])
    parser.add_argument("--baseline", nargs="*", default=[])
    args = parser.parse_args()
    write_set(args.out, args.name, args.runs, args.ref, args.baseline)
    print(os.path.join(args.out, "results.md"))


if __name__ == "__main__":
    main()
