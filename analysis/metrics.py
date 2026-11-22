"""Metrics of one run, from each agent's logged position (its autopilot's EKF estimate) and the
camera footprint.

Definitions: REPORT.md section 11. Usage: python -m analysis.metrics RUN_DIR
Writes RUN_DIR/metrics.json and RUN_DIR/coverage.csv (coverage in %, once per second).
"""
import csv
import itertools
import json
import math
import os
import sys

import numpy as np

from analysis.logs import load_run
from swarm import config, sensor

CELL_M = 2.0               # grid cell; the 50 m footprint is 24 cells long, so edges barely matter
TICK_S = 1.0               # coverage(t) is evaluated once per second
WINDOW_S = config.COVERAGE_WINDOW_S
END_OF_RUN_S = 30.0        # "end of run" = the last 30 s
RECOVERED = 0.9            # "within 10 %" of a reference value
CONFIRM_S = 10.0           # a recovery must hold at least this long before the run ends
SAFETY_FROM_S = 10.0       # second separation figure: from 10 s into the mission
STEP_S = 1.0 / config.CONTROL_HZ   # agents' positions are compared on this time grid

COLUMN_X_M = np.arange(config.AREA[0] + CELL_M / 2, config.AREA[1], CELL_M)   # cell centres
ROW_Y_M = np.arange(config.AREA[2] + CELL_M / 2, config.AREA[3], CELL_M)
GRID_SHAPE = (len(ROW_Y_M), len(COLUMN_X_M))                  # rows go north, columns go east
CELL_X_M, CELL_Y_M = [a.ravel() for a in np.meshgrid(COLUMN_X_M, ROW_Y_M)]   # one entry per cell
ALL_CELLS = np.ones(CELL_X_M.size, dtype=bool)


def in_phase(times, start_s, end_s):
    return (times >= start_s) & (times < end_s)


def first_time(times, mask):
    """The first time where mask is True, or None."""
    if not mask.any():
        return None
    return float(times[mask][0])


def mission_rows(run, agent_id, from_s=0.0):
    """Rows of an agent that count: from from_s until the end, or until the loss."""
    t = run.agents[agent_id]["t"]
    return (t >= from_s) & (t <= run.live_until(agent_id))


# ---------------------------------------------------------------------- coverage

def sightings(run):
    """[(t, indices of the cells seen)] for every counted log row, in time order.

    Rows count from t = -WINDOW_S: the swarm observes for one window before the mission clock
    starts, so coverage(0) already comes from a full window.
    """
    out = []
    for agent_id, log in run.agents.items():
        for i in np.flatnonzero(mission_rows(run, agent_id, -WINDOW_S)):
            seen = sensor.seen_mask(CELL_X_M, CELL_Y_M, log["x"][i], log["y"][i],
                                    log["z"][i], log["yaw"][i])
            out.append((log["t"][i], np.flatnonzero(seen).astype(np.int32)))
    out.sort(key=lambda sighting: sighting[0])
    return out


def last_seen_at(sightings_list, times):
    """For each of the ascending times, yield (t, per-cell time of the latest sighting <= t).

    -inf means never seen. The same array is updated and yielded every time.
    """
    last = np.full(CELL_X_M.size, -np.inf)
    i = 0
    for t in times:
        while i < len(sightings_list) and sightings_list[i][0] <= t:
            seen_t, cells = sightings_list[i]
            last[cells] = seen_t
            i += 1
        yield t, last


def percent_fresh(sightings_list, ticks, window_s, cells):
    """At each tick t: % of the selected cells seen in (t - window_s, t]."""
    out = []
    for t, last in last_seen_at(sightings_list, ticks):
        out.append(100.0 * np.mean(last[cells] > t - window_s))
    return np.array(out)


def coverage_block(ticks, coverage, end_s):
    end_of_run = coverage[in_phase(ticks, end_s - END_OF_RUN_S, math.inf)]
    return {"mean_pct": float(np.mean(coverage)),
            "end_of_run_pct": float(np.mean(end_of_run)),
            "end_of_run_min_pct": float(np.min(end_of_run))}


# -------------------------------------------------------------------- revisit gap

def revisit_gaps(sightings_list, start_s, end_s):
    """Per cell, for the phase [start_s, end_s): the longest gap between two consecutive
    sightings that are both inside the phase (NaN if there are fewer than two), and the time of
    the last sighting before end_s (NaN if never seen)."""
    longest = np.full(CELL_X_M.size, np.nan)
    last = np.full(CELL_X_M.size, np.nan)
    for t, cells in sightings_list:
        if t >= end_s:
            break
        if t >= start_s:
            previous = last[cells]
            gap = np.where(previous >= start_s, t - previous, np.nan)
            longest[cells] = np.fmax(longest[cells], gap)
        last[cells] = t
    return longest, last


def revisit_block(sightings_list, start_s, end_s):
    longest, last = revisit_gaps(sightings_list, start_s, end_s)
    open_s = end_s - np.nan_to_num(last, nan=0.0)        # never seen: open since the start
    return {"phase_s": [start_s, end_s],
            "worst_completed_s": float(np.nanmax(longest)),
            "worst_open_s": float(np.max(open_s)),
            "no_completed_gap_pct": 100.0 * float(np.mean(np.isnan(longest)))}


# ----------------------------------------------------------------------- the loss

def recovery_time(ticks, coverage, loss_s, threshold_pct):
    """Seconds from the loss to the first tick from which coverage stays >= threshold until
    the end. 0 if it never fell below; None if it is not back for CONFIRM_S before the end."""
    after = ticks >= loss_s
    t = ticks[after]
    below = np.flatnonzero(coverage[after] < threshold_pct)
    if len(below) == 0:
        return 0.0
    back = below[-1] + 1
    if back >= len(t) or t[-1] - t[back] < CONFIRM_S:
        return None
    return float(t[back] - loss_s)


def loss_replan(run):
    """The ground's first re-plan after the loss that was triggered by a lost agent."""
    for e in run.events("assign"):
        if e["t_mission"] is None or e["t_mission"] < run.loss_s:
            continue
        if e["trigger"] == "lost":
            return e
    return None


def recovery_milestones(run, recovery_s):
    """Seconds from the loss to: lost declared, new assignment held by the last survivor,
    first survivor on a slot of its new loop, coverage recovered."""
    replan = loss_replan(run)
    received = []
    on_loop = []
    for agent_id, log in run.agents.items():
        if agent_id == run.victim:
            continue
        loop_from = replan["assignments"][str(agent_id)]["loop_from"]
        received.append(first_time(log["t"], log["epoch"] >= replan["epoch"]))
        flying_loop = (log["epoch"] == replan["epoch"]) & (log["slot"] >= loop_from)
        on_loop.append(first_time(log["t"], flying_loop))
    received_last = None if None in received else max(received)
    on_loop = [t for t in on_loop if t is not None]
    first_on_loop = min(on_loop) if on_loop else None

    def since_loss(t):
        return None if t is None else t - run.loss_s

    return {"lost_declared": since_loss(run.lost_declared_s(run.victim)),
            "assignment_received": since_loss(received_last),
            "on_new_loop": since_loss(first_on_loop),
            "recovered": recovery_s}


def loss_block(run, ticks, coverage):
    loss_s = run.loss_s
    n = run.meta["n"]
    before = coverage[in_phase(ticks, loss_s - WINDOW_S, loss_s)]
    after = coverage[in_phase(ticks, loss_s + WINDOW_S, math.inf)]
    from_loss = in_phase(ticks, loss_s, math.inf)
    before_pct = float(np.mean(before))
    share_pct = (n - 1) / n * before_pct
    recovery_s = recovery_time(ticks, coverage, loss_s, RECOVERED * before_pct)
    dip = int(np.argmin(coverage[from_loss]))
    shortfall = np.maximum(0.0, before_pct - coverage[from_loss])
    lost_declared_s = run.lost_declared_s(run.victim)
    milestones = None
    if lost_declared_s is not None:
        milestones = recovery_milestones(run, recovery_s)
    return {"loss_s": loss_s,
            "lost_declared_s": lost_declared_s,
            "before_pct": before_pct,
            "before_min_pct": float(np.min(before)),
            "after_mean_pct": float(np.mean(after)),
            "after_min_pct": float(np.min(after)),
            "share_pct": share_pct,
            "after_over_share": float(np.mean(after)) / share_pct,
            "recovery_s": recovery_s,
            "recovery_threshold_pct": RECOVERED * before_pct,
            "recovery_to_share_s": recovery_time(ticks, coverage, loss_s, RECOVERED * share_pct),
            "recovery_to_share_threshold_pct": RECOVERED * share_pct,
            "dip_pct": float(coverage[from_loss][dip]),
            "dip_after_loss_s": float(ticks[from_loss][dip] - loss_s),
            "deficit_pct_s": float(np.sum(shortfall)) * TICK_S,
            "milestones_s": milestones}


def against_level(ticks, coverage, loss_s, level_pct):
    """Recovery and deficit after the loss against a fixed level: the no-loss N-agent baseline
    (analysis/report.py). Against each run's own P a run that started lower looks better."""
    from_loss = ticks >= loss_s
    shortfall = np.maximum(0.0, level_pct - coverage[from_loss])
    return {"level_pct": level_pct,
            "recovery_s": recovery_time(ticks, coverage, loss_s, RECOVERED * level_pct),
            "deficit_pct_s": float(np.sum(shortfall)) * TICK_S}


# -------------------------------------------------------------------------- churn

def count_changes(run, assigns, start_s, end_s):
    """Assignment changes of live agents in [start_s, end_s), from the ground's "changed" lists."""
    count = 0
    for e in assigns:
        if start_s <= e["t_mission"] < end_s:
            live = [a for a in e["changed"] if e["t_mission"] < run.live_until(a)]
            count += len(live)
    return count


def live_minutes(run, start_s, end_s):
    seconds = sum(max(0.0, min(end_s, run.live_until(a)) - start_s) for a in run.agents)
    return seconds / 60.0


def epoch_mismatches(run, assigns):
    """Agents whose own log shows a different number of epoch changes than the ground's
    re-plans while they were live. {agent: [ground re-plans, epoch changes in the log]}"""
    out = {}
    for agent_id, log in run.agents.items():
        epochs = log["epoch"][mission_rows(run, agent_id)]
        changes = int(np.count_nonzero(np.diff(epochs)))
        replans = sum(1 for e in assigns if e["t_mission"] < run.live_until(agent_id))
        if changes != replans:
            out[str(agent_id)] = [replans, changes]
    return out


def by_trigger(assigns) -> dict:
    triggers = [e["trigger"] for e in assigns]
    return {name: triggers.count(name) for name in sorted(set(triggers))}


def churn_block(run):
    """Changes per live agent per minute of mission time (t > 0). Re-plans before t = 0, except
    the first plan, are counted apart: the infeasible counters have no time filter."""
    every = run.events("assign")
    assigns = [e for e in every if e["t_mission"] is not None and e["t_mission"] > 0.0]
    warmup = [e for e in every[1:] if e["t_mission"] is None or e["t_mission"] <= 0.0]
    if run.victim is None:
        windows = {"whole": (0.0, run.end_s)}
    else:
        windows = {"before": (0.0, run.loss_s), "after": (run.loss_s, run.end_s)}
    out = {}
    for name, (start_s, end_s) in windows.items():
        changes = count_changes(run, assigns, start_s, end_s)
        out[name + "_per_agent_min"] = changes / live_minutes(run, start_s, end_s)
    out["replans_by_trigger"] = by_trigger(assigns)
    out["replans_warmup_by_trigger"] = by_trigger(warmup)
    if run.victim is not None:
        replan = loss_replan(run)
        forced = count_changes(run, [replan], 0.0, math.inf) if replan else 0
        after_loss = count_changes(run, assigns, run.loss_s, run.end_s)
        out["extra_changes_after_loss"] = after_loss - forced
    out["epoch_mismatch"] = epoch_mismatches(run, assigns)
    return out


# ------------------------------------------------------------------ safety, control

def separation_series(run):
    """Time grid (s, from the first log row) and {(a, b): 3D distance (EKF)} for every pair.

    NaN where either agent has no log yet, or no longer counts (after the end or the loss).
    """
    t_first = min(log["t"][0] for log in run.agents.values())
    times = np.arange(t_first, run.end_s, STEP_S)
    positions = {}
    for agent_id, log in run.agents.items():
        rows = log["t"] <= run.live_until(agent_id)
        positions[agent_id] = np.array([
            np.interp(times, log["t"][rows], log[axis][rows], left=np.nan, right=np.nan)
            for axis in ("x", "y", "z")])
    pairs = {}
    for a, b in itertools.combinations(positions, 2):
        pairs[(a, b)] = np.linalg.norm(positions[a] - positions[b], axis=0)
    return times, pairs


def flag_seconds(log, rows, flag):
    """Seconds the agent carried a flag: each flagged row counts until the next row."""
    row_s = np.diff(log["t"], append=log["t"][-1])
    flagged = np.array([flag in flags.split("|") for flags in log["flags"]], dtype=bool)
    return float(np.sum(row_s[rows & flagged]))


def safety_block(run):
    times, pairs = separation_series(run)
    closest = np.fmin.reduce(np.array(list(pairs.values())), axis=0)   # NaN only if no pair
    per_agent = {}
    for agent_id, log in run.agents.items():
        rows = mission_rows(run, agent_id)
        per_agent[str(agent_id)] = {flag + "_s": flag_seconds(log, rows, flag)
                                    for flag in ("brake", "deaf")}
    infeasible = run.events("infeasible")
    accepted = sum(1 for e in infeasible if e["accepted"])
    out = {"min_separation_m": float(np.nanmin(closest)),
           "min_separation_from_10s_m": float(np.nanmin(closest[times > SAFETY_FROM_S])),
           "below_d_min_s": float(np.count_nonzero(closest < config.D_MIN_M)) * STEP_S,
           "infeasible_accepted": accepted,
           "infeasible_ignored": len(infeasible) - accepted,
           "per_agent": per_agent}
    for flag in ("brake", "deaf"):
        out[flag + "_s"] = sum(agent[flag + "_s"] for agent in per_agent.values())
    return out


def control_block(run):
    """Setpoint rate from the wall-clock spacing of log rows (one row per control tick)."""
    rates, p99s, longest, speeds = [], [], [], []
    for agent_id, log in run.agents.items():
        rows = mission_rows(run, agent_id)
        periods = np.diff(log["t_wall"][rows])
        rates.append(1.0 / np.mean(periods))
        p99s.append(np.percentile(periods, 99))
        longest.append(np.max(periods))
        speeds.append(np.max(np.hypot(log["vx"][rows], log["vy"][rows])))
    return {"min_rate_hz": float(min(rates)), "max_p99_period_s": float(max(p99s)),
            "max_period_s": float(max(longest)), "max_ground_speed_ms": float(max(speeds))}


# ---------------------------------------------------------------- missions B and C

def longest_unseen(sightings_list, end_s, from_s=0.0, open_at_end=True):
    """Per cell: the longest time unseen (s) among the gaps that end in [from_s, end_s).

    A gap runs from one sighting to the next and belongs to the phase where it ends, however
    early it started: a gap the loss opens counts in full after the loss. Its start may lie in
    the warm-up; a cell not seen since counting began (t = -WINDOW_S) counts from there.
    open_at_end: a gap still open at end_s also counts, as its age at end_s.
    """
    last = np.full(CELL_X_M.size, -WINDOW_S)
    longest = np.zeros(CELL_X_M.size)
    for t, cells in sightings_list:
        if t >= end_s:
            break
        if t >= from_s:
            longest[cells] = np.maximum(longest[cells], t - last[cells])
        last[cells] = t
    if open_at_end:
        longest = np.maximum(longest, end_s - last)
    return longest


UNSEEN_LIMITS_S = (60.0, 90.0)   # report the share of cells that went unseen longer than these


def unseen_block(sightings_list, from_s, end_s, open_at_end):
    """Worst revisit gap as the task defines it: the longest time any cell went unseen."""
    longest = longest_unseen(sightings_list, end_s, from_s, open_at_end)
    out = {"phase_s": [from_s, end_s], "worst_s": float(np.max(longest))}
    for limit_s in UNSEEN_LIMITS_S:
        out[f"cells_over_{limit_s:.0f}s_pct"] = 100.0 * float(np.mean(longest > limit_s))
    return out


def unseen_blocks(run, sightings_list):
    """Whole mission [0, end]; with a loss also before [0, L) and from the loss [L, end]. Only
    the phase in which a gap ends gets it, so a gap across L is in "after" and "whole"."""
    out = {"whole": unseen_block(sightings_list, 0.0, run.end_s, True)}
    if run.victim is not None:
        out["before"] = unseen_block(sightings_list, 0.0, run.loss_s, False)
        out["after"] = unseen_block(sightings_list, run.loss_s, run.end_s, True)
    return out


def priority_zone_block(run, sightings_list, ticks):
    xmin, ymin, xmax, ymax = run.meta["priority_zone"]
    zone = (CELL_X_M >= xmin) & (CELL_X_M <= xmax) & (CELL_Y_M >= ymin) & (CELL_Y_M <= ymax)
    fresh = percent_fresh(sightings_list, ticks, config.PRIORITY_REVISIT_S, zone)
    return {"zone": [xmin, ymin, xmax, ymax],
            "longest_unseen_s": float(longest_unseen(sightings_list, run.end_s)[zone].max()),
            "limit_s": config.PRIORITY_REVISIT_S,
            "whole_zone_fresh_pct": 100.0 * float(np.mean(fresh == 100.0))}


def blackout_block(run, safety):
    blackout = run.meta["blackout"]
    agent_id = blackout["agent"]
    log = run.agents[agent_id]
    replans = [e for e in run.events("assign")
               if e["t_mission"] is not None and e["t_mission"] >= blackout["t0"]]
    in_cover = (log["t"] >= blackout["t1"]) & (np.array(log["status"]) == "cover")
    back_s = first_time(log["t"], in_cover)
    return {"agent": agent_id,
            "blackout_s": [blackout["t0"], blackout["t1"]],
            "declared_lost": run.lost_declared_s(agent_id) is not None,
            "replans_from_blackout": len(replans),
            "deaf_s": safety["per_agent"][str(agent_id)]["deaf_s"],
            "back_in_cover_after_s": None if back_s is None else back_s - blackout["t1"]}


def radio_block(total):
    """The radio's totals, plus the loss rate over the copies it did not drop for a window."""
    out = dict(total)
    out["loss_pct"] = 100.0 * total["dropped_loss"] / (total["sent"] - total["dropped_window"])
    return out


# ------------------------------------------------------------------------- driver

def compute(run):
    """All metrics of a run. Returns (metrics dict, ticks, coverage % per tick)."""
    seen = sightings(run)
    ticks = np.arange(0.0, math.floor(run.end_s) + TICK_S, TICK_S)
    coverage = percent_fresh(seen, ticks, WINDOW_S, ALL_CELLS)
    meta = run.meta
    m = {"run": run.name, "mission": meta["mission"], "scenario": meta["scenario"],
         "seed": meta["seed"], "n": meta["n"], "backend": meta["backend"],
         "victim": run.victim, "end_s": run.end_s,
         "coverage": coverage_block(ticks, coverage, run.end_s),
         "unseen": unseen_blocks(run, seen),
         "loss": None, "revisit": None, "mission_b": None, "mission_c": None}
    if run.victim is not None:
        m["loss"] = loss_block(run, ticks, coverage)
        lost_declared_s = m["loss"]["lost_declared_s"]
        m["revisit"] = {"before": revisit_block(seen, 0.0, run.loss_s), "after": None}
        if lost_declared_s is not None:
            m["revisit"]["after"] = revisit_block(seen, lost_declared_s, run.end_s)
    m["churn"] = churn_block(run)
    m["safety"] = safety_block(run)
    m["control"] = control_block(run)
    m["radio"] = radio_block(run.radio["total"])
    if meta["mission"] == "B":
        m["mission_b"] = priority_zone_block(run, seen, ticks)
    if meta["mission"] == "C":
        m["mission_c"] = blackout_block(run, m["safety"])
    return m, ticks, coverage


def analyse(run_dir):
    m, ticks, coverage = compute(load_run(run_dir))
    with open(os.path.join(run_dir, "metrics.json"), "w") as f:
        json.dump(m, f, indent=1)
    with open(os.path.join(run_dir, "coverage.csv"), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_s", "coverage_pct"])
        for t, c in zip(ticks, coverage):
            writer.writerow([f"{t:.0f}", f"{c:.3f}"])
    return m


def read_coverage_csv(run_dir):
    """(ticks, coverage %) as written by analyse()."""
    data = np.loadtxt(os.path.join(run_dir, "coverage.csv"), delimiter=",", skiprows=1, ndmin=2)
    return data[:, 0], data[:, 1]


if __name__ == "__main__":
    result = analyse(sys.argv[1])
    print(f"{result['run']}: coverage {result['coverage']['mean_pct']:.1f} %,"
          f" min separation {result['safety']['min_separation_m']:.1f} m")
