"""Per-run metrics, computed OFFLINE from the sensor footprint model (swarm/sensor.py) and each
agent's own logged TRUE position. Nothing here uses what agents told each other.

Usage:  python -m analysis.metrics runs/<name> [--cell 2.0]
Writes: runs/<name>/metrics.json, coverage.csv, separation.csv, gapmaps.npz

Definitions (times are mission time t, seconds; L = meta kill_t_mission, the loss time)
  sample        one agent log row with status in FLYING_STATUSES and 0 <= t_mission <= mission_s.
                Each sample marks the grid cells whose centre lies inside the footprint
                (logged x, y, z, yaw) as seen at time t.
  coverage(t)   % of cells seen by any agent at some sample time in [t - 60 s, t]; evaluated at
                t = 0, 1, 2, ... mission_s. For t < 60 s the window is still filling (warm-up).
  pre-loss P    mean of coverage(t) over the ticks L-10 <= t < L. Flagged
                "warmup_incomplete" when L-10 < 60 s. Sensitivity: P from 5 s and 20 s windows
                and coverage(L) are also reported, with the recovery times they would give.
  recovery      t_rec - L, where t_rec is the first tick >= L after which coverage stays
                >= threshold until mission end, with at least HOLD_S of data after t_rec
                (otherwise None = "not recovered"). 0.0 if coverage never fell below it after L.
                (a) threshold = 0.9 * P                (within 10 % of the pre-loss value)
                (b) threshold = 0.9 * (N-1)/N * P      (within 10 % of the ideal survivor share)
                "within 10 %" is relative (x 0.9), not 10 percentage points.
  phases        pre  = [0, L];  post = [L + max(60 s, recovery_b), mission_s] (needs >= HOLD_S).
                The post phase never starts before L+60 s, so its coverage window contains no
                sighting by the lost agent. If recovery_b is None, post = [L+60, end] and the
                result is flagged recovered_b = false.
  revisit gap   per cell, the longest time inside the phase during which the cell was not seen.
                Unseen stretches that started before the phase are clipped to the phase start,
                stretches still open at the phase end run to the phase end. Exception: at
                mission start (t=0) the time until a cell's FIRST sighting is first-visit latency,
                not a revisit gap, and is excluded. Cells never seen in the phase get the full
                phase length (a censored lower bound) and are counted separately
                ("censored_by_phase_length" is then true: the post phase is only 30-60 s long).
  worst age     companion to the gap: max over cells and t in the phase of (t - last sighting),
                NOT clipped at the phase start, so a stretch that began before the phase counts
                in full. This is the stalest any cell got during the phase; it is not censored
                by a short post phase. Gap <= age always.
  detection     lost_declared_t = first ground "liveness" event with the victim going to "lost";
                detection_delay = lost_declared_t - L; recovery_after_declared = L + recovery -
                lost_declared_t (negative when coverage never fell below the threshold).
  churn         assignment changes per live agent-minute: number of (assign event, agent) pairs
                with the agent in that event's "changed" list and alive at the event, divided by
                the live agent-minutes in the phase. Given with all events and without the
                initial assignment and the one re-plan forced by the loss.
  safety        min over time of the TRUE 3D distance between any two live agents (logs joined on
                t_mission by linear interpolation at 10 Hz, no interpolation across >0.5 s gaps);
                time with that distance < D_MIN.
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

from swarm import config, sensor
from analysis.logs import FLYING_STATUSES, RunLoadError, load_run

CELL_M = 2.0          # default grid cell: 1 m of travel per 10 Hz sample at 10 m/s; the smallest
                      # footprint (48 m) is 24 cells long, so edge quantisation is ~2 % of a
                      # footprint's area and averages out; 150 x 150 = 22 500 cells stays cheap.
WINDOW_S = 60.0       # coverage memory
TICK_S = 1.0          # coverage(t) sampling
PRE_WINDOW_S = 10.0   # pre-loss reference window
HOLD_S = 10.0         # minimum confirmed time after recovery / minimum post phase length
RECOVERY_FRAC = 0.9   # "within 10 %"
SEP_DT = 0.1          # separation join step (10 Hz)
SEP_MAX_GAP_S = 0.5   # no interpolation across larger log gaps


# ----------------------------------------------------------------------------------- grid

class Grid:
    """Square cells over config.AREA. Arrays are indexed [iy, ix]; cx/cy are cell centres (m)."""

    def __init__(self, cell=CELL_M, area=config.AREA):
        self.cell = float(cell)
        self.xmin, self.xmax, self.ymin, self.ymax = area
        self.nx = int(round((self.xmax - self.xmin) / self.cell))
        self.ny = int(round((self.ymax - self.ymin) / self.cell))
        self.cx = self.xmin + (np.arange(self.nx) + 0.5) * self.cell
        self.cy = self.ymin + (np.arange(self.ny) + 0.5) * self.cell

    @property
    def shape(self):
        return (self.ny, self.nx)

    @property
    def extent(self):
        return (self.xmin, self.xmax, self.ymin, self.ymax)

    def footprint_cells(self, x, y, z, yaw):
        """(row_slice, col_slice, mask) of the cells seen from this pose, or None if none.
        Only the axis-aligned bounding box of the footprint is tested (cheap)."""
        if not all(map(math.isfinite, (x, y, z, yaw))):
            return None
        hx, hy = sensor.bounding_half_extent(z, yaw)
        ix0 = max(0, int(math.floor((x - hx - self.xmin) / self.cell)))
        ix1 = min(self.nx, int(math.ceil((x + hx - self.xmin) / self.cell)) + 1)
        iy0 = max(0, int(math.floor((y - hy - self.ymin) / self.cell)))
        iy1 = min(self.ny, int(math.ceil((y + hy - self.ymin) / self.cell)) + 1)
        if ix0 >= ix1 or iy0 >= iy1:
            return None
        m = sensor.seen_mask(self.cx[None, ix0:ix1], self.cy[iy0:iy1, None], x, y, z, yaw)
        if not m.any():
            return None
        return slice(iy0, iy1), slice(ix0, ix1), m


# -------------------------------------------------------------------------------- samples

def coverage_samples(run, mission_s):
    """All usable footprint samples, sorted by time: array of rows (t, x, y, z, yaw, agent_id)."""
    out = []
    for aid, log in run.agents.items():
        t = log["t_mission"]
        ok = np.isfinite(t) & (t >= 0.0) & (t <= mission_s)
        ok &= np.array([s in FLYING_STATUSES for s in log["status"]], dtype=bool)
        for i in np.nonzero(ok)[0]:
            out.append((t[i], log["x"][i], log["y"][i], log["z"][i], log["yaw"][i], aid))
    arr = np.array(out, dtype=float).reshape(-1, 6)
    return arr[np.argsort(arr[:, 0], kind="stable")]


def sightings(grid, samples, t_max=math.inf):
    """Yield (t, row_slice, col_slice, mask) for each sample with t <= t_max, in time order."""
    for t, x, y, z, yaw, _ in samples:
        if t > t_max:
            return
        fc = grid.footprint_cells(x, y, z, yaw)
        if fc is not None:
            yield (t,) + fc


class LastSeen:
    """Incremental last-seen-time map; advance(t) applies every sample with time <= t."""

    def __init__(self, grid, samples):
        self.grid, self.samples = grid, samples
        self.last = np.full(grid.shape, -np.inf)
        self.i = 0

    def advance(self, t):
        while self.i < len(self.samples) and self.samples[self.i, 0] <= t:
            s = self.samples[self.i]
            fc = self.grid.footprint_cells(*s[1:5])
            if fc is not None:
                rs, cs, m = fc
                self.last[rs, cs][m] = s[0]
            self.i += 1
        return self.last

    def fresh_fraction(self, t, window=WINDOW_S):
        return float(np.mean(self.last >= t - window))


# ------------------------------------------------------------------------------- coverage

def coverage_series(grid, samples, mission_s, window=WINDOW_S, tick=TICK_S):
    """(ticks, coverage fraction 0..1) at t = 0, tick, ... mission_s. See module docstring."""
    ticks = np.arange(0.0, mission_s + 1e-9, tick)
    ls = LastSeen(grid, samples)
    cov = np.empty(len(ticks))
    for k, t in enumerate(ticks):
        ls.advance(t)
        cov[k] = ls.fresh_fraction(t, window)
    return ticks, cov


def pre_loss_value(ticks, cov, t_loss, width=PRE_WINDOW_S):
    """Mean coverage over ticks t_loss - width <= t < t_loss (None if no ticks)."""
    sel = (ticks >= t_loss - width) & (ticks < t_loss)
    return float(np.mean(cov[sel])) if sel.any() else None


def recovery_time(ticks, cov, t_loss, threshold, hold_s=HOLD_S):
    """Seconds from t_loss until coverage is back >= threshold for good (see module docstring).
    Returns 0.0 if it never fell below after t_loss, None if not recovered/confirmed in the run."""
    if threshold is None:
        return None
    after = ticks >= t_loss
    t_after, c_after = ticks[after], cov[after]
    if len(t_after) == 0:
        return None
    below = np.nonzero(c_after < threshold)[0]
    if len(below) == 0:
        return 0.0
    k = below[-1] + 1
    if k >= len(t_after):
        return None
    t_rec = t_after[k]
    if t_after[-1] - t_rec < hold_s:
        return None
    return float(t_rec - t_loss)


def dip(ticks, cov, t_loss):
    """(min coverage after loss, time of that minimum)."""
    sel = ticks >= t_loss
    if not sel.any():
        return None, None
    k = int(np.argmin(cov[sel]))
    return float(cov[sel][k]), float(ticks[sel][k])


# ---------------------------------------------------------------------------- revisit gap

def phase_gaps(grid, samples, a, b, from_first_sighting=False):
    """Per-cell max unseen time inside phase [a, b] (s) and the never-seen-in-phase mask.

    Unseen stretches are clipped to [a, b]. With from_first_sighting=True (use for a phase that
    starts at mission start) the time before a cell's first ever sighting is not counted.
    Cells never seen in the phase are given b - a.
    """
    gap, never, _ = phase_gaps_and_age(grid, samples, a, b, from_first_sighting)
    return gap, never


def phase_gaps_and_age(grid, samples, a, b, from_first_sighting=False):
    """As phase_gaps, plus the per-cell max AGE (staleness) over t in [a, b]:
    age(t) = t - last sighting time, NOT clipped to the phase start (a stretch that began before
    the phase counts in full). Cells not seen since mission start age from t = 0, except with
    from_first_sighting=True, where they are not counted until their first sighting
    (and get NaN if never seen)."""
    last = np.full(grid.shape, -np.inf)
    gap = np.zeros(grid.shape)
    age = np.zeros(grid.shape)
    seen = np.zeros(grid.shape, dtype=bool)
    for t, rs, cs, m in sightings(grid, samples, t_max=b):
        if t >= a:
            prev = last[rs, cs][m]
            if from_first_sighting:
                g = np.where(np.isfinite(prev), t - np.maximum(prev, a), 0.0)
                ag = np.where(np.isfinite(prev), t - prev, 0.0)
            else:
                g = t - np.maximum(prev, a)
                ag = t - np.maximum(prev, 0.0)
            sub_gap, sub_age = gap[rs, cs], age[rs, cs]
            sub_gap[m] = np.maximum(sub_gap[m], g)
            sub_age[m] = np.maximum(sub_age[m], ag)
            seen[rs, cs][m] = True
        last[rs, cs][m] = t
    gap = np.where(seen, np.maximum(gap, b - np.maximum(last, a)), b - a)
    end_age = b - np.maximum(last, 0.0)
    age = np.maximum(age, end_age)
    if from_first_sighting:
        age = np.where(np.isfinite(last), age, np.nan)
    return gap, ~seen, age


def gap_summary(gap, never, age=None):
    if gap is None:
        return None
    return {"worst_s": float(gap.max()),
            "censored_by_phase_length": bool(never.any()),
            "worst_age_s": None if age is None or not np.isfinite(age).any() else float(np.nanmax(age)),
            "worst_seen_cells_s": float(gap[~never].max()) if (~never).any() else None,
            "p95_s": float(np.percentile(gap, 95)),
            "median_s": float(np.median(gap)),
            "never_seen_cells_pct": 100.0 * float(never.mean()),
            "cells_gap_over_60s_pct": 100.0 * float((gap > WINDOW_S).mean())}


# ------------------------------------------------------------------------------ events

def event_time(e, run):
    """Mission time of a ground event: t_mission, else t_wall - meta t0_wall, else None."""
    t = e.get("t_mission")
    if isinstance(t, (int, float)):
        return float(t)
    t0, tw = run.meta.get("t0_wall"), e.get("t_wall")
    if isinstance(t0, (int, float)) and isinstance(tw, (int, float)):
        return float(tw - t0)
    return None


def lost_declared_time(run, victim, t_loss):
    """First ground 'liveness' event declaring the victim lost (at or after t_loss)."""
    for e in run.events("liveness"):
        if e.get("agent") == victim and e.get("to") == "lost":
            t = event_time(e, run)
            if t is not None and (t_loss is None or t >= t_loss):
                return t
    return None


def loss_time(run):
    """Mission time of the kill: meta kill_wall - t0_wall (the ACTUAL SIGKILL time written by
    run_mission.py) when both exist, else meta kill_t_mission (the planned time). None if no kill."""
    if run.meta.get("kill_agent") is None:
        return None
    kw, t0 = run.meta.get("kill_wall"), run.meta.get("t0_wall")
    if isinstance(kw, (int, float)) and isinstance(t0, (int, float)):
        return float(kw - t0)
    t = run.meta.get("kill_t_mission")
    return None if t is None else float(t)


def alive_interval(aid, victim, t_loss, mission_s):
    end = t_loss if (aid == victim and t_loss is not None) else mission_s
    return 0.0, end


def churn(run, agent_ids, victim, t_loss, mission_s, windows, t_lost_decl):
    """Assignment changes per live agent-minute for each window {name: (a, b)}.
    Events with no usable time are treated as t=0 (sent before mission start)."""
    events = []
    for e in run.events("assign"):
        t = event_time(e, run)
        events.append((0.0 if t is None or t < 0 else t, e))
    events.sort(key=lambda p: p[0])
    initial = id(events[0][1]) if events else None
    loss_replan, t_ref = None, (t_lost_decl if t_lost_decl is not None else t_loss)
    if t_ref is not None:
        for t, e in events:
            if t >= t_ref:
                loss_replan = id(e)
                break
    missing_changed = sum(1 for _, e in events if "changed" not in e)
    out = {"n_assign_events": len(events),
           "initial_epoch": events[0][1].get("epoch") if events else None,
           "loss_replan_epoch": next((e.get("epoch") for _, e in events if id(e) == loss_replan), None),
           "events_without_changed_field": missing_changed}
    for name, win in windows.items():
        if win is None:
            out[name] = None
            continue
        a, b = win
        minutes = 0.0
        for aid in agent_ids:
            s, f = alive_interval(aid, victim, t_loss, mission_s)
            minutes += max(0.0, min(b, f) - max(a, s)) / 60.0
        n_all, n_excl = 0, 0
        for t, e in events:
            if not (a <= t <= b):
                continue
            k = sum(1 for aid in e.get("changed", [])
                    if aid in agent_ids and alive_interval(aid, victim, t_loss, mission_s)[1] >= t)
            n_all += k
            if id(e) not in (initial, loss_replan):
                n_excl += k
        out[name] = {"window_s": [a, b], "live_agent_minutes": minutes,
                     "changes_all": n_all, "changes_excl_initial_and_loss_replan": n_excl,
                     "per_agent_min_all": n_all / minutes if minutes > 0 else None,
                     "per_agent_min_excl": n_excl / minutes if minutes > 0 else None}
    return out


# -------------------------------------------------------------------------------- safety

def interp_positions(log, tgrid):
    """(len(tgrid), 3) positions interpolated on t_mission; NaN outside the log or across gaps."""
    t = log["t_mission"]
    ok = np.isfinite(t) & np.isfinite(log["x"]) & np.isfinite(log["y"]) & np.isfinite(log["z"])
    t = t[ok]
    out = np.full((len(tgrid), 3), np.nan)
    if len(t) < 2:
        return out
    order = np.argsort(t)
    t = t[order]
    for j, c in enumerate(("x", "y", "z")):
        out[:, j] = np.interp(tgrid, t, log[c][ok][order], left=np.nan, right=np.nan)
    k = np.searchsorted(t, tgrid)
    k = np.clip(k, 1, len(t) - 1)
    big = (t[k] - t[k - 1]) > SEP_MAX_GAP_S
    out[big] = np.nan
    return out


def separation_series(run, mission_s, dt=SEP_DT):
    """(tgrid, min pairwise 3D distance between live agents; NaN when < 2 agents logged)."""
    tgrid = np.arange(0.0, mission_s + 1e-9, dt)
    pos = [interp_positions(log, tgrid) for log in run.agents.values()]
    best = np.full(len(tgrid), np.inf)
    for i in range(len(pos)):
        for j in range(i + 1, len(pos)):
            d = np.linalg.norm(pos[i] - pos[j], axis=1)
            best = np.fmin(best, np.where(np.isfinite(d), d, np.inf))
    best[~np.isfinite(best)] = np.nan
    return tgrid, best


def flag_tokens(s):
    return [tok for tok in s.replace(";", "|").replace(",", "|").replace(" ", "|").split("|") if tok]


def flag_times(run, mission_s):
    """Seconds in fallback (status or flag) and 'deaf' (flag) per agent, from 10 Hz rows."""
    out, vocab = {}, set()
    for aid, log in run.agents.items():
        t = log["t_mission"]
        ok = np.isfinite(t) & (t >= 0) & (t <= mission_s)
        tw = log["t_wall"][ok]
        dt = float(np.median(np.diff(tw))) if len(tw) > 1 else 1.0 / config.CONTROL_HZ
        fb = deaf = 0
        for i in np.nonzero(ok)[0]:
            toks = flag_tokens(log["flags"][i])
            vocab.update(toks)
            fb += (log["status"][i] == "fallback") or any("fallback" in x for x in toks)
            deaf += any("deaf" in x for x in toks)
        out[str(aid)] = {"fallback_s": fb * dt, "deaf_s": deaf * dt}
    return out, sorted(vocab)


def control_rate(run, mission_s):
    """Per agent: log period stats from t_wall deltas over rows with 0 <= t_mission <= mission_s."""
    out = {}
    for aid, log in run.agents.items():
        t = log["t_mission"]
        ok = np.isfinite(t) & (t >= 0) & (t <= mission_s)
        d = np.diff(log["t_wall"][ok])
        if len(d) == 0:
            out[str(aid)] = None
            continue
        out[str(aid)] = {"rows": int(ok.sum()), "mean_period_s": float(d.mean()),
                         "rate_hz": float(1.0 / d.mean()) if d.mean() > 0 else None,
                         "p99_period_s": float(np.percentile(d, 99)), "max_period_s": float(d.max())}
    return out


# ------------------------------------------------------------------------------ driver

def _pct(x):
    return None if x is None else 100.0 * x


def compute(run, cell=CELL_M):
    """All metrics for a loaded Run. Returns (metrics dict, series dict for files/plots)."""
    mission_s = float(run.meta_get("mission_s", config.MISSION_S))
    if "mission_s" not in run.meta:
        run.warn(f"mission_s not in meta.json; using config.MISSION_S = {mission_s}")
    victim = run.meta.get("kill_agent")
    t_loss = loss_time(run)
    if t_loss is None:
        run.warn("kill_t_mission unknown: loss-relative metrics are null")
    n = int(run.meta_get("n", len(run.agents)))
    agent_ids = sorted(run.agents)

    grid = Grid(cell)
    samples = coverage_samples(run, mission_s)
    ticks, cov = coverage_series(grid, samples, mission_s)

    m = {"run": run.name, "seed": run.meta.get("seed"), "backend": run.meta.get("backend"),
         "n_agents": n, "agents_logged": agent_ids, "victim": victim, "t_loss_s": t_loss,
         "mission_s": mission_s, "cell_m": cell, "window_s": WINDOW_S,
         "sensor": {"product": sensor.PRODUCT, "hfov_deg": sensor.HFOV_DEG,
                    "vfov_deg": round(sensor.VFOV_DEG, 2),
                    "footprint_60m_m": [round(v, 2) for v in sensor.footprint_size(60.0)],
                    # informational only: coverage uses the logged z of every sample
                    "alt_nominal_m": float(run.meta.get("alt_nominal", 60.0)),
                    "footprint_nominal_m": [round(v, 2) for v in sensor.footprint_size(
                        float(run.meta.get("alt_nominal", 60.0)))]},
         "n_samples": int(len(samples)), "definitions": __doc__}

    t_lost_decl = lost_declared_time(run, victim, t_loss) if victim is not None else None
    loss = None
    phases = {"overall": (0.0, mission_s), "pre": None, "post": None}
    if t_loss is not None:
        P = pre_loss_value(ticks, cov, t_loss)
        ideal = None if P is None else P * (n - 1) / n
        rec_a = recovery_time(ticks, cov, t_loss, None if P is None else RECOVERY_FRAC * P)
        rec_b = recovery_time(ticks, cov, t_loss, None if ideal is None else RECOVERY_FRAC * ideal)
        cmin, tmin = dip(ticks, cov, t_loss)
        sens = {}
        for label, val in (("mean_5s", pre_loss_value(ticks, cov, t_loss, 5.0)),
                           ("mean_10s", P),
                           ("mean_20s", pre_loss_value(ticks, cov, t_loss, 20.0)),
                           ("at_loss", float(np.interp(t_loss, ticks, cov)))):
            if val is None:
                sens[label] = None
                continue
            sens[label] = {"pre_loss_pct": _pct(val),
                           "recovery_a_s": recovery_time(ticks, cov, t_loss, RECOVERY_FRAC * val),
                           "recovery_b_s": recovery_time(ticks, cov, t_loss,
                                                         RECOVERY_FRAC * val * (n - 1) / n)}
        phases["pre"] = (0.0, float(t_loss))
        post_a = t_loss + max(WINDOW_S, rec_b if rec_b is not None else 0.0)
        if mission_s - post_a >= HOLD_S:
            phases["post"] = (float(post_a), mission_s)
        post_cov = None
        if phases["post"]:
            sel = (ticks >= phases["post"][0]) & (ticks <= phases["post"][1])
            post_cov = float(np.mean(cov[sel]))
        loss = {"pre_loss_pct": _pct(P),
                "pre_loss_definition": f"mean coverage(t) for {t_loss - PRE_WINDOW_S:.0f} <= t < {t_loss:.0f} s",
                "warmup_incomplete": bool(t_loss - PRE_WINDOW_S < WINDOW_S),
                "ideal_survivor_pct": _pct(ideal),
                "recovery_a_s": rec_a, "recovery_a_threshold_pct": _pct(None if P is None else RECOVERY_FRAC * P),
                "recovery_b_s": rec_b, "recovery_b_threshold_pct": _pct(None if ideal is None else RECOVERY_FRAC * ideal),
                "recovered_a": rec_a is not None, "recovered_b": rec_b is not None,
                "dip_min_pct": _pct(cmin), "dip_min_t_s": tmin,
                "lost_declared_t_s": t_lost_decl,
                "detection_delay_s": None if t_lost_decl is None else t_lost_decl - t_loss,
                "recovery_after_declared_a_s": None if (rec_a is None or t_lost_decl is None) else t_loss + rec_a - t_lost_decl,
                "recovery_after_declared_b_s": None if (rec_b is None or t_lost_decl is None) else t_loss + rec_b - t_lost_decl,
                "post_coverage_pct": _pct(post_cov),
                "post_vs_ideal_ratio": None if (post_cov is None or not ideal) else post_cov / ideal,
                "post_minus_ideal_pct_points": None if (post_cov is None or ideal is None) else _pct(post_cov - ideal),
                "pre_loss_sensitivity": sens}
        if t_lost_decl is None:
            run.warn("no ground 'liveness ... lost' event for the victim: detection delay unknown")
    m["loss"] = loss
    m["phases_s"] = phases

    gaps, maps = {}, {}
    for name, ph in phases.items():
        if ph is None:
            gaps[name] = None
            continue
        g, never, age = phase_gaps_and_age(grid, samples, ph[0], ph[1],
                                           from_first_sighting=(ph[0] == 0.0))
        gaps[name] = gap_summary(g, never, age)
        maps[name] = g
    m["revisit_gap"] = gaps

    pre_w = phases["pre"]
    m["churn"] = churn(run, agent_ids, victim, t_loss, mission_s,
                       {"pre": (pre_w[0], pre_w[1] - 1e-9) if pre_w else None,
                        "post": phases["post"], "overall": phases["overall"]}, t_lost_decl)

    tsep, sep = separation_series(run, mission_s)
    below = np.nan_to_num(sep, nan=np.inf) < config.D_MIN
    k = int(np.nanargmin(sep)) if np.isfinite(sep).any() else None
    infeas = run.events("infeasible")
    fl, vocab = flag_times(run, mission_s)
    m["safety"] = {"min_separation_m": None if k is None else float(sep[k]),
                   "min_separation_t_s": None if k is None else float(tsep[k]),
                   "time_below_d_min_s": float(below.sum() * SEP_DT), "d_min_m": config.D_MIN,
                   "infeasible_reports": len(infeas),
                   "infeasible_by_reason": _count(e.get("reason") for e in infeas),
                   "flag_time_s": fl, "flag_tokens_seen": vocab}
    m["control_rate"] = control_rate(run, mission_s)
    m["radio"] = run.radio[-1] if run.radio else None
    m["warnings"] = run.warnings
    series = {"ticks": ticks, "coverage": cov, "t_sep": tsep, "sep": sep, "maps": maps, "grid": grid}
    return m, series


def _count(items):
    out = {}
    for x in items:
        out[str(x)] = out.get(str(x), 0) + 1
    return out


def write_outputs(run_dir, m, series):
    with open(os.path.join(run_dir, "metrics.json"), "w") as f:
        json.dump(m, f, indent=1, default=float)
    with open(os.path.join(run_dir, "coverage.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_mission_s", "coverage_pct"])
        for t, c in zip(series["ticks"], series["coverage"]):
            w.writerow([f"{t:.1f}", f"{100 * c:.3f}"])
    with open(os.path.join(run_dir, "separation.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_mission_s", "min_separation_m"])
        for t, d in zip(series["t_sep"], series["sep"]):
            w.writerow([f"{t:.1f}", "" if not np.isfinite(d) else f"{d:.3f}"])
    g = series["grid"]
    np.savez_compressed(os.path.join(run_dir, "gapmaps.npz"), extent=np.array(g.extent),
                        cell=g.cell, **{k: v.astype(np.float32) for k, v in series["maps"].items()})


def analyse(run_dir, cell=CELL_M):
    run = load_run(run_dir)
    m, series = compute(run, cell)
    write_outputs(run_dir, m, series)
    return m, series


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run_dir")
    ap.add_argument("--cell", type=float, default=CELL_M, help="grid cell size, m (default 2)")
    a = ap.parse_args(argv)
    try:
        m, _ = analyse(a.run_dir, a.cell)
    except RunLoadError as e:
        sys.exit(f"error: {e}")
    for w in m["warnings"]:
        print("warning:", w)
    L = m["loss"] or {}
    print(f"{m['run']}: pre-loss {L.get('pre_loss_pct')}, recovery a {L.get('recovery_a_s')} s, "
          f"b {L.get('recovery_b_s')} s, min sep {m['safety']['min_separation_m']} m")


if __name__ == "__main__":
    main()
