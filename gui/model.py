"""One run directory as the web view sees it: read incrementally, so the same code follows a live
mission and replays a finished one. Coverage age is computed here, from the camera footprint of
swarm/sensor.py, never in the browser.
"""
import bisect
import csv
import glob
import io
import json
import math
import os

import numpy as np

from swarm import config, sensor

GROUND_IDS = (0, 255)          # the ground's node id: 0 in old logs, 255 after the rewrite
CELL_M = 5.0                   # heatmap cell
WINDOW_S = config.COVERAGE_WINDOW_S
NEVER = -1                     # heat value of a cell never seen
SEND_EVERY = 2                 # 20 Hz rows -> 10 Hz to the browser; enough for drawing
DRONE_COLORS = ("#1f5fd6", "#0b6b2a", "#111111")   # blue, dark green, black
if os.environ.get("SWARM_DEMO_GROUND") == "field":   # the live view's bright drones, darker for a light map
    DRONE_COLORS = ("#b58900", "#0891b2", "#c026d3")

COLUMN_X_M = np.arange(config.AREA[0] + CELL_M / 2, config.AREA[1], CELL_M)
ROW_Y_M = np.arange(config.AREA[2] + CELL_M / 2, config.AREA[3], CELL_M)
CELL_X_M, CELL_Y_M = [a.ravel() for a in np.meshgrid(COLUMN_X_M, ROW_Y_M)]  # row 0 = south


def number(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


class Tail:
    """New complete lines of a growing file; a half-written last line waits for the next read."""

    def __init__(self, path):
        self.path = path
        self.offset = 0

    def lines(self):
        if not os.path.exists(self.path):
            return []
        with open(self.path, "rb") as f:
            f.seek(self.offset)
            data = f.read()
        end = data.rfind(b"\n") + 1
        self.offset += end
        return data[:end].decode(errors="replace").splitlines()


class Coverage:
    """Per-cell time of the last sighting, fed with footprints in any order."""

    def __init__(self):
        self.sightings = []          # sorted (t_wall, x, y, z, yaw, agent id, covering)
        self.reset()

    def reset(self):
        self.last_t = np.full(CELL_X_M.size, -np.inf)
        self.applied = 0             # sightings[:applied] are in last_t
        self.cursor_t = -math.inf

    def add(self, sighting):
        i = bisect.bisect(self.sightings, sighting)
        self.sightings.insert(i, sighting)
        if i < self.applied:         # arrived late for a time already shown: apply it now
            self.apply(sighting)
            self.applied += 1

    def apply(self, sighting):
        t, x, y, z, yaw = sighting[:5]
        seen = sensor.seen_mask(CELL_X_M, CELL_Y_M, x, y, z, yaw)
        self.last_t[seen] = np.maximum(self.last_t[seen], t)

    def advance(self, t_wall, on_sighting=None):
        if t_wall < self.cursor_t:   # seek backwards: rebuild from the start
            self.reset()
        while self.applied < len(self.sightings) and self.sightings[self.applied][0] <= t_wall:
            self.apply(self.sightings[self.applied])
            if on_sighting:
                on_sighting(self.sightings[self.applied])
            self.applied += 1
        self.cursor_t = t_wall

    def fresh_pct(self, t_wall):
        return 100.0 * float(np.mean(self.last_t > t_wall - WINDOW_S))

    def ages_s(self, t_wall):
        age = np.clip(t_wall - self.last_t, 0, WINDOW_S)
        return np.where(np.isinf(self.last_t), NEVER, np.round(age)).astype(int).tolist()


class RunModel:
    def __init__(self, run_dir):
        self.dir = run_dir
        self.base_wall = None        # all times sent to the browser are t_wall - base_wall
        self.meta = {}
        self.agent_tails = {}
        self.agent_header = {}
        self.ground_tail = Tail(os.path.join(run_dir, "ground.jsonl"))
        self.radio_tail = Tail(os.path.join(run_dir, "radio.jsonl"))
        self.rows = {}               # agent id -> [compact rows], all of them
        self.events = []
        self.radio = None
        self.view = Coverage()       # follows the browser's time
        self.series = Coverage()     # runs ahead once, for the coverage-over-time plot
        self.series_points = []      # [t, coverage %, min pairwise distance so far, zone age]
        self.last_pos = {}           # agent id -> (t_wall, x, y, z) for the separation
        self.min_dist_m = None
        self.latest_wall = None

    # ---- reading
    def refresh(self):
        self.read_meta()
        for path in glob.glob(os.path.join(self.dir, "agent_*.csv")):
            agent_id = number(os.path.basename(path)[len("agent_"):-len(".csv")])
            if agent_id is not None and int(agent_id) not in GROUND_IDS:
                self.read_agent(int(agent_id), path)
        for line in self.ground_tail.lines():
            self.add_event(json.loads(line))
        for line in self.radio_tail.lines():
            record = json.loads(line)
            if record.get("event") == "stats":
                self.radio = radio_summary(record)
        self.advance_series()

    def read_meta(self):
        try:
            with open(os.path.join(self.dir, "meta.json")) as f:
                self.meta = json.load(f)
        except (OSError, ValueError):
            pass                     # not written yet, or being rewritten right now

    def read_agent(self, agent_id, path):
        tail = self.agent_tails.setdefault(agent_id, Tail(path))
        lines = tail.lines()
        if agent_id not in self.agent_header and lines:
            self.agent_header[agent_id] = next(csv.reader([lines.pop(0)]))
        rows = self.rows.setdefault(agent_id, [])
        for record in csv.DictReader(io.StringIO("\n".join(lines)),
                                     fieldnames=self.agent_header.get(agent_id)):
            self.add_row(agent_id, record, rows)

    def add_row(self, agent_id, record, rows):
        t_wall, x, y, z = (number(record.get(k)) for k in ("t_wall", "x", "y", "z"))
        yaw = number(record.get("yaw")) or 0.0
        if t_wall is None or x is None or y is None:
            return                   # no position before the autopilot has one
        if self.base_wall is None:
            self.base_wall = math.floor(t_wall)
        self.latest_wall = max(self.latest_wall or t_wall, t_wall)
        if not self.is_dead(agent_id, t_wall):
            sighting = (t_wall, x, y, z or 0.0, yaw, agent_id, record.get("status") == "cover")
            self.series.add(sighting)
            self.view.add(sighting)
        compact = [round(t_wall - self.base_wall, 2), round(x, 1), round(y, 1),
                   round(z or 0.0, 1), round(yaw, 3), record.get("status"),
                   number(record.get("epoch")), number(record.get("slot")),
                   number(record.get("nbr_age")), number(record.get("nbr_dist"))]
        rows.append(compact)

    def is_dead(self, agent_id, t_wall):
        """After the kill the victim's log shows a dead vehicle, not a camera."""
        kill_wall = self.meta.get("kill_wall")
        return agent_id == self.meta.get("kill_agent") and kill_wall and t_wall >= kill_wall

    def track_separation(self, sighting):
        """Closest pair from mission t = 0 (as analysis/metrics.py), at the same tick."""
        t_wall, x, y, z, _, agent_id, _ = sighting
        t0_wall = self.t0_wall()
        self.last_pos[agent_id] = (t_wall, x, y, z) if t0_wall and t_wall >= t0_wall else None
        for other, pos in self.last_pos.items():
            if other != agent_id and pos and abs(pos[0] - t_wall) < 0.06:
                d = math.dist(pos[1:], (x, y, z))
                self.min_dist_m = d if self.min_dist_m is None else min(self.min_dist_m, d)

    def add_event(self, event):
        if event.get("t_wall") is None:
            return
        if self.base_wall is None:
            self.base_wall = math.floor(event["t_wall"])
        self.events.append(event)

    def advance_series(self):
        """One point per second, 1 s behind the newest row: other agents' rows may still come."""
        if self.latest_wall is None:
            return
        t_next = (self.base_wall + len(self.series_points)) * 1.0
        while t_next <= self.latest_wall - 1.0:
            self.series.advance(t_next, self.track_separation)
            self.series_points.append([t_next - self.base_wall,
                                       round(self.series.fresh_pct(t_next), 1),
                                       None if self.min_dist_m is None else round(self.min_dist_m, 1),
                                       self.zone_age_s(self.series, t_next)])
            t_next += 1.0

    # ---- what the browser gets
    def t0_wall(self):
        for e in self.events:
            if e["event"] == "start":
                return e["t0"]
        return self.meta.get("t0_wall")

    def rel(self, t_wall):
        return None if t_wall is None or self.base_wall is None else round(t_wall - self.base_wall, 3)

    def delta(self, cursor):
        """Everything new since cursor (None = from the start); returns (payload, new cursor)."""
        cursor = cursor or {"rows": {}, "events": 0, "series": 0}
        agents = {}
        for agent_id, rows in self.rows.items():
            sent = cursor["rows"].get(agent_id, 0)
            start = sent + (-sent) % SEND_EVERY
            agents[agent_id] = rows[start::SEND_EVERY]
            cursor["rows"][agent_id] = len(rows)
        events = [dict(e, t=self.rel(e["t_wall"])) for e in self.events[cursor["events"]:]]
        series = self.series_points[cursor["series"]:]
        cursor["events"], cursor["series"] = len(self.events), len(self.series_points)
        meta = self.meta
        payload = {"agents": agents, "events": events, "series": series, "radio": self.radio,
                   "t0": self.rel(self.t0_wall()), "kill_t": self.rel(meta.get("kill_wall")),
                   "victim": meta.get("kill_agent"), "outcome": meta.get("outcome"),
                   "mission": meta.get("mission"), "seed": meta.get("seed"),
                   "zone": meta.get("priority_zone"), "loss": self.metrics_loss(),
                   "end_t": self.rel(self.latest_wall)}
        return payload, cursor

    def metrics_loss(self):
        """analysis/metrics.py's loss figures (mission seconds), when the run has been analysed."""
        try:
            with open(os.path.join(self.dir, "metrics.json")) as f:
                loss = json.load(f).get("loss") or {}
        except (OSError, ValueError):
            return None
        keys = ("loss_s", "before_pct", "share_pct", "recovery_s")
        return {k: loss.get(k) for k in keys} if loss.get("before_pct") is not None else None

    def zone_age_s(self, coverage, t_wall):
        """Seconds since the whole priority zone was last seen: the age of its stalest cell, so
        no part of the zone is older. None: no zone, or not all of it seen yet."""
        zone = self.meta.get("priority_zone")
        if not zone:
            return None
        x0, y0, x1, y1 = zone
        inside = (CELL_X_M >= x0) & (CELL_X_M <= x1) & (CELL_Y_M >= y0) & (CELL_Y_M <= y1)
        oldest = coverage.last_t[inside].min()
        return None if np.isinf(oldest) else round(t_wall - oldest, 1)

    def heat(self, t_rel):
        t_wall = self.base_wall + t_rel
        self.view.advance(t_wall)
        return {"zone_age_s": self.zone_age_s(self.view, t_wall),
                "t": t_rel, "cols": len(COLUMN_X_M), "rows": len(ROW_Y_M), "cell_m": CELL_M,
                "age_s": self.view.ages_s(t_wall), "pct": round(self.view.fresh_pct(t_wall), 1)}


def radio_summary(stats):
    """Measured loss and mean delay, split into ground links and agent-agent links."""
    groups = {"ground": [0, 0, 0, 0.0], "agents": [0, 0, 0, 0.0]}  # sent, delivered, lost, delay sum
    for pair, s in stats.get("pairs", {}).items():
        ends = [int(v) for v in pair.split("->")]
        g = groups["ground" if set(ends) & set(GROUND_IDS) else "agents"]
        g[0] += s.get("sent", 0)
        g[1] += s.get("delivered", 0)
        g[2] += s.get("dropped_loss", 0)
        g[3] += s.get("delay_mean_s", 0.0) * s.get("delivered", 0)
    return {name: {"loss_pct": round(100.0 * lost / sent, 1) if sent else None,
                   "delay_ms": round(1000.0 * delay / delivered) if delivered else None}
            for name, (sent, delivered, lost, delay) in groups.items()}


def footprint_geometry():
    """The browser draws the footprint with sensor.py's numbers instead of its own."""
    return {"tan_half_across": sensor.TAN_HALF_ACROSS, "tan_half_along": sensor.TAN_HALF_ALONG,
            "area": config.AREA, "d_min_m": config.D_MIN_M, "window_s": WINDOW_S,
            "colors": DRONE_COLORS, "zone_limit_s": config.PRIORITY_REVISIT_S}
