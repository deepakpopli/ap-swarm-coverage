"""Read a run directory (runs/<name>/) written by the agents, the ground station and the runner.

Files (see the task contract):
  agent_<id>.csv   10 Hz rows, header AGENT_COLUMNS. The agent's own TRUE state, area ENU metres,
                   z up, yaw rad compass (0 north, clockwise). t_mission empty before mission start.
                   A SIGKILLed agent's file just ends, possibly with a truncated last line.
  ground.jsonl     JSON lines: "start", "liveness", "infeasible", "assign", "blacklist" events.
  meta.json        {"seed","n","backend","t0_wall","kill_agent","kill_t_mission","mission_s","radio"}
  radio.jsonl      radio counters (optional).

Everything here is tolerant: a missing optional file or field gives a warning string in
`Run.warnings`, never a crash. Only "no agent_*.csv at all" is fatal.
"""
import csv
import glob
import json
import math
import os
import re

import numpy as np

AGENT_COLUMNS = ["t_wall", "t_mission", "x", "y", "z", "vx", "vy", "vz", "yaw", "status",
                 "epoch", "slot", "cmd_vx", "cmd_vy", "cmd_z", "nom_vx", "nom_vy",
                 "min_nbr_dist", "min_nbr_age", "flags"]
TEXT_COLUMNS = ("status", "flags")
FLYING_STATUSES = ("takeoff", "ready", "cover", "hold", "fallback")   # everything but "ground"


class RunLoadError(Exception):
    pass


class AgentLog:
    """One agent's log as numpy columns (floats; NaN where empty) plus text columns as lists."""

    def __init__(self, agent_id, cols, n_bad_lines):
        self.id = agent_id
        self.cols = cols
        self.n_bad_lines = n_bad_lines

    def __getitem__(self, name):
        return self.cols[name]

    def __len__(self):
        return len(self.cols["t_wall"])


class Run:
    def __init__(self, path):
        self.path = path
        self.name = os.path.basename(os.path.normpath(path))
        self.warnings = []
        self.agents = {}          # id -> AgentLog
        self.meta = {}
        self.ground = []          # list of dicts
        self.radio = []           # list of dicts

    def warn(self, msg):
        self.warnings.append(msg)

    def meta_get(self, key, default=None):
        v = self.meta.get(key, default)
        return default if v is None else v

    def events(self, kind):
        return [e for e in self.ground if e.get("event") == kind]


def _to_float(s):
    try:
        return float(s) if s not in ("", None) else math.nan
    except ValueError:
        return math.nan


def read_agent_csv(path, agent_id, warn):
    """Parse one agent CSV. Rows with the wrong field count (e.g. a SIGKILL-truncated last
    line) or an unparsable t_wall are skipped and counted."""
    with open(path, newline="") as f:
        text = f.read()
    lines = text.splitlines()
    if not lines:
        raise RunLoadError(f"{path}: empty file")
    header = next(csv.reader([lines[0]]))
    missing = [c for c in AGENT_COLUMNS if c not in header]
    if missing:
        warn(f"{os.path.basename(path)}: missing columns {missing} (filled with NaN/empty)")
    rows, bad = [], 0
    for i, rec in enumerate(csv.reader(lines[1:])):
        if len(rec) != len(header):
            bad += 1
            continue
        d = dict(zip(header, rec))
        if math.isnan(_to_float(d.get("t_wall"))):
            bad += 1
            continue
        rows.append(d)
    if not text.endswith("\n") and bad:
        warn(f"{os.path.basename(path)}: last line truncated (agent killed?); skipped")
    elif bad:
        warn(f"{os.path.basename(path)}: skipped {bad} malformed rows")
    cols = {}
    for c in AGENT_COLUMNS:
        if c in TEXT_COLUMNS:
            cols[c] = [r.get(c, "") or "" for r in rows]
        else:
            cols[c] = np.array([_to_float(r.get(c)) for r in rows], dtype=float)
    return AgentLog(agent_id, cols, bad)


def read_jsonl(path, warn):
    out = []
    with open(path) as f:
        for i, line in enumerate(f):
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                warn(f"{os.path.basename(path)} line {i + 1}: not valid JSON (truncated?), skipped")
    return out


def load_run(path):
    """Load a run directory. Raises RunLoadError only if there is no usable agent log."""
    if not os.path.isdir(path):
        raise RunLoadError(f"{path}: not a directory")
    run = Run(path)
    for p in sorted(glob.glob(os.path.join(path, "agent_*.csv"))):
        m = re.match(r"agent_(\d+)\.csv$", os.path.basename(p))
        if not m:
            continue
        try:
            run.agents[int(m.group(1))] = read_agent_csv(p, int(m.group(1)), run.warn)
        except RunLoadError as e:
            run.warn(str(e))
    if not run.agents:
        raise RunLoadError(f"{path}: no readable agent_<id>.csv files")

    meta_p = os.path.join(path, "meta.json")
    if os.path.exists(meta_p):
        try:
            with open(meta_p) as f:
                run.meta = json.load(f)
        except json.JSONDecodeError as e:
            run.warn(f"meta.json unreadable ({e}); kill time/mission length unknown")
    else:
        run.warn("meta.json missing: kill time, victim and mission length unknown")

    for name, attr in (("ground.jsonl", "ground"), ("radio.jsonl", "radio")):
        p = os.path.join(path, name)
        if os.path.exists(p):
            setattr(run, attr, read_jsonl(p, run.warn))
        else:
            run.warn(f"{name} missing: metrics that need it are reported as null")
    return run
