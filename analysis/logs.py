"""Read one run directory: agent logs, ground events, meta.json and the radio totals."""
import csv
import glob
import json
import os

import numpy as np

from swarm.protocol import AGENT_CSV_COLUMNS

TEXT_COLUMNS = ("status", "flags")


class Run:
    """One run in memory. All times are mission seconds (t = 0 when every agent is on station)."""

    def __init__(self, name, meta, agents, ground, radio=None):
        self.name = name
        self.meta = meta
        self.agents = agents          # {agent id: {column name: array or list}}
        self.ground = ground          # ground.jsonl records, in file order
        self.radio = radio            # the radio's final "stats" record
        self.end_s = meta["mission_s"]
        self.victim = meta["kill_agent"]
        self.loss_s = None
        if self.victim is not None:
            self.loss_s = meta["kill_wall"] - meta["t0_wall"]

    def events(self, kind):
        return [e for e in self.ground if e["event"] == kind]

    def live_until(self, agent_id):
        """The victim stops counting at the loss; after the kill its log shows a dead vehicle."""
        if agent_id == self.victim:
            return self.loss_s
        return self.end_s

    def lost_declared_s(self, agent_id):
        """When the ground first declared this agent lost, or None."""
        for e in self.events("liveness"):
            if e["agent"] == agent_id and e["to"] == "lost":
                return e["t_mission"]
        return None


def read_agent_log(path, t0_wall):
    """Columns of one agent_<id>.csv, plus "t" = t_wall - t0_wall for every row.

    "t" is the mission time; unlike t_mission it also exists before the start (negative).
    A SIGKILL can cut the last line. Complete lines end with a newline, so the text after the
    last newline is dropped.
    """
    with open(path) as f:
        lines = f.read().split("\n")[:-1]
    rows = list(csv.DictReader(lines))
    log = {}
    for name in AGENT_CSV_COLUMNS:
        values = [row[name] for row in rows]
        if name in TEXT_COLUMNS:
            log[name] = values
        else:
            log[name] = np.array([float(v) if v else np.nan for v in values])
    log["t"] = log["t_wall"] - t0_wall
    return log


def read_jsonl(path):
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def load_run(run_dir):
    with open(os.path.join(run_dir, "meta.json")) as f:
        meta = json.load(f)
    agents = {}
    for path in glob.glob(os.path.join(run_dir, "agent_*.csv")):
        agent_id = int(os.path.basename(path)[len("agent_"):-len(".csv")])
        agents[agent_id] = read_agent_log(path, meta["t0_wall"])
    ground = read_jsonl(os.path.join(run_dir, "ground.jsonl"))
    stats = [r for r in read_jsonl(os.path.join(run_dir, "radio.jsonl")) if r["event"] == "stats"]
    name = os.path.basename(os.path.normpath(run_dir))
    return Run(name, meta, dict(sorted(agents.items())), ground, stats[-1])
