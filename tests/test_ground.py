"""Ground station logic, in-process: a fake link instead of sockets and a simulated clock."""
import json

import pytest

from swarm import config, ground
from swarm.allocator import slot_key
from swarm.ground import Ground, JsonLines

TICK_S = 0.2


class FakeLink:
    def __init__(self):
        self.inbox = []
        self.sent = []

    def recv_all(self):
        messages, self.inbox = self.inbox, []
        return messages

    def send(self, dst, mtype, **body):
        self.sent.append((dst, mtype, body))


class FakeAgent:
    """Talks STATE every tick while `talking`; adopts the newest ASSIGN and, if `on_loop`,
    reports a slot inside its loop, else the slot before it."""

    def __init__(self, agent_id, n):
        self.id = agent_id
        self.xy = list(config.start_xy(agent_id - 1, n))
        self.status = "ready"
        self.talking = True
        self.on_loop = True
        self.epoch = 0
        self.slot = -1

    def state(self):
        return {"src": self.id, "type": "STATE", "pos": self.xy + [50.0], "epoch": self.epoch,
                "slot": self.slot, "status": self.status}

    def hear(self, body):
        self.epoch = body["epoch"]
        self.slot = body["loop_from"] if self.on_loop else body["loop_from"] - 1


class Sim:
    def __init__(self, tmp_path, n=3, previous_t0=None):
        self.path = tmp_path / "ground.jsonl"
        self.link = FakeLink()
        self.ground = Ground(n, self.link, JsonLines(self.path), mission_s=1e6,
                             previous_t0=previous_t0)
        self.agents = {a: FakeAgent(a, n) for a in range(1, n + 1)}
        self.now = 1000.0

    def run(self, seconds):
        for _ in range(round(seconds / TICK_S)):
            for agent in self.agents.values():
                if agent.talking:
                    self.link.inbox.append(agent.state())
            self.ground.step(self.now)
            for dst, mtype, body in self.link.sent:
                if mtype == "ASSIGN" and self.agents[dst].talking:
                    self.agents[dst].hear(body)
            self.link.sent.clear()
            self.now += TICK_S

    def infeasible(self, agent_id, epoch, xy):
        self.link.inbox.append({"src": agent_id, "type": "INFEASIBLE", "epoch": epoch,
                                "slot": 0, "slot_xy": xy})

    def events(self, kind):
        with open(self.path) as f:
            return [r for r in map(json.loads, f) if r["event"] == kind]

    def plan(self, agent_id):
        return self.ground.assignments[agent_id]


def test_start_when_all_ready_and_on_station(tmp_path):
    sim = Sim(tmp_path)
    sim.run(1.0)
    assert [r["trigger"] for r in sim.events("assign")] == ["start"]
    assert sim.events("assign")[0]["changed"] == [1, 2, 3]
    [start] = sim.events("start")
    assert start["late"] == [] and start["t0"] == pytest.approx(start["t_wall"] + 1.0)
    assert len(sim.events("ready")) == 3


def test_ready_deadline_then_late_agent_joins(tmp_path):
    sim = Sim(tmp_path)
    sim.agents[3].status = "takeoff"
    sim.run(ground.READY_DEADLINE_S - 1.0)
    assert sim.events("assign") == []
    sim.run(2.0)
    [first] = sim.events("assign")
    assert sorted(first["assignments"]) == ["1", "2"]
    sim.agents[3].status = "ready"
    sim.run(ground.REJOIN_S + 1.0)
    assert [r["trigger"] for r in sim.events("assign")] == ["start", "rejoin"]
    assert sorted(sim.ground.assignments) == [1, 2, 3]


def test_on_station_deadline(tmp_path):
    sim = Sim(tmp_path)
    sim.agents[3].on_loop = False
    sim.run(ground.ON_STATION_DEADLINE_S - 1.0)
    assert sim.events("start") == []
    sim.run(2.0)
    assert sim.events("start")[0]["late"] == [3]


def test_lost_then_replan(tmp_path):
    sim = Sim(tmp_path)
    sim.run(2.0)
    sim.agents[2].talking = False
    sim.run(config.LOST_S + 1.0)
    moves = [(r["agent"], r["from"], r["to"]) for r in sim.events("liveness")]
    assert moves == [(2, "alive", "suspect"), (2, "suspect", "lost")]
    replan = sim.events("assign")[-1]
    assert replan["trigger"] == "lost" and sorted(replan["assignments"]) == ["1", "3"]


def test_rejoin_needs_steady_contact(tmp_path):
    sim = Sim(tmp_path)
    sim.run(2.0)
    sim.agents[2].talking = False
    sim.run(config.LOST_S + 1.0)
    for _ in range(5):                    # heard for 1 s, silent for 2.4 s: never steady
        sim.agents[2].talking = True
        sim.run(1.0)
        sim.agents[2].talking = False
        sim.run(2.4)
    assert sim.events("assign")[-1]["trigger"] == "lost"
    sim.agents[2].talking = True
    sim.run(ground.REJOIN_S + 0.4)
    assert sim.events("assign")[-1]["trigger"] == "rejoin"
    assert sorted(sim.ground.assignments) == [1, 2, 3]


def test_failsafe_is_lost_at_once(tmp_path):
    sim = Sim(tmp_path)
    sim.run(2.0)
    sim.agents[2].status = "failsafe"
    sim.run(0.4)
    [record] = sim.events("liveness")
    assert (record["agent"], record["to"], record["status"]) == (2, "lost", "failsafe")
    assert sorted(sim.ground.assignments) == [1, 3]


def test_infeasible_survives_unrelated_replan_and_repeats_are_ignored(tmp_path):
    sim = Sim(tmp_path)
    sim.run(2.0)
    epoch_1 = sim.ground.epoch
    xy_1 = sim.plan(1).slots[2]
    sim.infeasible(3, epoch_1, sim.plan(3).slots[2])     # agent 3's report re-plans first
    sim.run(0.2)
    assert sim.ground.epoch == epoch_1 + 1
    for _ in range(3):                                   # then agent 1's, naming the old epoch
        sim.infeasible(1, epoch_1, xy_1)
        sim.run(0.2)
    assert sim.ground.epoch == epoch_1 + 2
    reports = [(r["agent"], r["accepted"]) for r in sim.events("infeasible")]
    assert reports == [(3, True), (1, True), (1, False), (1, False)]
    assert slot_key(xy_1) not in [slot_key(p) for p in sim.plan(1).slots]


def test_blacklist_holds_for_cooldown_and_expiry_does_not_replan(tmp_path):
    sim = Sim(tmp_path)
    sim.run(2.0)
    xy = sim.plan(1).slots[2]
    sim.infeasible(1, sim.ground.epoch, xy)
    sim.run(1.0)
    for _ in range(2):                    # re-plans while the pair is blacklisted
        sim.agents[3].talking = False
        sim.run(config.LOST_S + 1.0)
        sim.agents[3].talking = True
        sim.run(ground.REJOIN_S + 1.0)
        assert slot_key(xy) not in [slot_key(p) for p in sim.plan(1).slots]
    epoch = sim.ground.epoch
    sim.run(config.BLACKLIST_COOLDOWN_S)
    assert sim.events("blacklist")[-1]["pairs"] == []
    assert sim.ground.epoch == epoch


def test_restart_jumps_epoch_and_keeps_t0(tmp_path):
    sim = Sim(tmp_path, previous_t0=500.0)
    for agent in sim.agents.values():
        agent.status, agent.epoch, agent.slot = "cover", 7, 4
    sim.run(ground.RESTART_LISTEN_S + 0.4)
    assert sim.ground.epoch == 7 + ground.EPOCH_STRIDE + 1
    assert sim.ground.t0 == 500.0
    assert sim.events("assign")[0]["trigger"] == "restart"
    assert sim.events("start") == []      # the mission clock was already running


def test_last_t0_reads_the_old_log(tmp_path):
    path = tmp_path / "ground.jsonl"
    path.write_text(json.dumps({"event": "start", "t0": 500.0}) + "\n" + '{"event": "ass')
    assert ground.last_t0(str(path)) == 500.0
    assert ground.last_t0(str(tmp_path / "missing.jsonl")) is None
