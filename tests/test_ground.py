"""Ground station logic, in-process: a fake link instead of sockets and a simulated clock."""
import json

import pytest

from swarm import config, ground, protocol
from swarm import swarm_mav as mav
from swarm.allocator import slot_key
from swarm.ground import Ground, JsonLines

TICK_S = 0.2


def wire(src, msg):
    """msg as `src` would send it, packed and decoded."""
    packer = mav.MAVLink(None, srcSystem=src, srcComponent=protocol.COMPONENT)
    return packer.decode(bytearray(msg.pack(packer)))


class FakeLink:
    def __init__(self):
        self.inbox = []
        self.sent = []

    def recv_all(self):
        messages, self.inbox = self.inbox, []
        return messages

    def send(self, msg):
        self.sent.append(wire(config.GROUND_ID, msg))


class FakeAgent:
    """Talks STATE every tick while `talking`. On a new ASSIGN it starts at loop_from; if
    `on_loop` it reaches one slot per tick from then on, else it never moves on."""

    def __init__(self, agent_id, n):
        self.id = agent_id
        self.xy = list(config.start_xy(agent_id - 1, n))
        self.status = "ready"
        self.talking = True
        self.on_loop = True
        self.epoch = 0
        self.slot = -1

    def state(self):
        msg = wire(self.id, protocol.state(0.0, self.xy + [50.0], [0.0] * 3, self.epoch,
                                           self.slot, self.status))
        if self.on_loop and self.epoch > 0:
            self.slot += 1
        return msg

    def hear(self, msg):
        if msg.epoch != self.epoch:
            self.epoch = msg.epoch
            self.slot = msg.loop_from


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
            for msg in self.link.sent:
                if msg.get_type() == "SWARM_ASSIGN" and self.agents[msg.target_system].talking:
                    self.agents[msg.target_system].hear(msg)
            self.link.sent.clear()
            self.now += TICK_S

    def infeasible(self, agent_id, epoch, xy):
        self.link.inbox.append(wire(agent_id, protocol.infeasible(epoch, 0, xy)))

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
    assert start["late"] == []
    assert start["t0"] == pytest.approx(start["t_wall"] + config.COVERAGE_WINDOW_S)
    assert start["t_mission"] == pytest.approx(-config.COVERAGE_WINDOW_S)
    assert len(sim.events("ready")) == 3


def test_spare_agent_does_not_delay_the_start(tmp_path):
    """N = 5 at 50 m: agent 3 gets no slots, so it never reaches a loop slot."""
    sim = Sim(tmp_path, n=5)
    sim.agents[3].on_loop = False
    sim.run(1.0)
    assert sim.ground.assignments[3].slots == []
    [start] = sim.events("start")
    assert start["late"] == []


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


def test_lost_agent_is_told_it_has_no_plan(tmp_path):
    """A lost agent that still hears (dead transmitter) must stop flying its old lanes."""
    sim = Sim(tmp_path)
    sim.run(2.0)
    sim.agents[2].talking = False
    sim.run(config.LOST_S + 1.0)
    sim.ground.step(sim.now + 1.0)
    assigns = {m.target_system: m for m in sim.link.sent if m.get_type() == "SWARM_ASSIGN"}
    assert assigns[2].slot_count == 0 and assigns[2].epoch == sim.ground.epoch
    assert assigns[1].slot_count and assigns[3].slot_count


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


def test_swap_hook_reverses_the_groups_once(tmp_path):
    sim = Sim(tmp_path)
    sim.ground.swap_at = 5.0 - config.COVERAGE_WINDOW_S   # 5 s after START
    sim.run(2.0)
    west_first = [sim.plan(a).share[0] for a in (1, 2, 3)]
    sim.run(5.0)
    assert [r["trigger"] for r in sim.events("assign")] == ["start", "swap"]
    assert [sim.plan(a).share[0] for a in (1, 2, 3)] == west_first[::-1]
