"""SWARM_BACKUP=1: the onboard backup allocator (swarm/backup.py), and its off path.

In-process runs: 3 agents on fake vehicles, the ground, the lossy fake radio, stepped time.
"""
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))

import test_vision_off  # noqa: E402
from fakes import FakeRadio, SimClock  # noqa: E402
from sim.fake_vehicle import FakeVehicle  # noqa: E402
from swarm import backup, config, ground, protocol  # noqa: E402
from swarm import swarm_mav as mav  # noqa: E402
from swarm.agent import Agent  # noqa: E402
from swarm.ground import Ground, JsonLines  # noqa: E402

N = 3
TICK_S = 1.0 / config.CONTROL_HZ


class CutRadio(FakeRadio):
    """FakeRadio that also drops every copy from src to dst while (src, dst) is in `cuts`."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cuts = set()

    def submit(self, src, dst, msg):
        if dst == "all":
            for d in [n for n in self.nodes if n != src and (src, n) not in self.cuts]:
                super().submit(src, d, msg)
        elif (src, dst) not in self.cuts:
            super().submit(src, dst, msg)


class Swarm:
    """The ground and N agents in stepped time; the ground can die and restart."""

    def __init__(self, out_dir, seed=7, loss=config.RADIO_LOSS):
        self.out = str(out_dir)
        self.clock = SimClock()
        self.radio = CutRadio(self.clock, loss, config.RADIO_DELAY_S, seed=seed)
        self.ground_path = os.path.join(self.out, "ground.jsonl")
        self.ground = None
        self.start_ground()
        self.agents = {}
        self.silent = set()          # agents that no longer tick (dead)
        for a in range(1, N + 1):
            x, y = config.start_xy(a - 1, N)
            vehicle = FakeVehicle(a, (x, y, config.hold_alt(a)), clock=self.clock)
            self.agents[a] = Agent(a, vehicle, self.radio.link(a), self.out, n=N, mission_s=1e6)

    def start_ground(self):
        self.log = JsonLines(self.ground_path)
        self.ground = Ground(N, self.radio.link(config.GROUND_ID), self.log, mission_s=1e6,
                             previous_t0=ground.last_t0(self.ground_path))

    def kill_ground(self):
        self.ground = None
        self.log.close()

    def run(self, seconds):
        for _ in range(round(seconds / TICK_S)):
            self.clock.advance(TICK_S)
            now = self.clock()
            for a, agent in self.agents.items():
                if a not in self.silent:
                    agent.tick(now)
                    if agent.takeoff_thread is not None:
                        agent.takeoff_thread.join()
            if self.ground is not None:
                self.ground.step(now)

    def run_until_start(self, limit_s=120.0):
        while self.ground.t0 is None:
            assert limit_s > 0, "the mission never started"
            self.run(1.0)
            limit_s -= 1.0

    def epochs(self):
        return {a: agent.coverage.epoch for a, agent in self.agents.items()
                if a not in self.silent}

    def events(self, a, kind=None):
        with open(os.path.join(self.out, f"agent_{a}.events.jsonl")) as f:
            records = [json.loads(line) for line in f]
        return [r for r in records if kind is None or r["event"] == kind]

    def backup_log(self, a, kind=None):
        path = os.path.join(self.out, f"agent_{a}.ground.jsonl")
        if not os.path.exists(path):
            return []
        with open(path) as f:
            records = [json.loads(line) for line in f]
        return [r for r in records if kind is None or r["event"] == kind]

    def close(self):
        for agent in self.agents.values():
            agent.close()
        if self.ground is not None:
            self.log.close()


@pytest.fixture
def backup_on(monkeypatch):
    monkeypatch.setattr(config, "BACKUP", True)


# ---- off path -----------------------------------------------------------------------------

def test_off_path_logs_are_unchanged(tmp_path, monkeypatch):
    """The golden in-process run of test_vision_off, hash for hash, with the backup off."""
    monkeypatch.setattr(config, "BACKUP", False)
    monkeypatch.setattr(config, "VISION", False, raising=False)
    assert test_vision_off.run_scenario(str(tmp_path)) == test_vision_off.GOLDEN


def test_off_nobody_takes_over_when_the_ground_dies(tmp_path, monkeypatch):
    monkeypatch.setattr(config, "BACKUP", False)
    swarm = Swarm(tmp_path)
    swarm.run_until_start()
    epochs = swarm.epochs()
    swarm.kill_ground()
    swarm.run(3 * config.GROUND_LOST_S)
    swarm.close()
    assert swarm.epochs() == epochs
    for a, agent in swarm.agents.items():
        assert agent.backup is None
        assert {m.get_type() for m in agent.link.sent} <= {"SWARM_STATE", "SWARM_INFEASIBLE"}
        assert not os.path.exists(tmp_path / f"agent_{a}.ground.jsonl")
        assert all("sender" not in e for e in swarm.events(a, "assign"))


# ---- election and takeover ----------------------------------------------------------------

def test_lowest_alive_takes_over_after_ground_lost_s(tmp_path, backup_on):
    swarm = Swarm(tmp_path)
    swarm.run_until_start()
    t0, old_epoch = swarm.ground.t0, swarm.ground.epoch
    swarm.run(5.0)
    assert all(not agent.backup.running for agent in swarm.agents.values())
    killed_at = swarm.clock()
    swarm.kill_ground()
    swarm.run(config.GROUND_LOST_S - 4.0)           # the last ground message: <= 4 s before
    assert all(not agent.backup.running for agent in swarm.agents.values())
    swarm.run(ground.RESTART_LISTEN_S + 13.0)       # seed 7 loses 4 ASSIGNs to agent 2 in a row
    [start] = swarm.events(1, "backup_start")
    assert start["boss_silent_s"] == pytest.approx(config.GROUND_LOST_S, abs=TICK_S + 1e-6)
    in_flight_s = config.RADIO_DELAY_S + 2 * TICK_S     # the last copy lands after the kill
    assert start["t_wall"] - killed_at <= config.GROUND_LOST_S + in_flight_s
    assert start["alive"] == [1, 2, 3] and start["t0"] == pytest.approx(t0, abs=1e-6)
    assert swarm.events(2, "backup_start") == swarm.events(3, "backup_start") == []
    [plan] = swarm.backup_log(1, "assign")
    assert plan["trigger"] == "backup" and plan["epoch"] > old_epoch + ground.EPOCH_STRIDE
    assert sorted(plan["assignments"]) == ["1", "2", "3"]
    assert swarm.agents[1].backup.ground.t0 == pytest.approx(t0, abs=1e-6)   # t0 kept
    assert set(swarm.epochs().values()) == {plan["epoch"]}
    assert swarm.events(3, "assign")[-1]["sender"] == 1
    swarm.close()


def test_backup_declares_a_dead_agent_lost_and_replans(tmp_path, backup_on):
    swarm = Swarm(tmp_path)
    swarm.run_until_start()
    swarm.kill_ground()
    swarm.run(config.GROUND_LOST_S + ground.RESTART_LISTEN_S + 3.0)
    swarm.silent.add(3)
    swarm.run(config.LOST_S + 6.0)
    lost = [e for e in swarm.backup_log(1, "liveness") if e["to"] == "lost"]
    assert [e["agent"] for e in lost] == [3]
    assigns = swarm.backup_log(1, "assign")
    assert [e["trigger"] for e in assigns] == ["backup", "lost"]
    assert sorted(assigns[-1]["assignments"]) == ["1", "2"]
    assert swarm.epochs() == {1: assigns[-1]["epoch"], 2: assigns[-1]["epoch"]}
    swarm.close()


def test_next_lowest_takes_over_when_the_backup_dies(tmp_path, backup_on):
    swarm = Swarm(tmp_path)
    swarm.run_until_start()
    swarm.kill_ground()
    swarm.run(config.GROUND_LOST_S + ground.RESTART_LISTEN_S + 3.0)
    epoch_1 = swarm.agents[1].backup.ground.epoch
    swarm.silent.add(1)                    # the backup's drone dies with its allocator
    swarm.run(config.LOST_S + ground.RESTART_LISTEN_S + 3.0)
    [start] = swarm.events(2, "backup_start")
    assert start["alive"] == [2, 3]
    [plan] = swarm.backup_log(2, "assign")
    assert plan["epoch"] > epoch_1 and sorted(plan["assignments"]) == ["2", "3"]
    [lost] = swarm.backup_log(2, "liveness")
    assert lost["agent"] == 1 and lost["reason"] == "backup_plan"
    assert swarm.epochs() == {2: plan["epoch"], 3: plan["epoch"]}
    swarm.close()


def test_hand_back_when_the_ground_restarts(tmp_path, backup_on):
    swarm = Swarm(tmp_path)
    swarm.run_until_start()
    t0 = swarm.ground.t0
    swarm.kill_ground()
    swarm.run(config.GROUND_LOST_S + ground.RESTART_LISTEN_S + 3.0)
    backup_epoch = swarm.agents[1].backup.ground.epoch
    swarm.start_ground()
    swarm.run(ground.RESTART_LISTEN_S + 4.0)
    [stop] = swarm.events(1, "backup_stop")
    assert stop["reason"] == "ground_back" and stop["epoch"] == backup_epoch
    assert not swarm.agents[1].backup.running
    assert swarm.ground.t0 == t0
    assert swarm.ground.epoch > backup_epoch + ground.EPOCH_STRIDE - 1
    assert set(swarm.epochs().values()) == {swarm.ground.epoch}
    assert all(swarm.agents[a].assigner == config.GROUND_ID for a in swarm.agents)
    swarm.run(2 * config.GROUND_LOST_S)    # the ground stays: no second takeover
    assert len(swarm.events(1, "backup_start")) == 1
    swarm.close()


def mute(swarm, agent_id, on):
    for dst in [config.GROUND_ID] + list(swarm.agents):
        if on:
            swarm.radio.cuts.add((agent_id, dst))
        else:
            swarm.radio.cuts.discard((agent_id, dst))


def test_two_backups_converge_on_the_lower_id(tmp_path, backup_on):
    """Agent 1's transmitter is dead while the ground dies: agent 2 hears neither, so it takes
    over, and agent 1 (which hears everyone) does too. Agent 2 re-plans, agent 1 sees the
    higher epoch in agent 3's STATE and overtakes. When agent 1 is heard again, agent 2 stops
    and everyone flies agent 1's plan."""
    swarm = Swarm(tmp_path)
    swarm.run_until_start()
    mute(swarm, 1, True)
    swarm.kill_ground()
    swarm.run(config.GROUND_LOST_S + ground.RESTART_LISTEN_S + 5.0)
    assert swarm.agents[1].backup.running and swarm.agents[2].backup.running
    assert swarm.agents[3].assigner == 2                 # agent 1 is not heard
    b1 = swarm.agents[1].backup.ground
    swarm.agents[2].backup.ground.replan(swarm.clock(), "lost")   # as if it lost someone
    swarm.run(3.0)
    assert b1.epoch > swarm.agents[2].backup.ground.epoch      # agent 1 overtook, unheard
    mute(swarm, 1, False)
    swarm.run(5.0)
    assert swarm.agents[1].backup.running and not swarm.agents[2].backup.running
    assert swarm.events(2, "backup_stop")[0]["reason"] in ("outranked", "lower_backup")
    assert {agent.assigner for agent in swarm.agents.values()} == {1}
    epoch = swarm.agents[1].backup.ground.epoch
    assert set(swarm.epochs().values()) == {epoch}
    swarm.run(2 * config.GROUND_LOST_S)                  # and it stays that way
    assert len(swarm.events(2, "backup_start")) == 1
    assert swarm.agents[1].backup.ground.epoch == epoch
    swarm.close()


def test_simultaneous_backups_tie_goes_to_the_lower_id(tmp_path, backup_on):
    """Agent 2 cannot hear agent 1 (one way only): both plan the same jumped epoch and agent
    3 keeps agent 1's plan by the tie rule; nobody overtakes on a tie."""
    swarm = Swarm(tmp_path)
    swarm.run_until_start()
    swarm.radio.cuts.add((1, 2))
    swarm.kill_ground()
    swarm.run(config.GROUND_LOST_S + ground.RESTART_LISTEN_S + 5.0)
    b1, b2 = swarm.agents[1].backup.ground, swarm.agents[2].backup.ground
    assert b1.epoch == b2.epoch and swarm.agents[3].assigner == 1
    swarm.radio.cuts.clear()
    swarm.run(5.0)
    assert not swarm.agents[2].backup.running
    assert {agent.assigner for agent in swarm.agents.values()} == {1}
    assert set(swarm.epochs().values()) == {swarm.agents[1].backup.ground.epoch}
    swarm.close()


# ---- unit: epochs and ties ------------------------------------------------------------------

def wire(src, msg):
    packer = mav.MAVLink(None, srcSystem=src, srcComponent=protocol.COMPONENT)
    return packer.decode(bytearray(msg.pack(packer)))


class NullRadio:
    def __init__(self):
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)


class Events:
    def __init__(self):
        self.records = []

    def write(self, now, event, **fields):
        self.records.append(dict(fields, event=event))


def running_backup(tmp_path, agent_id=1, epoch=5):
    """A Backup that took over: it and the higher ids report `epoch`; the ground last spoke at
    0; lower ids are never heard."""
    b = backup.Backup(agent_id, N, NullRadio(), str(tmp_path), Events(), mission_s=1e6)
    b.on_message(wire(config.GROUND_ID, protocol.start(500.0)), 0.0)
    now = config.GROUND_LOST_S
    while b.ground is None or b.ground.planned_at is None:
        for a in range(agent_id, N + 1):
            msg = wire(a, protocol.state(0.0, (100.0 * a, 0.0, 50.0), (0, 0, 0), epoch, 0,
                                         "cover"))
            b.on_own(msg) if a == agent_id else b.on_message(msg, now)
        b.step(now, "cover", 500.0)
        now += 0.2
    return b, now


def test_not_armed_before_the_ground_is_heard(tmp_path):
    b = backup.Backup(1, N, NullRadio(), str(tmp_path), Events())
    b.step(1e6, "cover", None)
    assert not b.running


def test_an_agent_on_a_higher_epoch_makes_the_backup_overtake(tmp_path, backup_on):
    b, now = running_backup(tmp_path)
    own = b.ground.epoch
    assert own == 5 + ground.EPOCH_STRIDE + 1
    report = wire(3, protocol.state(0.0, (300.0, 0.0, 50.0), (0, 0, 0), own, 0, "cover"))
    b.on_message(report, now)                       # our own epoch: nothing to do
    b.step(now, "cover", 500.0)
    assert b.ground.epoch == own
    report = wire(3, protocol.state(0.0, (300.0, 0.0, 50.0), (0, 0, 0), own + 3, 0, "cover"))
    b.on_message(report, now)
    b.step(now, "cover", 500.0)
    assert b.ground.epoch == own + 3 + ground.EPOCH_STRIDE
    with open(tmp_path / "agent_1.ground.jsonl") as f:
        assert [json.loads(line)["trigger"] for line in f if '"assign"' in line] == \
            ["backup", "overtake"]


def test_overtake_is_off_without_the_switch(tmp_path, monkeypatch):
    """The rule is in Ground: with SWARM_BACKUP off a higher reported epoch changes nothing."""
    monkeypatch.setattr(config, "BACKUP", False)
    b, now = running_backup(tmp_path)
    own = b.ground.epoch
    b.on_message(wire(3, protocol.state(0.0, (300.0, 0.0, 50.0), (0, 0, 0), own + 3, 0,
                                        "cover")), now)
    b.step(now, "cover", 500.0)
    assert b.ground.epoch == own


def test_a_lower_id_assign_stops_a_higher_backup(tmp_path):
    b, now = running_backup(tmp_path, agent_id=2)
    b.on_message(wire(1, protocol.assign(2, 3, [0.0] * 4, [], 0)), now)
    assert not b.running
    assert b.events.records[-1] == {"event": "backup_stop", "reason": "lower_backup",
                                    "epoch": 5 + ground.EPOCH_STRIDE + 1}
    b.step(now + 1.0, "cover", 500.0)      # the lower backup now is the boss: no restart
    assert not b.running


def test_backup_handles_infeasible_like_the_ground(tmp_path):
    b, now = running_backup(tmp_path)
    epoch = b.ground.epoch
    xy = b.ground.assignments[3].slots[0]
    b.on_message(wire(3, protocol.infeasible(epoch, 0, xy, target=1)), now)
    b.step(now, "cover", 500.0)
    assert b.ground.epoch == epoch + 1 and len(b.ground.blacklist) == 1


def test_agent_takes_an_equal_epoch_only_from_a_lower_sender(tmp_path, backup_on):
    swarm = Swarm(tmp_path, loss=0.0)
    agent = swarm.agents[3]
    swarm.run(1.0)
    st = agent.vehicle.state()
    plan = {"share": [0.0] * 4, "slots": [(150.0, 100.0)], "loop_from": 0}
    agent._on_assign(wire(2, protocol.assign(3, 9, **plan)), 0.0, st)
    assert agent.assigner == 2
    plan["slots"] = [(50.0, 100.0)]
    agent._on_assign(wire(3, protocol.assign(3, 9, **plan)), 0.0, st)
    assert agent.assigner == 2                       # tie, higher sender: kept
    agent._on_assign(wire(1, protocol.assign(3, 9, **plan)), 0.0, st)
    assert agent.assigner == 1 and agent.coverage.slots == [(50.0, 100.0)]
    agent._on_assign(wire(2, protocol.assign(3, 8, **plan)), 0.0, st)
    assert agent.assigner == 1                       # older epoch: ignored
    agent._on_assign(wire(config.GROUND_ID, protocol.assign(3, 9, **plan)), 0.0, st)
    assert agent.assigner == config.GROUND_ID        # the ground wins every tie
    swarm.close()


# ---- analysis: the backup logs count -------------------------------------------------------

def test_metrics_read_the_loss_and_replans_from_a_backup_log(tmp_path):
    """Agent 1 (the backup) dies at 80 s; agent 2 takes over and re-plans without it at 95 s."""
    import numpy as np
    from analysis import logs, metrics
    from test_metrics import T0_WALL, agent_log, assign, make_run, track, write_run_dir
    agents = {a: agent_log(track(0.0, 80.0 if a == 1 else 180.0, 50.0 + 100.0 * (a - 1),
                                 150.0)) for a in (1, 2, 3)}
    for a in (2, 3):
        agents[a]["epoch"] = np.where(agents[a]["t"] >= 96.0, 1002.0, 1.0)
    first = dict(assign(None, 1, "start", [1, 2, 3]), t_wall=T0_WALL - 30.0)
    plan = {"2": {"share": [0, 150, 0, 300], "slots": [], "loop_from": 0},
            "3": {"share": [150, 300, 0, 300], "slots": [], "loop_from": 0}}
    takeover = dict(assign(95.0, 1002, "backup", [1, 2, 3], plan), t_wall=T0_WALL + 95.0)
    declared = {"t_wall": T0_WALL + 95.0, "t_mission": 95.0, "event": "liveness", "agent": 1,
                "status": "cover", "reason": "backup_plan", "from": "alive", "to": "lost"}
    run = make_run(agents, [first], kill_agent=1, kill_wall=T0_WALL + 80.0)
    out = str(tmp_path / "run")
    write_run_dir(out, run)
    with open(os.path.join(out, "agent_2.ground.jsonl"), "w") as f:
        f.writelines(json.dumps(e) + "\n" for e in (takeover, declared))
    assert [e.get("backup") for e in logs.load_run(out).ground] == [None, 2, 2]
    m = metrics.analyse(out)
    assert m["loss"]["lost_declared_s"] == 95.0
    assert m["loss"]["milestones_s"]["lost_declared"] == 15.0
    assert m["loss"]["milestones_s"]["assignment_received"] == 16.0
    assert m["churn"]["replans_by_trigger"] == {"backup": 1}
