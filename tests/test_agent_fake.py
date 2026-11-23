"""Agent process tests with the fake backend: stepped time, plus one real-time run (slow)."""
import csv
import json
import threading
import time

import pytest

from fakes import FakeRadio, SimClock
from sim.fake_vehicle import FakeVehicle
from swarm import config, protocol
from swarm.agent import Agent

TICK_S = 1.0 / config.CONTROL_HZ


class CountingVehicle(FakeVehicle):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setpoints = 0

    def send_setpoint(self, vx, vy, z):
        self.setpoints += 1
        super().send_setpoint(vx, vy, z)


class Rig:
    """One agent on the south edge, a lossy radio and a ground link, in stepped time."""

    def __init__(self, log_dir, start_z=0.0, clock=None, seed=0):
        self.clock = clock or SimClock()
        self.radio = FakeRadio(self.clock, config.RADIO_LOSS, config.RADIO_DELAY_S, seed)
        self.ground = self.radio.link(config.GROUND_ID)
        x, y = config.start_xy(0, 1)
        self.vehicle = CountingVehicle(1, (x, y, start_z), clock=self.clock)
        self.link = self.radio.link(1)
        self.agent = Agent(1, self.vehicle, self.link, str(log_dir))
        self.log_dir = log_dir
        self.assignment = None    # ASSIGN body the ground repeats every second
        self.t0 = None            # START t0 the ground repeats every second
        self.alive = True

    def run(self, seconds):
        for k in range(round(seconds / TICK_S)):
            self.clock.advance(TICK_S)
            now = self.clock()
            if k % round(config.CONTROL_HZ) == 0:
                self.ground_sends()
            if self.alive:
                self.alive = self.agent.tick(now)
            if self.agent.status == "takeoff":
                time.sleep(0.0005)            # let the take-off thread poll the vehicle

    def run_until(self, status, limit_s=60.0):
        while self.agent.status != status:
            assert limit_s > 0, f"still {self.agent.status}"
            self.run(1.0)
            limit_s -= 1.0

    def ground_sends(self):
        if self.assignment is not None:
            self.ground.send(protocol.assign(1, **self.assignment))
        if self.t0 is not None:
            self.ground.send(protocol.start(self.t0))

    def rows(self):
        with open(self.log_dir / "agent_1.csv") as f:
            return list(csv.DictReader(f))

    def events(self):
        with open(self.log_dir / "agent_1.events.jsonl") as f:
            return [json.loads(line)["event"] for line in f]


def test_start_sequence_flies_on_assign_before_start(tmp_path):
    """Take off to the hold altitude, report ready, and fly the first ASSIGN without START."""
    rig = Rig(tmp_path)
    rig.run_until("ready")
    assert abs(rig.vehicle.pos[2] - config.hold_alt(1)) < 1.5
    x, _ = config.start_xy(0, 1)
    rig.assignment = {"epoch": 1, "share": [0, 300, 0, 300],
                      "slots": [[x, 100.0], [x, 200.0]], "loop_from": 0}
    rig.run(30)
    assert rig.agent.status == "cover"
    assert abs(rig.vehicle.pos[2] - config.ALT_CRUISE_M) < 1.0
    assert "slot_reached" in rig.events()
    assert all(r["t_mission"] == "" for r in rig.rows())
    states = [m for m in rig.link.sent if m.get_type() == "SWARM_STATE"]
    assert (states[-1].epoch, protocol.STATUSES[states[-1].status]) == (1, "cover")
    assert states[-1].time_usec / 1e6 == pytest.approx(rig.clock(), abs=0.2)
    rig.t0 = rig.clock() + 3.0                     # START only sets the mission clock
    rig.run(5)
    rows = rig.rows()
    assert rows[-1]["t_mission"] != "" and float(rows[-1]["t_mission"]) > 0
    with open(tmp_path / "agent_1.csv") as f:
        assert next(csv.reader(f)) == protocol.AGENT_CSV_COLUMNS


def test_take_off_waits_until_nobody_is_close(tmp_path):
    rig = Rig(tmp_path)
    x, y = config.start_xy(0, 1)
    other = rig.radio.link(2)
    for _ in range(round(3 / TICK_S)):              # agent 2 hovers 4 m away for 3 s
        other.send(protocol.state(rig.clock(), (x + 4.0, y, 40.0), (0, 0, 0), -1, -1, "hold"))
        rig.run(TICK_S)
    rig.run(2 * config.RADIO_DELAY_S)                # its last STATEs arrive
    assert rig.agent.takeoff_thread is None
    rig.agent.safety.nbrs.clear()                    # it left (and was forgotten)
    rig.run(1)
    assert rig.agent.takeoff_thread is not None


def test_one_setpoint_per_tick_after_take_off(tmp_path):
    rig = Rig(tmp_path, start_z=config.hold_alt(1))
    rig.run_until("ready")
    rig.vehicle.setpoints, rows_before = 0, len(rig.rows())
    rig.run(10)
    commanded = [r for r in rig.rows()[rows_before:] if r["cmd_z"] != ""]
    assert rig.vehicle.setpoints == len(commanded) == round(10 / TICK_S)


def test_goes_silent_and_exits_when_its_autopilot_link_dies(tmp_path):
    rig = Rig(tmp_path, start_z=config.hold_alt(1))
    rig.run_until("ready")
    rig.vehicle.link_ok = False
    sent, setpoints = len(rig.link.sent), rig.vehicle.setpoints
    rig.run(1.5)
    assert rig.alive
    rig.run(1.0)
    assert not rig.alive
    assert (len(rig.link.sent), rig.vehicle.setpoints) == (sent, setpoints)
    assert rig.events()[-2:] == ["autopilot_link_down", "autopilot_link_lost"]


def test_failsafe_when_the_autopilot_leaves_guided(tmp_path):
    rig = Rig(tmp_path, start_z=config.hold_alt(1))
    rig.run_until("ready")
    rig.vehicle.guided = False
    rig.run(1.5)
    assert rig.agent.status == "failsafe" and "failsafe" in rig.events()
    setpoints, states = rig.vehicle.setpoints, len(rig.link.sent)
    rig.run(2)
    assert rig.vehicle.setpoints == setpoints                  # no more setpoints ...
    assert protocol.STATUSES[rig.link.sent[-1].status] == "failsafe"    # ... but STATE goes on
    assert len(rig.link.sent) > states


def test_obstacle_appears_at_its_mission_time(tmp_path):
    rig = Rig(tmp_path, start_z=config.hold_alt(1))
    rig.agent.obstacle = (150.0, 100.0, 10.0, 5.0)
    rig.run_until("ready")
    rig.t0 = rig.clock() + 2.0
    rig.run(5)
    assert rig.agent.safety.obstacle is None
    rig.run(3)
    assert rig.agent.safety.obstacle is not None and "obstacle" in rig.events()


@pytest.mark.slow
def test_real_time_loop_ticks_at_20_hz(tmp_path):
    """run() on the wall clock: rows every 50 ms, then exit when the autopilot link dies."""
    rig = Rig(tmp_path, start_z=config.hold_alt(1), clock=time.time)
    stopper = threading.Timer(4.0, lambda: setattr(rig.vehicle, "link_ok", False))
    stopper.start()
    assert rig.agent.run() == 2
    t_wall = [float(r["t_wall"]) for r in rig.rows()]
    periods = [b - a for a, b in zip(t_wall, t_wall[1:])]
    assert sum(periods) / len(periods) == pytest.approx(TICK_S, abs=0.002)
    assert max(periods) < 2.5 * TICK_S


def test_an_assign_for_another_drone_is_ignored(tmp_path):
    """On a shared radio every agent hears every ASSIGN; only its own may change its plan."""
    rig = Rig(tmp_path)
    rig.run_until("ready")
    other = protocol.assign(2, 5, [0, 300, 0, 300], [[10.0, 100.0]], 0)
    rig.agent._on_message(other, rig.clock(), rig.vehicle.state())
    assert rig.agent.coverage.epoch is None
    mine = protocol.assign(1, 5, [0, 300, 0, 300], [[10.0, 100.0]], 0)
    rig.agent._on_message(mine, rig.clock(), rig.vehicle.state())
    assert rig.agent.coverage.epoch == 5
