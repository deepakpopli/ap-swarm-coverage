"""Whole onboard stack (Agent + CoverageController + SafetyFilter + FakeVehicle) with a fake radio
(20 % loss, 200 ms delay) and a scripted ground, stepped in simulated time. One agent is killed."""
import csv
import itertools
import json
import math
import os
import shutil
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fakes import FakeRadio, SimClock  # noqa: E402
from swarm import config as C  # noqa: E402
from swarm.agent import CSV_HEADER, Agent  # noqa: E402
from swarm.fake_vehicle import FakeVehicle  # noqa: E402

DT = 0.02


def strip_slots(k):
    """Agent k (1..3) covers x in [100(k-1), 100k]: one transit slot (keeps every leg well inside
    SLOT_TIMEOUT_S), then a rectangle loop inside its strip (loop_from = 1)."""
    x0 = 100.0 * (k - 1)
    return [[x0 + 50, 20], [x0 + 25, 25], [x0 + 25, 150], [x0 + 25, 275], [x0 + 75, 275], [x0 + 75, 150],
            [x0 + 75, 25]]


class CountingVehicle(FakeVehicle):
    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.setpoints = 0

    def send_setpoint(self, vx, vy, z):
        self.setpoints += 1
        super().send_setpoint(vx, vy, z)


def read_csv(path):
    with open(path) as fh:
        return list(csv.DictReader(fh))


def read_events(path):
    with open(path) as fh:
        return [json.loads(line) for line in fh]


class TestAgentStack(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="agenttest_")

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def _make(self, n=3, loss=C.RADIO_LOSS, delay=C.RADIO_DELAY_S, seed=4):
        clock = SimClock()
        radio = FakeRadio(clock, loss=loss, delay=delay, seed=seed)
        ground = radio.link(0)
        agents = []
        for i in range(1, n + 1):
            x, y = C.start_xy(i - 1, n)
            veh = CountingVehicle(i, start=(x, y, C.ALT_NOMINAL), clock=clock)  # already airborne
            agents.append(Agent(i, n, veh, radio.link(i), self.dir))
        return clock, radio, ground, agents

    def test_three_agents_one_killed(self):
        clock, radio, ground, agents = self._make()
        t_begin = clock()
        t0 = t_begin + 10.0       # gives the agents time to settle on their layers
        T, T_KILL = 100.0, 50.0
        for a in agents:
            a.start(clock())
        for a in agents:
            a._takeoff.join(1.0)
        alive = {a.id: a for a in agents}
        epoch, infeasible_rx = 1, []
        dmin = float("inf")
        filter_steps = {a.id: 0 for a in agents}
        orig = {}
        for a in agents:                       # count safety-filter calls per agent
            def wrapped(*args, _f=a.safety.step, _id=a.id, **kw):
                filter_steps[_id] += 1
                return _f(*args, **kw)
            orig[a.id] = a.safety.step
            a.safety.step = wrapped
        for k in range(int(T / DT)):
            clock.advance(DT)
            now = clock()
            # minimal allocator: on INFEASIBLE for the current epoch, re-plan (same slots, new epoch)
            for m in ground.recv_all():
                if m["type"] == "INFEASIBLE" and m["epoch"] == epoch:
                    epoch += 1
                    infeasible_rx.append(m)
            if k % 50 == 0 and now >= t_begin + 2.0:
                ground.send("all", "START", t0=t0)
                for a in agents:
                    ground.send(a.id, "ASSIGN", epoch=epoch, agent=a.id, share=[0, 300, 0, 300],
                                slots=strip_slots(a.id), loop_from=1, alive=sorted(alive))
            if now - t_begin >= T_KILL and 3 in alive:
                radio.dead.add(3)              # SIGKILL: no more messages, no goodbye
                alive.pop(3).log.close()
            for a in list(alive.values()):
                self.assertTrue(a.step(now))
            for a, b in itertools.combinations(agents, 2):
                if a.id in alive and b.id in alive:
                    dmin = min(dmin, math.dist(a.vehicle.pos, b.vehicle.pos))
        for a in alive.values():
            a.log.close()

        self.assertGreaterEqual(dmin, C.D_MIN)
        for a in agents:
            rows = read_csv(os.path.join(self.dir, f"agent_{a.id}.csv"))
            with open(os.path.join(self.dir, f"agent_{a.id}.csv")) as fh:
                self.assertEqual(fh.readline().strip(), CSV_HEADER)
            ev = read_events(os.path.join(self.dir, f"agent_{a.id}.events.jsonl"))
            names = [e["event"] for e in ev]
            # one setpoint per control tick, and every one went through the filter
            self.assertEqual(a.vehicle.setpoints, len(rows))
            self.assertEqual(filter_steps[a.id], len(rows))
            # 10 Hz
            ts = [float(r["t_wall"]) for r in rows]
            periods = [b - a_ for a_, b in zip(ts, ts[1:])]
            self.assertAlmostEqual(sum(periods) / len(periods), 0.1, delta=0.002)
            self.assertLessEqual(max(periods), 0.1 + DT + 1e-6)
            # t_mission empty before START, present after
            self.assertEqual(rows[0]["t_mission"], "")
            self.assertNotEqual(rows[-1]["t_mission"], "")
            self.assertIn("assign", names)
            self.assertIn("slot_reached", names)
            self.assertIn("loop_stats", names)
            self.assertTrue(all("t_wall" in e and "t_mission" in e for e in ev))
            statuses = {r["status"] for r in rows}
            self.assertIn("ready", statuses)
            self.assertIn("cover", statuses)
            if a.id != 3:
                lost = [e for e in ev if e["event"] == "neighbour_lost" and e["nbr"] == 3]
                self.assertEqual(len(lost), 1)
                self.assertAlmostEqual(lost[0]["t_wall"] - t_begin, T_KILL + C.LOST_S, delta=1.5)
                self.assertGreaterEqual(len([r for r in rows if r["status"] == "cover"]), 600)
                # every re-plan continued from the active slot instead of restarting the loop
                self.assertTrue(all(e["continued"] for e in ev if e["event"] == "assign" and e["epoch"] > 1))
                self.assertNotIn("deaf_enter", names)     # neighbours + ground keep it hearing
            # the loop reaches its slots in order
            reached = [e["slot"] for e in ev if e["event"] == "slot_reached"]
            self.assertEqual(reached[:8], [0, 1, 2, 3, 4, 5, 6, 1][:len(reached[:8])])
            self.assertGreaterEqual(len(reached), 3)
            if a.id != 3:
                self.assertGreaterEqual(len(reached), 8)          # completed at least one loop
                inf_epochs = [e["epoch"] for e in ev if e["event"] == "infeasible"]
                self.assertEqual(len(inf_epochs), len(set(inf_epochs)))      # at most one per epoch

    def test_autopilot_link_loss_exits_silently(self):
        clock, radio, ground, agents = self._make(n=1, loss=0.0, delay=0.0)
        a = agents[0]
        a.start(clock())
        a._takeoff.join(1.0)
        for _ in range(100):
            clock.advance(DT)
            self.assertTrue(a.step(clock()))
        a.vehicle.kill_link()
        t_kill = clock()
        n_sent = len(a.link.sent)
        while a.step(clock()):
            clock.advance(DT)
            self.assertLess(clock() - t_kill, 3.0)
        self.assertGreater(clock() - t_kill, 2.0)
        self.assertLessEqual(clock() - t_kill, 2.0 + 2 * DT)
        # STATE broadcasts continued only while the link was < 2 s dead? No: nothing is sent at exit.
        sent_after = a.link.sent[n_sent:]
        self.assertFalse([m for m in sent_after if m["type"] != "STATE"])
        a.log.close()

    def test_real_time_loop_rate(self):
        """3 s of the real-time loop (FakeVehicle near altitude, so takeoff completes at once)."""
        radio = FakeRadio(time.time)
        x, y = C.start_xy(0, 1)
        veh = FakeVehicle(1, start=(x, y, C.ALT_NOMINAL - 0.2))
        a = Agent(1, 1, veh, radio.link(1), self.dir)
        t_end = time.time() + 1.5

        def killer_clock():
            now = time.time()
            if now > t_end:
                veh.kill_link()
            return now
        t0 = time.time()
        code = a.run(clock=killer_clock)
        dur = time.time() - t0
        self.assertEqual(code, 2)                      # exited because the autopilot link died
        self.assertGreater(dur, 3.4)
        rows = read_csv(os.path.join(self.dir, "agent_1.csv"))
        ts = [float(r["t_wall"]) for r in rows]
        periods = sorted(b - a_ for a_, b in zip(ts, ts[1:]))
        self.assertAlmostEqual(sum(periods) / len(periods), 0.1, delta=0.005)
        self.assertLess(periods[-1], 0.13)
        self.assertGreaterEqual(len(rows), 30)


if __name__ == "__main__":
    unittest.main()
