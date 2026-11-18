"""Safety filter tests: point masses (FakeVehicle) stepped in simulated time, radio via FakeRadio."""
import itertools
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fakes import FakeRadio, SimClock  # noqa: E402
from swarm import config as C  # noqa: E402
from swarm.fake_vehicle import FakeVehicle  # noqa: E402
from swarm.safety import DEAF, LAYERED, NORMAL, SafetyFilter, SafetyParams  # noqa: E402

DT = 0.02          # physics / main loop step, s
CTRL_EVERY = 5     # 10 Hz
STATE_EVERY = 10   # 5 Hz
GROUND_EVERY = 50  # 1 Hz ground heartbeat (START re-send)


class SimAgent:
    def __init__(self, aid, start, goal, clock, radio, mode=NORMAL):
        self.id, self.goal = aid, goal
        self.veh = FakeVehicle(aid, start=start, clock=clock)
        self.veh.connect()
        self.filt = SafetyFilter(aid)
        self.filt.mode = mode
        self.link = radio.link(aid)
        self.reached_t = None
        self.events = []
        self.infeasible = 0
        self.log = []          # (t, x, y, z, speed_xy, mode)

    def nominal(self):
        if self.goal is None:
            return (0.0, 0.0, C.ALT_NOMINAL)
        x, y, _ = self.veh.pos
        dx, dy = self.goal[0] - x, self.goal[1] - y
        d = math.hypot(dx, dy)
        if d < 0.5:
            return (0.0, 0.0, C.ALT_NOMINAL)
        s = min(C.V_CRUISE, math.sqrt(2 * 1.5 * d))
        return (s * dx / d, s * dy / d, C.ALT_NOMINAL)


def run_sim(specs, T, loss=0.0, delay=0.0, blackouts=(), seed=0):
    """specs: list of (id, start_xyz, goal_xy or None[, mode]). Returns (agents, min 3D distance, t of min)."""
    clock = SimClock()
    radio = FakeRadio(clock, loss=loss, delay=delay, seed=seed)
    t_start = clock()
    radio.blackouts = [(n, t_start + a, t_start + b) for n, a, b in blackouts]
    ground = radio.link(0)
    agents = [SimAgent(*s[:3], clock, radio, *(s[3:])) for s in specs]
    dmin, tmin = float("inf"), None
    for k in range(1, int(T / DT) + 1):
        clock.advance(DT)
        now = clock()
        for a in agents:
            a.veh.poll()
        for a in agents:
            for m in a.link.recv_all():
                a.filt.heard(now)
                if m["type"] == "STATE":
                    a.filt.update_neighbour(m["src"], m["pos"], m["vel"], m["status"], m["t"], now)
        if k % GROUND_EVERY == 0:
            ground.send("all", "START", t0=t_start)
        if k % STATE_EVERY == 0:
            for a in agents:
                a.link.send("all", "STATE", pos=list(a.veh.pos), vel=list(a.veh.vel), epoch=1, slot=0,
                            status="cover")
        if k % CTRL_EVERY == 0:
            for a in agents:
                cmd = a.filt.step(now, tuple(a.veh.pos), tuple(a.veh.vel), a.nominal())
                a.veh.send_setpoint(cmd.vx, cmd.vy, cmd.z)
                a.events += [dict(e, t=now - t_start) for e in cmd.events]
                a.infeasible += int(cmd.infeasible)
                a.log.append((now - t_start, *a.veh.pos, math.hypot(*a.veh.vel[:2]), cmd.mode))
                if a.goal and a.reached_t is None and \
                        math.hypot(a.goal[0] - a.veh.pos[0], a.goal[1] - a.veh.pos[1]) < C.SLOT_REACH_M:
                    a.reached_t = now - t_start
        for a, b in itertools.combinations(agents, 2):
            d = math.dist(a.veh.pos, b.veh.pos)
            if d < dmin:
                dmin, tmin = d, now - t_start
    return agents, dmin, tmin


def head_on(**kw):
    return run_sim([(1, (0.0, 150.0, 60.0), (300.0, 150.0)),
                    (2, (300.0, 150.0, 60.0), (0.0, 150.0))], **kw)


class TestRadius(unittest.TestCase):
    def test_numbers(self):
        p = SafetyParams()
        self.assertAlmostEqual(p.t_go, 4.3)
        self.assertAlmostEqual(p.radius(0.0), 9.0)
        self.assertAlmostEqual(p.radius(1.0), 11.0)
        self.assertAlmostEqual(p.radius(3.0), 27.0)
        self.assertAlmostEqual(p.radius(10.0), 9.0 + 0.5 * 4 * 4.3 ** 2 + 24.0)
        self.assertAlmostEqual(p.radius(100.0), p.radius(10.0))           # bounded
        ages = [i * 0.05 for i in range(300)]
        self.assertTrue(all(p.radius(a) <= p.radius(b) for a, b in zip(ages, ages[1:])))  # monotone

    def test_v_stop_is_braking_curve(self):
        p = SafetyParams()
        for gap in (3.5, 5.0, 30.6, 80.0):
            v = p.v_stop(gap)
            self.assertAlmostEqual(p.stop_offset + v * p.t_react + v * v / (2 * p.a_brake), gap, places=6)
        self.assertEqual(p.v_stop(p.stop_offset), 0.0)
        self.assertEqual(p.v_stop(-1.0), 0.0)

    def test_layers(self):
        p = SafetyParams()
        self.assertEqual(p.layers(), [50.0, 60.0, 70.0])
        self.assertEqual([p.layer_of(i) for i in (1, 2, 3, 4)], [50.0, 60.0, 70.0, 50.0])
        self.assertGreaterEqual(p.layer_spacing - 2 * p.z_track, p.d_min)


class TestUnit(unittest.TestCase):
    def test_dead_neighbour_dropped_after_lost_s(self):
        f = SafetyFilter(1)
        f.mode = NORMAL
        f.update_neighbour(2, (100.0, 0.0, 60.0), (0.0, 0.0, 0.0), "cover", 0.0, 0.2)
        evs = []
        t = 0.2
        while t < 0.2 + C.LOST_S + 1.0:
            t += 0.1
            f.heard(t)                      # the ground keeps talking, we are not deaf
            out = f.step(t, (0.0, 0.0, 60.0), (0.0, 0.0, 0.0), (0.0, 0.0, 60.0))
            evs += [(t, e["event"]) for e in out.events]
            if 2 not in f.nbrs:
                break
        names = [e for _, e in evs]
        self.assertIn("neighbour_suspect", names)
        self.assertIn("neighbour_lost", names)
        t_lost = [t for t, e in evs if e == "neighbour_lost"][0]
        self.assertGreater(t_lost - 0.2, C.LOST_S)
        self.assertLess(t_lost - 0.2, C.LOST_S + 0.2)

    def test_deaf_rule(self):
        f = SafetyFilter(3)
        f.mode = NORMAL
        f.heard(0.0)
        out = f.step(1.0, (0.0, 0.0, 60.0), (5.0, 0.0, 0.0), (10.0, 0.0, 60.0))
        self.assertEqual(out.mode, NORMAL)
        self.assertAlmostEqual(out.vx, 10.0)
        out = f.step(3.1, (0.0, 0.0, 60.0), (5.0, 0.0, 0.0), (10.0, 0.0, 60.0))
        self.assertEqual(out.mode, DEAF)
        self.assertEqual((out.vx, out.vy, out.z), (0.0, 0.0, 70.0))
        f.heard(5.0)
        out = f.step(5.0, (0.0, 0.0, 70.0), (0.0, 0.0, 0.0), (10.0, 0.0, 60.0))
        self.assertEqual(out.mode, LAYERED)        # resumes through the layered state
        out = f.step(6.1, (0.0, 0.0, 70.0), (0.0, 0.0, 0.0), (10.0, 0.0, 60.0))
        self.assertEqual(out.mode, NORMAL)         # nobody around: clear after 1 s

    def test_output_clamp(self):
        f = SafetyFilter(1)
        f.mode = NORMAL
        f.heard(0.0)
        out = f.step(0.1, (0.0, 0.0, 60.0), (0.0, 0.0, 0.0), (30.0, 40.0, 95.0))
        self.assertAlmostEqual(math.hypot(out.vx, out.vy), C.V_MAX)
        self.assertEqual(out.z, C.ALT_BAND[1])

    def test_vertical_layers_are_separated(self):
        """Agent 1 at its layer (50) ignores agent 3 at 60/70; agent at 60 does not ignore agent 1."""
        f = SafetyFilter(1)
        f.mode = LAYERED
        f.layer_reason = "fallback"
        f.heard(0.0)
        f.update_neighbour(3, (5.0, 0.0, 70.0), (0.0, 0.0, 0.0), "fallback", 0.0, 0.0)
        out = f.step(0.1, (0.0, 0.0, 50.0), (0.0, 0.0, 0.0), (10.0, 0.0, 60.0))
        self.assertEqual(out.mode, LAYERED)
        self.assertAlmostEqual(out.vx, 10.0)       # passes under agent 3
        g = SafetyFilter(2)
        g.mode = NORMAL
        g.heard(0.0)
        g.update_neighbour(1, (5.0, 0.0, 50.0), (0.0, 0.0, 0.0), "fallback", 0.0, 0.0)
        out = g.step(0.1, (0.0, 0.0, 60.0), (0.0, 0.0, 0.0), (10.0, 0.0, 60.0))
        self.assertEqual(out.mode, LAYERED)        # 5 m from agent 1 whose interval reaches 60 -> fallback
        self.assertTrue(out.infeasible)


class TestScenarios(unittest.TestCase):
    def assertSeparated(self, dmin):
        self.assertGreaterEqual(dmin, C.D_MIN, f"min 3D distance {dmin:.2f} m < D_MIN")

    def test_head_on_fresh(self):
        agents, dmin, _ = head_on(T=80.0)
        self.assertSeparated(dmin)
        for a in agents:
            self.assertIsNotNone(a.reached_t, f"agent {a.id} did not reach its goal")
            self.assertEqual(a.infeasible, 0)

    def test_head_on_delay_and_loss(self):
        for seed in range(5):
            with self.subTest(seed=seed):
                agents, dmin, _ = head_on(T=80.0, loss=C.RADIO_LOSS, delay=C.RADIO_DELAY_S, seed=seed)
                self.assertSeparated(dmin)
                for a in agents:
                    self.assertIsNotNone(a.reached_t, f"agent {a.id} did not reach its goal")

    def test_head_on_blackout_10s(self):
        """Agent 2 hears nothing and is heard by nobody for 10 s, starting while the pair closes."""
        for t_black in (6.0, 9.0, 12.0):
            with self.subTest(t_black=t_black):
                agents, dmin, _ = head_on(T=90.0, loss=C.RADIO_LOSS, delay=C.RADIO_DELAY_S, seed=1,
                                          blackouts=[(2, t_black, t_black + 10.0)])
                self.assertSeparated(dmin)
                b = agents[1]
                deaf = [e for e in b.events if e["event"] == "deaf_enter"]
                self.assertEqual(len(deaf), 1)
                # deaf 3 s after the LAST message heard, which can be up to ~1 s before the blackout
                self.assertTrue(t_black + C.DEAF_STOP_S - 1.2 <= deaf[0]["t"] <= t_black + C.DEAF_STOP_S + 0.3,
                                deaf[0]["t"])
                # holds: after deaf + braking (12/3 = 4 s + lag) horizontal speed ~0, at its layer
                hold = [r for r in b.log if deaf[0]["t"] + 5.0 <= r[0] <= t_black + 10.0]
                self.assertTrue(hold)
                self.assertTrue(all(r[4] < 0.3 for r in hold), max(r[4] for r in hold))
                self.assertAlmostEqual(hold[-1][3], 60.0, delta=1.0)    # layer of id 2
                for a in agents:
                    self.assertIsNotNone(a.reached_t, f"agent {a.id} never reached its goal")

    def test_crossing_90deg(self):
        for loss, delay in ((0.0, 0.0), (C.RADIO_LOSS, C.RADIO_DELAY_S)):
            with self.subTest(loss=loss):
                agents, dmin, _ = run_sim([(1, (0.0, 150.0, 60.0), (300.0, 150.0)),
                                           (2, (150.0, 0.0, 60.0), (150.0, 300.0))],
                                          T=80.0, loss=loss, delay=delay, seed=3)
                self.assertSeparated(dmin)
                for a in agents:
                    self.assertIsNotNone(a.reached_t)

    def test_boxed_in_enters_fallback_and_separates_vertically(self):
        """Agent 1 inside a ring of 4 hovering agents (gaps between them < 2 R): no horizontal way out.
        Fresh states: it must stall and go to fallback as "boxed". With delay + loss the stale discs
        may grow over it first ("inside"); either way it must fall back, drop to 50 m and get out."""
        cx, cy = 100.0, 100.0
        for loss, delay, r, why in ((0.0, 0.0, 12.0, {"boxed"}),
                                    (C.RADIO_LOSS, C.RADIO_DELAY_S, 13.0, {"boxed", "inside"})):
            with self.subTest(loss=loss):
                ring = [(2, (cx + r, cy)), (3, (cx, cy + r)), (5, (cx, cy - r)), (6, (cx - r, cy))]
                specs = [(1, (cx, cy, 60.0), (200.0, 100.0))] + \
                        [(i, (x, y, 60.0), None) for i, (x, y) in ring]
                agents, dmin, _ = run_sim(specs, T=60.0, loss=loss, delay=delay, seed=2)
                self.assertSeparated(dmin)
                a = agents[0]
                enter = [e for e in a.events if e["event"] == "fallback_enter"]
                self.assertTrue(enter)
                self.assertIn(enter[0]["why"], why)
                self.assertEqual(a.infeasible, len(enter))
                self.assertLess(min(row[3] for row in a.log), 51.0)  # went down to its layer (50 m)
                self.assertIsNotNone(a.reached_t)                     # and escaped under the ring


if __name__ == "__main__":
    unittest.main()
