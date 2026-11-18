"""Coverage controller tests, simulated time, ideal point mass (pos += v*dt)."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from swarm import config as C  # noqa: E402
from swarm.coverage import CoverageController  # noqa: E402

DT = 0.1


def assign(epoch, slots, loop_from=0):
    return {"epoch": epoch, "agent": 1, "share": [0, 300, 0, 300], "slots": slots, "loop_from": loop_from,
            "alive": [1, 2, 3]}


def fly(cov, pos, t, T, move=True):
    """Integrate the nominal command. Returns (pos, t, events, speeds[(t, slot, speed)])."""
    events, speeds = [], []
    for _ in range(int(round(T / DT))):
        out = cov.step(t, pos)
        events += out.events
        speeds.append((t, out.slot, math.hypot(out.vx, out.vy)))
        if move:
            pos = (pos[0] + out.vx * DT, pos[1] + out.vy * DT, pos[2])
        t += DT
    return pos, t, events, speeds


class TestCoverage(unittest.TestCase):
    def test_transit_then_loop(self):
        cov = CoverageController(1)
        slots = [[0, 50], [50, 50], [50, 100], [100, 100], [100, 50]]
        cov.on_assign(assign(1, slots, loop_from=1), 0.0)
        _, _, ev, _ = fly(cov, (0.0, 0.0, 60.0), 0.0, 120.0)
        reached = [e["slot"] for e in ev if e["event"] == "slot_reached"]
        self.assertEqual(reached[:9], [0, 1, 2, 3, 4, 1, 2, 3, 4])
        self.assertEqual(reached.count(0), 1)                  # transit slot flown once
        self.assertFalse([e for e in ev if e["event"] == "infeasible"])

    def test_passes_collinear_slots_at_speed_and_slows_for_turns(self):
        cov = CoverageController(1)
        slots = [[0, 0], [0, 50], [0, 100], [0, 150], [100, 150]]   # straight north, then 90 deg east
        cov.on_assign(assign(1, slots, loop_from=0), 0.0)
        _, _, ev, speeds = fly(cov, (0.0, -30.0, 60.0), 0.0, 25.0)
        t_reach = {e["slot"]: e for e in ev if e["event"] == "slot_reached"}
        for k in (1, 2):   # collinear: at cruise speed when the slot switches
            sp = [s for t, sl, s in speeds if sl == k][-1]
            self.assertAlmostEqual(sp, C.V_CRUISE, delta=0.01)
        sp3 = [s for t, sl, s in speeds if sl == 3][-1]
        self.assertLess(sp3, 6.0)                               # decelerating into the 90 deg corner
        self.assertIn(3, t_reach)

    def test_passed_along_track_counts_as_reached(self):
        cov = CoverageController(1)
        cov.on_assign(assign(1, [[0, 0], [0, 100], [0, 200]], loop_from=0), 0.0)
        cov.step(0.0, (0.0, 0.0, 60.0))                        # reaches slot 0 -> active slot 1
        self.assertEqual(cov.idx, 1)
        out = cov.step(1.0, (8.0, 101.0, 60.0))                 # 8 m off track, beyond the slot
        self.assertEqual(cov.idx, 2)
        self.assertTrue(any(e["event"] == "slot_reached" and e["slot"] == 1 for e in out.events))
        out = cov.step(2.0, (12.0, 199.0, 60.0))                # 12 m off: not reached
        self.assertEqual(cov.idx, 2)

    def test_timeout_exactly_one_infeasible_per_epoch(self):
        cov = CoverageController(1)
        cov.on_assign(assign(1, [[200, 200], [250, 200]]), 0.0)
        pos, t, ev, speeds = fly(cov, (0.0, 0.0, 60.0), 0.0, 60.0, move=False)
        inf = [e for e in ev if e["event"] == "infeasible"]
        self.assertEqual(len(inf), 1)
        self.assertEqual((inf[0]["epoch"], inf[0]["slot"], inf[0]["reason"]), (1, 0, "timeout"))
        t_inf = [t_ for t_, sl, s in speeds if s == 0.0][0]
        self.assertAlmostEqual(t_inf, C.SLOT_TIMEOUT_S + DT, delta=DT + 1e-9)
        self.assertTrue(all(s == 0.0 for t_, sl, s in speeds if t_ >= t_inf))   # holds
        # re-sent at ~1 Hz while the epoch lasts
        sends = [k * 0.1 for k in range(0, 50) if cov.infeasible_message(100.0 + k * 0.1) is not None]
        self.assertEqual(len(sends), 5)
        # a second reason in the same epoch is not reported
        self.assertIsNone(cov.report_safety(105.0, pos))
        # an OLD epoch does not reset anything
        self.assertIsNone(cov.on_assign(assign(1, [[0, 0]]), 106.0))
        self.assertIsNotNone(cov.infeasible_message(107.5))
        # a newer epoch clears it and resumes
        self.assertIsNotNone(cov.on_assign(assign(2, [[10, 0], [20, 0]]), 108.0))
        self.assertIsNone(cov.infeasible_message(108.0))
        out = cov.step(108.0, pos)
        self.assertFalse(out.holding)
        self.assertGreater(math.hypot(out.vx, out.vy), 1.0)
        # and can time out again in the new epoch, once
        _, _, ev2, _ = fly(cov, pos, 108.0, 40.0, move=False)
        self.assertEqual(len([e for e in ev2 if e["event"] == "infeasible" and e["epoch"] == 2]), 1)

    def test_paused_time_does_not_count_toward_timeout(self):
        cov = CoverageController(1)
        cov.on_assign(assign(1, [[200, 200], [250, 200]]), 0.0)
        pos, t = (0.0, 0.0, 60.0), 0.0
        events = []
        for _ in range(300):                  # 30 s standing still, 12 s of it held by safety
            events += cov.step(t, pos).events
            if 5.0 <= t < 17.0:
                cov.pause_timer(DT)
            t += DT
        inf = [e for e in events if e["event"] == "infeasible"]
        self.assertEqual(len(inf), 1)          # still times out, but only after 15 s unpaused
        cov2 = CoverageController(1)
        cov2.on_assign(assign(1, [[200, 200], [250, 200]]), 0.0)
        t, events = 0.0, []
        for _ in range(200):                   # 20 s, 12 s paused: 8 s counted, no timeout
            events += cov2.step(t, pos).events
            if 5.0 <= t < 17.0:
                cov2.pause_timer(DT)
            t += DT
        self.assertFalse([e for e in events if e["event"] == "infeasible"])

    def test_safety_report_once_per_epoch(self):
        cov = CoverageController(1)
        cov.on_assign(assign(3, [[100, 0]]), 0.0)
        cov.step(0.0, (0.0, 0.0, 60.0))
        ev = cov.report_safety(1.0, (0.0, 0.0, 60.0))
        self.assertEqual((ev["epoch"], ev["reason"]), (3, "safety"))
        self.assertIsNone(cov.report_safety(2.0, (0.0, 0.0, 60.0)))
        self.assertFalse(cov.step(2.0, (0.0, 0.0, 60.0)).holding)   # safety does not stop coverage

    def test_epoch_change_continues_from_active_slot_geometry(self):
        cov = CoverageController(1)
        cov.on_assign(assign(1, [[0, 0], [0, 100], [100, 100], [100, 0]]), 0.0)
        cov.step(0.0, (0.0, 0.0, 60.0))          # slot 0 reached, active = (0, 100)
        self.assertEqual(cov.active_slot(), (0.0, 100.0))
        ev = cov.on_assign(assign(2, [[50, 50], [0, 100], [200, 100]]), 1.0)
        self.assertTrue(ev["continued"])
        self.assertEqual(cov.idx, 1)
        ev = cov.on_assign(assign(3, [[50, 50], [60, 100]]), 2.0)
        self.assertFalse(ev["continued"])
        self.assertEqual(cov.idx, 0)
        self.assertIsNone(cov.on_assign(assign(3, [[1, 1]]), 3.0))   # same epoch ignored
        self.assertIsNone(cov.on_assign(assign(2, [[1, 1]]), 3.0))   # older epoch ignored
        self.assertEqual(cov.epoch, 3)

    def test_single_slot_loop_station_keeps_without_timeout(self):
        cov = CoverageController(1)
        cov.on_assign(assign(1, [[0, 0], [30, 0]], loop_from=1), 0.0)
        pos, _, ev, speeds = fly(cov, (0.0, 0.0, 60.0), 0.0, 60.0)
        self.assertFalse([e for e in ev if e["event"] == "infeasible"])
        self.assertLess(math.hypot(pos[0] - 30.0, pos[1]), 1.0)

    def test_no_plan_holds(self):
        cov = CoverageController(1)
        out = cov.step(0.0, (0.0, 0.0, 60.0))
        self.assertTrue(out.holding)
        self.assertEqual((out.vx, out.vy, out.z), (0.0, 0.0, C.ALT_NOMINAL))


if __name__ == "__main__":
    unittest.main()
