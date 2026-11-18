"""Hand-checkable tests for swarm/sensor.py and analysis/*. Run: python -m pytest -q tests"""
import json
import math
import os
import tempfile
import unittest

import numpy as np

from swarm import config, sensor
from analysis import logs, metrics, synth
from analysis.logs import AGENT_COLUMNS, AgentLog, Run


def make_log(aid, track, status="cover"):
    """track: list of (t, x, y, z, yaw). t_wall = 1000 + t."""
    cols = {c: np.full(len(track), np.nan) for c in AGENT_COLUMNS if c not in logs.TEXT_COLUMNS}
    arr = np.array(track, dtype=float).reshape(-1, 5)
    for j, c in enumerate(("t_mission", "x", "y", "z", "yaw")):
        cols[c] = arr[:, j]
    cols["t_wall"] = 1000.0 + arr[:, 0]
    cols["status"] = [status] * len(track)
    cols["flags"] = [""] * len(track)
    return AgentLog(aid, cols, 0)


def make_run(tracks, meta=None, ground=None):
    run = Run("mem")
    run.agents = {aid: make_log(aid, tr) for aid, tr in tracks.items()}
    run.meta = meta or {}
    run.ground = ground or []
    return run


def hover(t0, t1, x, y, z=60.0, yaw=0.0, dt=0.1):
    n = int(round((t1 - t0) / dt))
    return [(t0 + k * dt, x, y, z, yaw) for k in range(n)]


class TestSensor(unittest.TestCase):
    def test_footprint_60m(self):
        across, along = sensor.footprint_size(60.0)
        self.assertAlmostEqual(across, 120 * math.tan(math.radians(40.5)), places=6)   # 102.5 m
        self.assertAlmostEqual(along, across * 9 / 16, places=6)                      # 57.7 m
        self.assertAlmostEqual(sensor.VFOV_DEG, 51.3, places=1)

    def test_yaw_rotates_wide_axis(self):
        # 50 m east of the camera: inside when heading north (across half-width 51.2 m),
        # outside when heading east (then east is along track, half-length 28.8 m).
        self.assertTrue(sensor.seen_mask(np.array([150.0]), np.array([100.0]), 100, 100, 60, 0.0)[0])
        self.assertFalse(sensor.seen_mask(np.array([150.0]), np.array([100.0]), 100, 100, 60, math.pi / 2)[0])

    def test_polygon_matches_mask(self):
        poly = np.array(sensor.footprint_polygon(100, 100, 60, 0.3))
        x, y = poly[:, 0], poly[:, 1]
        area = 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))
        self.assertAlmostEqual(area, np.prod(sensor.footprint_size(60)), places=3)
        inner = poly.mean(axis=0) * 0.0 + 0.999 * poly + 0.001 * poly.mean(axis=0)
        self.assertTrue(sensor.seen_mask(inner[:, 0], inner[:, 1], 100, 100, 60, 0.3).all())


class TestCoverage(unittest.TestCase):
    def test_single_hover_equals_footprint_fraction(self):
        run = make_run({1: hover(0, 120, 150, 150)})
        grid = metrics.Grid(1.0)
        ticks, cov = metrics.coverage_series(grid, metrics.coverage_samples(run, 180), 180)
        expected = np.prod(sensor.footprint_size(60)) / (300 * 300)        # 6.57 %
        self.assertAlmostEqual(cov[30], expected, delta=0.02 * expected)
        self.assertAlmostEqual(cov[119], cov[30])

    def test_window_forgets_after_60s(self):
        run = make_run({1: hover(0, 10.1, 150, 150)})       # last sample t = 10.0
        grid = metrics.Grid(2.0)
        ticks, cov = metrics.coverage_series(grid, metrics.coverage_samples(run, 180), 180)
        self.assertGreater(cov[70], 0.0)      # 70 - 60 = 10: still inside the window (inclusive)
        self.assertEqual(cov[71], 0.0)


class TestRevisitGap(unittest.TestCase):
    """Drone alternates between two disjoint footprints A (x=60) and B (x=240):
    A on [0,10), B on [10,40), A on [40,50). Samples every 0.1 s."""

    def setUp(self):
        tr = hover(0, 10, 60, 150) + hover(10, 40, 240, 150) + hover(40, 50, 60, 150)
        self.run = make_run({1: tr})
        self.grid = metrics.Grid(2.0)
        self.samples = metrics.coverage_samples(self.run, 180)
        self.ia = (75, 30)    # cell at (61, 151): under A only
        self.ib = (75, 120)   # cell at (241, 151): under B only

    def test_phase_from_mission_start(self):
        gap, never, age = metrics.phase_gaps_and_age(self.grid, self.samples, 0.0, 50.0,
                                                     from_first_sighting=True)
        self.assertAlmostEqual(gap[self.ia], 30.1, places=6)   # last seen 9.9, next 40.0
        self.assertAlmostEqual(gap[self.ib], 10.1, places=6)   # first seen 10 (latency excluded), last 39.9 -> end 50
        self.assertEqual(gap[75, 75], 50.0)                    # never seen: full phase, flagged
        self.assertTrue(never[75, 75])
        self.assertTrue(math.isnan(age[75, 75]))

    def test_open_start_is_clipped_but_age_is_not(self):
        gap, never, age = metrics.phase_gaps_and_age(self.grid, self.samples, 20.0, 50.0)
        self.assertAlmostEqual(gap[self.ia], 20.0, places=6)   # clipped: 40.0 - 20.0
        self.assertAlmostEqual(age[self.ia], 30.1, places=6)   # true staleness: 40.0 - 9.9
        self.assertFalse(never[self.ia])


class TestRecovery(unittest.TestCase):
    ticks = np.arange(0.0, 181.0)

    def series(self, low_from, low_to, low=0.5):
        c = np.ones_like(self.ticks)
        c[(self.ticks >= low_from) & (self.ticks < low_to)] = low
        return c

    def test_recovers(self):
        self.assertEqual(metrics.recovery_time(self.ticks, self.series(100, 130), 90, 0.9), 40.0)

    def test_never_recovers(self):
        self.assertIsNone(metrics.recovery_time(self.ticks, self.series(100, 999), 90, 0.9))

    def test_recovery_too_late_to_confirm(self):
        self.assertIsNone(metrics.recovery_time(self.ticks, self.series(100, 175), 90, 0.9))

    def test_never_dips(self):
        self.assertEqual(metrics.recovery_time(self.ticks, self.series(100, 130), 90, 0.4), 0.0)

    def test_pre_loss_value(self):
        c = self.ticks / 100.0
        self.assertAlmostEqual(metrics.pre_loss_value(self.ticks, c, 80.0), 0.745)   # mean of 70..79


class TestChurn(unittest.TestCase):
    def test_counting(self):
        ev = lambda t, ch, ep: {"t_mission": t, "event": "assign", "epoch": ep, "changed": ch}
        ground = [ev(0.0, [1, 2, 3], 1), ev(30.0, [2], 2),
                  {"t_mission": 72.0, "event": "liveness", "agent": 3, "from": "suspect", "to": "lost"},
                  ev(72.0, [1, 2], 3), ev(120.0, [1, 3], 4)]          # 3 is dead at 120: not counted
        run = make_run({1: hover(0, 1, 0, 0)}, ground=ground)
        out = metrics.churn(run, [1, 2, 3], 3, 60.0, 180.0, {"overall": (0.0, 180.0)}, 72.0)
        o = out["overall"]
        self.assertAlmostEqual(o["live_agent_minutes"], 7.0)        # 3 + 3 + 1
        self.assertEqual(o["changes_all"], 7)                          # 3 + 1 + 2 + 1
        self.assertEqual(o["changes_excl_initial_and_loss_replan"], 2)
        self.assertAlmostEqual(o["per_agent_min_all"], 1.0)
        self.assertEqual(out["loss_replan_epoch"], 3)


class TestSafety(unittest.TestCase):
    def test_min_separation_and_time_below(self):
        a = [(k * 0.1, 0.0, 0.0, 60.0, 0.0) for k in range(1801)]
        b = [(k * 0.1, 20.0 - (5.0 if 50 <= k * 0.1 < 52 else 0.0), 0.0, 60.0, 0.0) for k in range(1801)]
        run = make_run({1: a, 2: b})
        t, sep = metrics.separation_series(run, 180.0)
        self.assertAlmostEqual(np.nanmin(sep), 15.0)
        run = make_run({1: a, 2: [(t_, 5.0, 0, 60, 0) for t_, *_ in b[:100]]})   # 2 stops at 9.9 s
        t, sep = metrics.separation_series(run, 180.0)
        self.assertAlmostEqual(np.nansum(sep < 8.0) * 0.1, 10.0, places=6)
        self.assertTrue(np.isnan(sep[200]))


class TestLogsAndPipeline(unittest.TestCase):
    def test_synthetic_run_end_to_end(self):
        with tempfile.TemporaryDirectory() as d:
            synth.generate(d, seed=4)
            run = logs.load_run(d)
            self.assertTrue(any("truncated" in w for w in run.warnings))
            m, _ = metrics.analyse(d)
            self.assertTrue(os.path.exists(os.path.join(d, "metrics.json")))
            self.assertAlmostEqual(m["loss"]["detection_delay_s"], synth.DETECT_S)
            self.assertGreater(m["safety"]["min_separation_m"], config.D_MIN)
            self.assertEqual(m["safety"]["time_below_d_min_s"], 0.0)
            self.assertAlmostEqual(m["control_rate"]["1"]["rate_hz"], 10.0, delta=0.1)
            json.dumps(m)   # serialisable

    def test_missing_files_are_warnings(self):
        with tempfile.TemporaryDirectory() as d:
            synth.generate(d, seed=4)
            for f in ("meta.json", "ground.jsonl", "radio.jsonl"):
                os.remove(os.path.join(d, f))
            m, _ = metrics.analyse(d)
            self.assertIsNone(m["loss"])
            self.assertTrue(any("meta.json missing" in w for w in m["warnings"]))

    def test_no_agent_logs_is_a_clear_error(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(logs.RunLoadError):
                logs.load_run(d)


if __name__ == "__main__":
    unittest.main()
