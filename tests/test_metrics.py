"""Hand-checkable tests of swarm/sensor.py and analysis/. Runs are built in memory."""
import json
import math
import os
import tempfile
import unittest

import numpy as np

from analysis import logs, metrics, report, scenarios
from analysis.logs import TEXT_COLUMNS, Run
from swarm import sensor
from swarm.protocol import AGENT_CSV_COLUMNS

T0_WALL = 1000.0
STEP_S = 0.05          # 20 Hz rows


def track(t_from, t_to, x, y, z=50.0, yaw=0.0):
    """Rows (t, x, y, z, yaw) of an agent holding still at x, y from t_from until before t_to."""
    count = int(round((t_to - t_from) / STEP_S))
    return [(round(t_from + k * STEP_S, 6), x, y, z, yaw) for k in range(count)]


def agent_log(rows, flags=""):
    """The columns logs.read_agent_log would return for these rows."""
    rows = np.array(rows, dtype=float)
    log = {name: np.zeros(len(rows)) for name in AGENT_CSV_COLUMNS if name not in TEXT_COLUMNS}
    for i, name in enumerate(("t", "x", "y", "z", "yaw")):
        log[name] = rows[:, i]
    log["t_mission"] = log["t"]
    log["t_wall"] = T0_WALL + log["t"]
    log["epoch"] = np.ones(len(rows))
    log["status"] = ["cover"] * len(rows)
    log["flags"] = [flags] * len(rows)
    return log


def make_run(agents, ground=(), **meta):
    full_meta = {"mission": "A", "scenario": None, "seed": 1, "n": len(agents),
                 "backend": "gazebo", "mission_s": 180.0, "t0_wall": T0_WALL,
                 "kill_agent": None, "kill_wall": None, "blackout": None, "priority_zone": None}
    full_meta.update(meta)
    radio = {"total": {"sent": 100, "delivered": 80, "dropped_loss": 20, "dropped_window": 0,
                       "delay_mean_s": 0.2, "delay_max_s": 0.21}}
    return Run("test", full_meta, agents, list(ground), radio)


def assign(t, epoch, trigger, changed, assignments=None):
    return {"event": "assign", "t_mission": t, "epoch": epoch, "trigger": trigger,
            "changed": changed, "assignments": assignments or {}, "uncovered": None}


def lost(t, agent):
    return {"event": "liveness", "t_mission": t, "agent": agent, "from": "suspect", "to": "lost",
            "status": "cover"}


def cell_at(x, y):
    """Flat index of the grid cell containing the point x, y."""
    return int(np.argmin(np.hypot(metrics.CELL_X_M - x, metrics.CELL_Y_M - y)))


class SensorTest(unittest.TestCase):
    def test_footprint_at_50m(self):
        across_m, along_m = sensor.footprint_size(50.0)
        self.assertAlmostEqual(across_m, 100.0 * math.tan(math.radians(40.5)))   # 85.4 m
        self.assertAlmostEqual(along_m, across_m * 9.0 / 16.0)                   # 48.0 m
        self.assertEqual(sensor.lane_footprint(50.0), (85.0, 48.0))
        self.assertEqual(sensor.lane_footprint(60.0), (102.0, 57.0))

    def test_yaw_turns_the_wide_side(self):
        # 40 m east of the camera: inside when heading north (half width 42.7 m),
        # outside when heading east (then east is along track, half length 24.0 m).
        point = (np.array([140.0]), np.array([100.0]))
        self.assertTrue(sensor.seen_mask(*point, 100.0, 100.0, 50.0, 0.0)[0])
        self.assertFalse(sensor.seen_mask(*point, 100.0, 100.0, 50.0, math.pi / 2)[0])

    def test_polygon_area(self):
        x, y = np.array(sensor.footprint_polygon(100.0, 100.0, 50.0, 0.3)).T
        area = 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))
        self.assertAlmostEqual(area, np.prod(sensor.footprint_size(50.0)), places=6)


def coverage_of(run):
    ticks = np.arange(0.0, 181.0)
    seen = metrics.sightings(run)
    return ticks, metrics.percent_fresh(seen, ticks, metrics.WINDOW_S, metrics.ALL_CELLS)


class CoverageTest(unittest.TestCase):
    def test_one_hovering_drone_covers_its_footprint(self):
        run = make_run({1: agent_log(track(0.0, 180.0, 150.0, 150.0))})
        ticks, coverage = coverage_of(run)
        expected = 100.0 * np.prod(sensor.footprint_size(50.0)) / 300.0 ** 2     # 4.56 %
        self.assertAlmostEqual(coverage[0], expected, delta=0.02 * expected)
        self.assertAlmostEqual(coverage[180], coverage[0])

    def test_warm_up_rows_count_and_earlier_rows_do_not(self):
        run = make_run({1: agent_log(track(-90.0, -61.0, 60.0, 150.0)
                                     + track(-30.0, 0.0, 240.0, 150.0))})
        _, coverage = coverage_of(run)
        one_footprint = 100.0 * np.prod(sensor.footprint_size(50.0)) / 300.0 ** 2
        self.assertAlmostEqual(coverage[0], one_footprint, delta=0.02 * one_footprint)


class RevisitGapTest(unittest.TestCase):
    """The drone sits over A (x = 60) for [0, 10), over B (x = 240) for [10, 40), over A again
    for [40, 50). The two footprints do not overlap."""

    def setUp(self):
        rows = (track(0.0, 10.0, 60.0, 150.0) + track(10.0, 40.0, 240.0, 150.0)
                + track(40.0, 50.0, 60.0, 150.0))
        self.seen = metrics.sightings(make_run({1: agent_log(rows)}))
        self.a = cell_at(61.0, 151.0)
        self.b = cell_at(241.0, 151.0)

    def test_completed_gaps(self):
        longest, last = metrics.revisit_gaps(self.seen, 0.0, 50.0)
        self.assertAlmostEqual(longest[self.a], 30.05)      # last seen 9.95, again at 40.0
        self.assertAlmostEqual(longest[self.b], 0.05)       # seen every row while there
        self.assertAlmostEqual(last[self.b], 39.95)
        self.assertTrue(np.isnan(longest[cell_at(150.0, 150.0)]))   # never seen

    def test_gap_must_start_inside_the_phase(self):
        longest, _ = metrics.revisit_gaps(self.seen, 20.0, 50.0)
        self.assertAlmostEqual(longest[self.a], 0.05)       # 9.95 is before the phase
        block = metrics.revisit_block(self.seen, 20.0, 50.0)
        self.assertAlmostEqual(block["worst_open_s"], 50.0)  # a cell never seen: open since 0


class LongestUnseenTest(unittest.TestCase):
    """Mission of 100 s, loss at L = 40. Agent 1 sits over A (x = 60) for [-60, -50), over B
    (x = 240) for [-50, 20), A again [20, 25), C (x = 150) [25, 50), B again [50, 100).
    Agent 2 (the victim) sits over E (150, 30) from -60 until the loss. D (150, 280) is never
    seen. Footprints (85 x 48 m, wide side east-west) do not overlap."""

    def setUp(self):
        rows = (track(-60.0, -50.0, 60.0, 150.0) + track(-50.0, 20.0, 240.0, 150.0)
                + track(20.0, 25.0, 60.0, 150.0) + track(25.0, 50.0, 150.0, 150.0)
                + track(50.0, 100.0, 240.0, 150.0))
        victim = track(-60.0, 40.0, 150.0, 30.0)
        self.run = make_run({1: agent_log(rows), 2: agent_log(victim)}, mission_s=100.0,
                            kill_agent=2, kill_wall=T0_WALL + 40.0)
        self.seen = metrics.sightings(self.run)
        self.cells = {k: cell_at(*xy) for k, xy in {"A": (61, 151), "B": (241, 151),
                      "C": (151, 151), "D": (151, 281), "E": (151, 31)}.items()}

    def per_cell(self, *args):
        longest = metrics.longest_unseen(self.seen, *args)
        return {k: round(float(longest[i]), 2) for k, i in self.cells.items()}

    def test_phases(self):
        # whole [0, 100]: A open since 24.95; B 19.95 -> 50.0; C never seen until 25 (counts
        # from -60); D never seen; E open since the loss (last row 39.95)
        self.assertEqual(self.per_cell(100.0),
                         {"A": 75.05, "B": 30.05, "C": 85.0, "D": 160.0, "E": 60.05})
        # before [0, L): only gaps that end before 40; nothing open is counted
        self.assertEqual(self.per_cell(40.0, 0.0, False),
                         {"A": 70.05, "B": 0.05, "C": 85.0, "D": 0.0, "E": 0.05})
        # from the loss [L, end]: B's gap across L counts in full, A's and E's open gaps too
        self.assertEqual(self.per_cell(100.0, 40.0, True),
                         {"A": 75.05, "B": 30.05, "C": 50.05, "D": 160.0, "E": 60.05})

    def test_blocks(self):
        blocks = metrics.unseen_blocks(self.run, self.seen)
        self.assertAlmostEqual(blocks["whole"]["worst_s"], 160.0)
        self.assertAlmostEqual(blocks["before"]["worst_s"], 85.0)
        self.assertAlmostEqual(blocks["after"]["worst_s"], 160.0)
        # before: only cells first seen under C (85 s) and A (70.05 s) exceed 60 s
        before = metrics.longest_unseen(self.seen, 40.0, 0.0, False)
        self.assertAlmostEqual(blocks["before"]["cells_over_60s_pct"],
                               100.0 * np.mean(before > 60.0))
        self.assertAlmostEqual(blocks["before"]["cells_over_90s_pct"], 0.0)


class LossTest(unittest.TestCase):
    ticks = np.arange(0.0, 181.0)

    def coverage_with_dip(self, low_from, low_to, low=50.0):
        coverage = np.full(len(self.ticks), 90.0)
        coverage[(self.ticks >= low_from) & (self.ticks < low_to)] = low
        return coverage

    def test_recovery(self):
        self.assertEqual(metrics.recovery_time(self.ticks, self.coverage_with_dip(100, 130),
                                               90.5, 81.0), 39.5)
        self.assertEqual(metrics.recovery_time(self.ticks, self.coverage_with_dip(100, 130),
                                               90.5, 40.0), 0.0)        # never below 40
        self.assertIsNone(metrics.recovery_time(self.ticks, self.coverage_with_dip(100, 999),
                                                90.5, 81.0))           # still low at the end
        self.assertIsNone(metrics.recovery_time(self.ticks, self.coverage_with_dip(100, 175),
                                                90.5, 81.0))           # back for only 5 s

    def test_phases_and_deficit(self):
        run = make_run({1: agent_log(track(0, 1, 0, 0))}, n=3, kill_agent=3,
                       kill_wall=T0_WALL + 100.5)
        coverage = self.coverage_with_dip(110, 130)
        coverage[self.ticks < 41] = 0.0                     # outside the before window
        coverage[self.ticks >= 161] = 70.0                  # the after phase starts at 160.5
        loss = metrics.loss_block(run, self.ticks, coverage)
        self.assertAlmostEqual(loss["before_pct"], 90.0)    # ticks 41..100
        self.assertAlmostEqual(loss["share_pct"], 60.0)
        self.assertAlmostEqual(loss["after_mean_pct"], 70.0)
        self.assertAlmostEqual(loss["deficit_pct_s"], 20 * 40.0 + 20 * 20.0)
        self.assertEqual(loss["dip_pct"], 50.0)
        self.assertIsNone(loss["recovery_s"])
        self.assertEqual(loss["recovery_to_share_s"], 29.5)  # 0.9 x 60 = 54: back at t = 130


class ChurnTest(unittest.TestCase):
    def test_counting(self):
        ground = [assign(None, 1, "initial", [1, 2, 3]),     # before the start: not counted
                  assign(-20.0, 2, "infeasible", [2]),       # warm-up: counted apart
                  assign(30.0, 3, "infeasible", [2]),
                  lost(72.0, 3),
                  assign(72.0, 4, "lost", [1, 2, 3]),        # 3 is dead by then: not counted
                  assign(120.0, 5, "infeasible", [1])]
        agents = {a: agent_log(track(0.0, 180.0, 50.0 * a, 0.0)) for a in (1, 2, 3)}
        run = make_run(agents, ground, kill_agent=3, kill_wall=T0_WALL + 60.0)
        churn = metrics.churn_block(run)
        self.assertAlmostEqual(churn["before_per_agent_min"], 1 / 3.0)   # 1 change, 3 agent-min
        self.assertAlmostEqual(churn["after_per_agent_min"], 3 / 4.0)    # 3 changes, 2 x 2 min
        self.assertEqual(churn["replans_by_trigger"], {"infeasible": 2, "lost": 1})
        self.assertEqual(churn["replans_warmup_by_trigger"], {"infeasible": 1})
        self.assertEqual(churn["extra_changes_after_loss"], 1)
        self.assertEqual(churn["epoch_mismatch"]["1"], [3, 0])           # logs stayed at 1


class SafetyTest(unittest.TestCase):
    def test_separation_ignores_the_start_line(self):
        a = track(-5.0, 180.0, 100.0, 0.0)
        b = track(-5.0, 0.0, 110.0, 0.0) + track(0.0, 180.0, 150.0, 0.0)
        b = [(t, 107.0 if 5.0 <= t < 6.0 else x, y, z, yaw) for t, x, y, z, yaw in b]
        safety = metrics.safety_block(make_run({1: agent_log(a), 2: agent_log(b)}))
        self.assertAlmostEqual(safety["min_separation_m"], 7.0)
        self.assertAlmostEqual(safety["min_separation_from_10s_m"], 50.0)
        self.assertAlmostEqual(safety["below_d_min_s"], 1.0, delta=STEP_S)

    def test_flag_time(self):
        log = agent_log(track(0.0, 10.0, 0.0, 0.0), flags="filtered|deaf")
        rows = np.ones(len(log["t"]), dtype=bool)
        self.assertAlmostEqual(metrics.flag_seconds(log, rows, "deaf"), 9.95)
        self.assertEqual(metrics.flag_seconds(log, rows, "brake"), 0.0)


class MissionTest(unittest.TestCase):
    def test_priority_zone_gap(self):
        rows = (track(-60.0, 20.0, 150.0, 150.0) + track(20.0, 51.0, 250.0, 250.0)
                + track(51.0, 180.0, 150.0, 150.0))
        run = make_run({1: agent_log(rows)}, mission="B", priority_zone=[140, 140, 160, 160])
        zone = metrics.priority_zone_block(run, metrics.sightings(run), np.arange(0.0, 181.0))
        self.assertAlmostEqual(zone["longest_unseen_s"], 31.05)            # over the 30 s limit
        # whole zone seen within 30 s at every tick except t = 50 (last seen 19.95)
        self.assertAlmostEqual(zone["whole_zone_fresh_pct"], 100.0 * 180 / 181)

    def test_blackout(self):
        log = agent_log(track(0.0, 180.0, 50.0, 50.0), flags="")
        deaf = (log["t"] >= 60.0) & (log["t"] < 75.0)
        log["flags"] = ["deaf" if d else "" for d in deaf]
        log["status"] = ["hold" if d else "cover" for d in deaf]
        other = agent_log(track(0.0, 180.0, 250.0, 250.0))
        run = make_run({1: log, 2: other}, mission="C",
                       blackout={"agent": 1, "t0": 60.0, "t1": 70.0})
        m, _, _ = metrics.compute(run)
        c = m["mission_c"]
        self.assertFalse(c["declared_lost"])
        self.assertEqual(c["replans_from_blackout"], 0)
        self.assertAlmostEqual(c["deaf_s"], 15.0)
        self.assertAlmostEqual(c["back_in_cover_after_s"], 5.0)


def write_run_dir(path, run):
    """Write an in-memory run as the files a real run leaves, with a cut-off last log line."""
    os.makedirs(path)
    with open(os.path.join(path, "meta.json"), "w") as f:
        json.dump(run.meta, f)
    for agent_id, log in run.agents.items():
        with open(os.path.join(path, f"agent_{agent_id}.csv"), "w") as f:
            f.write(",".join(AGENT_CSV_COLUMNS) + "\n")
            for i in range(len(log["t"])):
                f.write(",".join(str(log[name][i]) for name in AGENT_CSV_COLUMNS) + "\n")
            f.write("1234.5,12")                          # what a SIGKILL can leave
    with open(os.path.join(path, "ground.jsonl"), "w") as f:
        f.writelines(json.dumps(e) + "\n" for e in run.ground)
    with open(os.path.join(path, "radio.jsonl"), "w") as f:
        f.write(json.dumps({"event": "config"}) + "\n")
        f.write(json.dumps(dict(run.radio, event="stats", pairs={})) + "\n")


def loss_run(seed):
    """Three agents hovering apart; agent 3 is lost at 80 s and declared lost at 92 s."""
    agents = {}
    for a in (1, 2, 3):
        end_s = 80.0 if a == 3 else 180.0
        agents[a] = agent_log(track(0.0, end_s, 50.0 + 100.0 * (a - 1), 150.0))
        agents[a]["epoch"] = np.where(agents[a]["t"] >= 93.0, 2.0, 1.0)
    new_plan = {"1": {"share": [0, 150, 0, 300], "slots": [], "loop_from": 0},
                "2": {"share": [150, 300, 0, 300], "slots": [], "loop_from": 0}}
    ground = [assign(None, 1, "initial", [1, 2, 3]), lost(92.0, 3),
              assign(92.0, 2, "lost", [1, 2], new_plan)]
    run = make_run(agents, ground, seed=seed, kill_agent=3, kill_wall=T0_WALL + 80.0)
    run.name = f"seed_{seed}"
    return run


class ReportTest(unittest.TestCase):
    def test_two_runs_end_to_end(self):
        with tempfile.TemporaryDirectory() as tmp:
            run_dirs = []
            for seed in (1, 2):
                run_dirs.append(os.path.join(tmp, f"seed_{seed}"))
                write_run_dir(run_dirs[-1], loss_run(seed))
            loaded = logs.load_run(run_dirs[0])
            self.assertEqual(len(loaded.agents[1]["t"]), 3600)   # cut-off line dropped
            m = metrics.analyse(run_dirs[0])
            self.assertEqual(m["loss"]["milestones_s"]["lost_declared"], 12.0)
            self.assertEqual(m["loss"]["milestones_s"]["assignment_received"], 13.0)
            out = os.path.join(tmp, "set")
            report.write_set(out, "test set", run_dirs, [], [])
            with open(os.path.join(out, "results.md")) as f:
                text = f.read()
            self.assertIn("2 of 2 runs recovered", text)
            self.assertTrue(os.path.exists(os.path.join(out, "runs", "seed_2.json")))
            report.write_comparison([out, out])
            self.assertTrue(os.path.exists(os.path.join(tmp, "comparison.md")))

    def test_fake_runs_are_refused_under_results(self):
        results_dir = os.path.join(report.REPO, "results", "_never_written")
        with self.assertRaises(SystemExit):
            report.refuse_fake_in_results(results_dir, {"gazebo", "fake"})
        report.refuse_fake_in_results("/tmp/elsewhere", {"fake"})       # allowed outside


class ScenarioLabelTest(unittest.TestCase):
    def test_labels_name_sums_windows_and_run_length(self):
        with tempfile.TemporaryDirectory() as tmp:
            write_run_dir(os.path.join(tmp, "r"), loss_run(1))
            m = metrics.analyse(os.path.join(tmp, "r"))
        text = scenarios.measured(m, {"outcome": "ok", "mission_s": 240.0})
        for label in ("summed over agents", "after (loss + 60 s to end)", "dip (loss to end)",
                      "warm-up: none", "run 240 s, longer than 180"):
            self.assertIn(label, text)
        self.assertIn("empty plan", scenarios.EXPECTED["mute"])


if __name__ == "__main__":
    unittest.main()
