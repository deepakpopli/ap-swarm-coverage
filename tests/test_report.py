"""Hand-checkable tests of the results.md helpers in analysis/report.py."""
import unittest

import numpy as np

from analysis import report


def summary(values):
    return report.summarise(values)


class ReferenceCellTest(unittest.TestCase):
    def test_one_run_is_a_single_value(self):
        self.assertEqual(report.reference_cell(summary([97.5]), unit=" %"), "97.5 % (1 run)")

    def test_several_runs_give_median_range_and_n(self):
        self.assertEqual(report.reference_cell(summary([97.0, 97.5, 98.0])),
                         "97.5 [97.0, 98.0] (3 runs)")

    def test_no_run_is_a_dash(self):
        self.assertEqual(report.reference_cell(summary([])), "-")


class SpeedTest(unittest.TestCase):
    def test_constant_speed_from_positions(self):
        t_s = np.arange(0.0, 5.0, 0.05)
        speed_ms = report.max_speed_over_step(t_s, 3.0 * t_s, 4.0 * t_s)
        self.assertAlmostEqual(speed_ms, 5.0, places=6)

    def test_too_short_a_log_has_no_speed(self):
        self.assertIsNone(report.max_speed_over_step(np.array([0.0, 0.5]), np.zeros(2),
                                                     np.zeros(2)))


class LanesTest(unittest.TestCase):
    def test_lanes_skip_the_transit_slots(self):
        ground = [{"event": "liveness"},
                  {"event": "assign", "assignments": {"3": {
                      "slots": [[50.0, 0.0], [187.5, 19.0], [187.5, 84.5], [262.5, 84.5]],
                      "loop_from": 1}}}]
        self.assertEqual(report.first_plan_lanes(ground, 3), [187.5, 262.5])
        self.assertIsNone(report.first_plan_lanes(ground, 1))


class AfterRecoveryTest(unittest.TestCase):
    def test_window_starts_at_the_recovery(self):
        ticks = np.arange(0.0, 10.0)
        coverage = np.array([90, 90, 60, 70, 80, 95, 95, 97, 97, 97], dtype=float)
        mean_pct, min_pct = report.coverage_after(ticks, coverage, 5.0)
        self.assertAlmostEqual(mean_pct, 96.2)
        self.assertEqual(min_pct, 95.0)

    def test_not_recovered_has_no_window(self):
        self.assertEqual(report.coverage_after(np.arange(3.0), np.ones(3), None), (None, None))


class RecoveryLineTest(unittest.TestCase):
    def test_never_below_is_separated_from_the_dipped_runs(self):
        all_runs = summary([0.0, 0.0, 40.0, 60.0, None])
        dipped = summary([report.recovery_if_dipped(r, d) for r, d in
                          ((0.0, False), (0.0, False), (40.0, True), (60.0, True), (None, True))])
        line = report.recovery_line("Recovery to 0.9 x P", all_runs, dipped, 3)
        self.assertIn("2 of 5 runs never fell below it", line)
        self.assertIn("3 fell below, 2 of them back within the run", line)
        self.assertIn("over those 2: 50.0 [40.0, 60.0] s", line)


if __name__ == "__main__":
    unittest.main()
