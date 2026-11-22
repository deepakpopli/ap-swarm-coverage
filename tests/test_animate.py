"""analysis/animate.py: the playback clock, lane owners and liveness, without rendering."""
import unittest

from analysis import animate
from analysis.logs import Run

MOMENTS = {"kill": 74.0, "suspect": 76.2, "declared": 86.2, "recovered": 123.0}


def run_with(ground):
    meta = {"mission_s": 180.0, "kill_agent": 2, "kill_wall": 1074.0, "t0_wall": 1000.0}
    return Run("seed_x", meta, {}, ground)


class PlaybackTest(unittest.TestCase):
    def test_segments_cover_the_mission_without_gaps(self):
        segments = animate.speed_segments(180.0, MOMENTS)
        self.assertEqual(segments[0][0], 0.0)
        self.assertEqual(segments[-1][1], 180.0)
        for (_, stop_s, _), (start_s, _, _) in zip(segments, segments[1:]):
            self.assertEqual(stop_s, start_s)

    def test_loss_plays_slowest(self):
        speeds = {s for a, b, s in animate.speed_segments(180.0, MOMENTS) if a <= 80.0 < b}
        self.assertEqual(speeds, {animate.EVENT_SPEED})

    def test_video_is_under_a_minute_with_one_hold_per_moment(self):
        plan = animate.frame_plan(180.0, MOMENTS)
        self.assertLess(animate.video_seconds(plan), 60.0)
        holds = [t for t, n, _ in plan if n > 1]
        self.assertEqual(len(holds), 1 + len(MOMENTS))       # the intro and each moment
        times = [t for t, _, _ in plan]
        self.assertEqual(times, sorted(times))               # metrics.last_seen_at needs this

    def test_no_loss_plays_at_one_speed(self):
        none = dict.fromkeys(MOMENTS)
        self.assertEqual(animate.speed_segments(180.0, none),
                         [(0.0, 180.0, animate.QUIET_SPEED)])


class RunFactsTest(unittest.TestCase):
    def test_lane_owners_skip_the_transit_slot(self):
        event = {"assignments": {"1": {"slots": [[35.2, 233.4], [37.5, 19.0], [112.5, 19.0]],
                                       "loop_from": 1},
                                 "3": {"slots": [[187.5, 19.0], [262.5, 19.0]], "loop_from": 0}}}
        owners = animate.lane_owners(event, [37.5, 112.5, 187.5, 262.5])
        self.assertEqual(owners, {0: 1, 1: 1, 2: 3, 3: 3})

    def test_liveness_follows_the_ground_and_kill_follows_meta(self):
        ground = [{"event": "liveness", "agent": 2, "t_mission": 76.2, "to": "suspect"},
                  {"event": "liveness", "agent": 2, "t_mission": 86.2, "to": "lost"}]
        run = run_with(ground)
        self.assertEqual(animate.liveness_at(run, 2, 75.0), "alive")
        self.assertEqual(animate.liveness_at(run, 2, 80.0), "suspect")
        self.assertEqual(animate.liveness_at(run, 2, 90.0), "lost")
        self.assertTrue(animate.killed_at(run, 2, 74.0))
        self.assertFalse(animate.killed_at(run, 2, 73.9))
        self.assertFalse(animate.killed_at(run, 1, 100.0))

    def test_captions_stack_downwards(self):
        story = [(0.0, "a\nb"), (1.0, "c"), (2.0, "d\ne")]
        tops = animate.caption_tops(story)
        self.assertEqual(tops, sorted(tops, reverse=True))
        self.assertGreater(tops[0] - tops[1], tops[1] - tops[2])   # 2 lines take more room


if __name__ == "__main__":
    unittest.main()
