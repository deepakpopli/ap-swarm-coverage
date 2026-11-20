"""Coverage controller tests: an ideal point mass (pos += v * dt) in stepped time."""
import math

from swarm import config
from swarm.coverage import CoverageController

TICK_S = 1.0 / config.CONTROL_HZ


def assign(epoch, slots, loop_from=0):
    return {"epoch": epoch, "share": [0, 300, 0, 300], "slots": slots, "loop_from": loop_from}


class Run:
    """A controller and a point mass that flies exactly what it is told (or stays put)."""

    def __init__(self, xy=(0.0, 0.0)):
        self.cov = CoverageController()
        self.xy = xy
        self.t = 0.0
        self.events = []
        self.speeds = []       # (active slot, commanded speed) per tick

    def assign(self, epoch, slots, loop_from=0):
        return self.cov.on_assign(assign(epoch, slots, loop_from), self.t, self.xy)

    def fly(self, seconds, move=True, deaf=False):
        for _ in range(round(seconds / TICK_S)):
            out = self.cov.step(self.t, self.xy, deaf)
            self.events += out.events
            self.speeds.append((self.cov.idx, math.hypot(out.vx, out.vy)))
            if move:
                self.xy = (self.xy[0] + out.vx * TICK_S, self.xy[1] + out.vy * TICK_S)
            self.t += TICK_S
        return out

    def named(self, name):
        return [e for e in self.events if e["event"] == name]


def test_follows_the_plan_then_repeats_the_loop():
    run = Run()
    run.assign(1, [[0, 50], [50, 50], [50, 100], [100, 100], [100, 50]], loop_from=1)
    run.fly(120)
    reached = [e["slot"] for e in run.named("slot_reached")]
    assert reached[:9] == [0, 1, 2, 3, 4, 1, 2, 3, 4]
    assert reached.count(0) == 1                            # the transit slot is flown once
    assert not run.named("infeasible")


def test_full_speed_through_collinear_slots_slow_into_a_sharp_turn():
    run = Run((0.0, -30.0))
    run.assign(1, [[0, 0], [0, 50], [0, 100], [0, 150], [100, 150]])
    run.fly(25)
    last_speed = {slot: speed for slot, speed in run.speeds}   # speed when the slot switched
    assert math.isclose(last_speed[1], config.WP_SPD_MS)
    assert math.isclose(last_speed[2], config.WP_SPD_MS)
    assert last_speed[3] < 6.0                              # 90 degree corner at slot 3


def test_empty_slot_list_means_hold():
    run = Run()
    run.assign(1, [])
    out = run.fly(1)
    assert (out.vx, out.vy, out.idle, run.cov.idx) == (0.0, 0.0, True, -1)


def test_identical_replan_keeps_the_slot_and_its_timer():
    """Re-plans every 10 s with the same slots must not hide a stuck slot."""
    run = Run()
    slots = [[0, 0], [0, 200], [100, 200]]
    run.assign(1, slots)
    run.fly(1)                                              # slot 0 reached, slot 1 active
    run.fly(10, move=False)
    run.assign(2, slots)
    assert run.cov.idx == 1
    run.fly(5, move=False)
    assert run.named("infeasible") == [
        {"event": "infeasible", "epoch": 2, "slot": 1, "slot_xy": [0, 200]}]


def test_different_plan_starts_at_the_nearest_slot():
    """Single-lane loop: the point near us appears twice; the lowest index wins."""
    run = Run((0.0, 98.0))
    run.assign(1, [[0, 0], [100, 0]])
    run.assign(2, [[0, 0], [0, 100], [0, 200], [0, 100]], loop_from=0)
    assert run.cov.idx == 1
    out = run.fly(TICK_S)
    assert out.vy > 0.0                                     # it does not turn back


def test_exactly_one_infeasible_per_stuck_slot_then_hold_and_resend():
    run = Run()
    run.assign(1, [[0, 200], [0, 300]])
    run.fly(16, move=False)
    assert len(run.named("infeasible")) == 1
    sends = 0
    for _ in range(round(5 / TICK_S)):                      # 5 s: re-sent about once a second
        out = run.fly(TICK_S, move=False)
        sends += run.cov.infeasible_due(run.t) is not None
    assert out.idle and (out.vx, out.vy) == (0.0, 0.0)
    assert 4 <= sends <= 6
    run.fly(30, move=False)
    assert len(run.named("infeasible")) == 1
    run.assign(2, [[0, 200], [0, 300]])                     # a newer epoch ends the re-sending
    assert run.cov.infeasible_due(run.t + 10) is None


def test_timer_pauses_while_deaf():
    """Mission C: a 10 s blackout must not blacklist a good slot."""
    run = Run()
    run.assign(1, [[0, 200], [0, 300]])
    run.fly(10, move=False, deaf=True)
    run.fly(10, move=False)
    assert not run.named("infeasible")
    run.fly(6, move=False)
    assert len(run.named("infeasible")) == 1


def test_timer_runs_while_the_filter_blocks_us():
    """Coverage asks to move, the vehicle does not (filter blocking): that is infeasible."""
    run = Run()
    run.assign(1, [[0, 200], [0, 300]])
    out = run.fly(14, move=False)
    assert math.hypot(out.vx, out.vy) > 1.0 and not run.named("infeasible")
    run.fly(2, move=False)
    assert len(run.named("infeasible")) == 1


# ---- stuck: no real progress to the active slot (the safety filter's "blocked")

def test_stuck_after_no_progress_while_wanting_to_move():
    run = Run()
    run.assign(1, [[0, 200], [0, 300]])
    assert not run.fly(2.9, move=False).stuck
    assert run.fly(0.2, move=False).stuck


def test_sliding_sideways_is_not_progress():
    """The command keeps a large component toward the slot, but the vehicle only slides
    sideways at 3 m/s: the old commanded-progress test never fired in this case."""
    run = Run()
    run.assign(1, [[0, 200], [0, 300]])
    for _ in range(round(3.2 / TICK_S)):
        out = run.cov.step(run.t, run.xy, False)
        run.xy = (run.xy[0] + 3.0 * TICK_S, run.xy[1])
        run.t += TICK_S
    assert math.hypot(out.vx, out.vy) > 1.0 and out.stuck


def test_slow_real_progress_is_not_stuck():
    run = Run()
    run.assign(1, [[0, 200], [0, 300]])
    for _ in range(round(10 / TICK_S)):
        out = run.cov.step(run.t, run.xy, False)
        run.xy = (run.xy[0], run.xy[1] + 1.0 * TICK_S)     # 1 m/s toward the slot
        run.t += TICK_S
        assert not out.stuck


def test_not_stuck_while_deaf_or_at_the_slot():
    run = Run()
    run.assign(1, [[0, 200], [0, 300]])
    assert not run.fly(5, move=False, deaf=True).stuck
    run.xy = (0.0, 197.0)                                  # within SLOT_REACH_M: reached
    run.fly(0.05, move=False)
    assert run.cov.idx == 1 and not run.fly(2.0).stuck     # new slot: the watch restarts


def test_reversal_at_cruise_is_not_stuck():
    """At a lane end the next slot is behind: the vehicle brakes (the distance grows), then
    comes back. Replayed on a 4 m/s^2 point mass: about 6 s without getting closer."""
    run = Run((0.0, 0.0))
    run.assign(1, [[0, -100], [0, -200]])
    vy = 11.5
    for _ in range(round(8 / TICK_S)):
        out = run.cov.step(run.t, run.xy, False)
        assert not out.stuck
        vy = max(-11.5, vy - 4.0 * TICK_S)
        run.xy = (0.0, run.xy[1] + vy * TICK_S)
        run.t += TICK_S
