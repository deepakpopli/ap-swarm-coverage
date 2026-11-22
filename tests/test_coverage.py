"""Coverage controller tests: an ideal point mass (pos += v * dt) in stepped time."""
import math

from swarm import config
from swarm.coverage import CoverageController

TICK_S = 1.0 / config.CONTROL_HZ


class Run:
    """A controller and a point mass that flies exactly what it is told (or stays put)."""

    def __init__(self, xy=(0.0, 0.0), vel=(0.0, 0.0)):
        self.cov = CoverageController()
        self.xy = xy
        self.vel = vel
        self.t = 0.0
        self.events = []
        self.speeds = []       # (active slot, commanded speed) per tick

    def assign(self, epoch, slots, loop_from=0):
        return self.cov.on_assign(epoch, slots, loop_from, self.xy, self.vel)

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
    run.assign(1, [[50, 0], [100, 0]])
    run.assign(2, [[0, 0], [0, 100], [0, 200], [0, 100]], loop_from=0)
    assert run.cov.idx == 1
    out = run.fly(TICK_S)
    assert out.vy > 0.0                                     # it does not turn back


def test_replan_keeps_the_slot_we_head_to():
    run = Run((0.0, 100.0), vel=(0.0, config.WP_SPD_MS))
    run.assign(1, [[0, 0], [0, 200]])
    run.fly(TICK_S, move=False)
    run.assign(2, [[0, 110], [100, 200], [0, 200], [0, 0]])
    assert run.cov.idx == 2


def test_replan_does_not_turn_us_round():
    """Flying north at cruise, the nearest new slot 10 m behind: start at the one ahead."""
    run = Run((0.0, 100.0), vel=(0.0, config.WP_SPD_MS))
    run.assign(1, [[0, 300]])
    run.assign(2, [[0, 90], [0, 170], [0, 250]])
    assert run.cov.idx == 1
    run.vel = (0.0, 0.0)                                    # standing: the nearest again
    run.assign(3, [[50, 90], [50, 170]])
    assert run.cov.idx == 0


def test_allocator_predicts_the_start_slot():
    """_first_slots must name the slot the controller really starts on (ACTIVE_GAP_M)."""
    from swarm.allocator import Assignment, _first_slots
    asg = Assignment([0, 300, 0, 300], [[0, 90], [0, 170], [0, 250], [50, 250]], 0)
    run = Run((0.0, 100.0), vel=(0.0, config.WP_SPD_MS))
    run.assign(1, asg.slots)
    assert _first_slots(asg, (run.xy, run.vel, None)) == [asg.slots[run.cov.idx]] == [[0, 170]]


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
