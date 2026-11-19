"""Tests for the "lanes" layout of swarm/allocator.py (fixed lanes, shared tour or lane groups)
and the shared-tour schedule in swarm/coverage.py. Pure functions: no sockets, no clock."""
import itertools
import math
import random

import pytest

from swarm import allocator, config
from swarm.allocator import allocate
from swarm.coverage import CoverageController

LANES = allocator.area_lanes()


def starts(n):
    return {i + 1: config.start_xy(i, n) for i in range(n)}


def plan(ids, positions, previous=None, now=0.0, blacklist=frozenset(), reverse=False):
    return allocate(ids, positions, set(blacklist), previous, layout="lanes", now=now,
                    reverse=reverse)


def lane_xs(asg):
    """x of every lane end flown in the loop (slots on a lane end's y)."""
    y_ends = {round(allocator._end_inset(), 2), round(300 - allocator._end_inset(), 2)}
    return {p[0] for p in asg.slots[asg.loop_from:] if p[1] in y_ends}


def test_four_lanes_at_50m():
    assert LANES == [37.5, 112.5, 187.5, 262.5]


@pytest.mark.parametrize("n, tour", [(2, False), (3, True), (4, False), (5, True)])
def test_which_option(n, tour):
    """N = 3: tour (period / 3 < 1+1+2 split). N = 2: split 2+2 (tour period / 2 is longer).
    N = 4: the tour would be shorter, but evenly spaced agents meet head-on on its closing leg,
    so 1+1+1+1. N = 5: more agents than lanes, tour."""
    assert allocator.use_tour(n) == tour
    result = plan(range(1, n + 1), starts(n))
    assert all((asg.team > 1) == tour for asg in result.values())


def test_tour_period_and_clearance():
    _, period = allocator.tour_schedule(allocator.tour_loop())
    assert period / 3 < config.COVERAGE_WINDOW_S          # three on the tour: 100 % coverage
    assert allocator.tour_clearance_m(3) >= 74.9           # they only pass on adjacent lanes


def test_determinism():
    pos = {1: (40.0, 100.0), 2: (150.0, 20.0), 3: (260.0, 200.0)}
    a = plan([3, 1, 2], pos, now=100.0)
    b = plan([2, 3, 1], dict(reversed(list(pos.items()))), now=100.0)
    assert a == b
    shifted = plan([1, 2, 3], pos, now=200.0)          # only the clock origin moves
    assert all(math.isclose(shifted[k].t_ref - a[k].t_ref, 100.0) for k in a)
    assert all(shifted[k].slots == a[k].slots for k in a)


def test_tour_evenly_spaced():
    result = plan([1, 2, 3], starts(3), now=0.0)
    period = result[1].period_s
    phases = sorted(asg.t_ref % period for asg in result.values())
    gaps = [b - a for a, b in zip(phases, phases[1:])] + [phases[0] + period - phases[-1]]
    assert all(math.isclose(g, period / 3, abs_tol=0.01) for g in gaps)


@pytest.mark.parametrize("victim", [1, 2, 3])
def test_lanes_fixed_across_membership(victim):
    """Every plan flies lanes of the fixed set; after a loss the two groups cover all lanes;
    after the agent comes back, the tour again."""
    rnd = random.Random(victim)
    before = plan([1, 2, 3], starts(3))
    assert all(lane_xs(asg) == set(LANES) for asg in before.values())
    pos = {a: tuple(rnd.choice(before[a].slots)) for a in before}
    alive = [a for a in (1, 2, 3) if a != victim]
    after = plan(alive, pos, previous=before, now=80.0)
    groups = [lane_xs(after[a]) for a in alive]
    assert sorted(x for g in groups for x in g) == LANES and len(groups[0]) == 2
    back = plan([1, 2, 3], pos, previous=after, now=100.0)
    assert all(lane_xs(asg) == set(LANES) and asg.team == 3 for asg in back.values())


def test_kept_when_nothing_changes():
    before = plan([1, 2, 3], starts(3))
    again = plan([1, 2, 3], {1: (0.0, 0.0), 2: (9.0, 9.0), 3: (99.0, 9.0)}, previous=before)
    assert all(again[a] is before[a] for a in before)


def test_blacklist_changes_only_the_reporter_and_keeps_its_phase():
    before = plan([1, 2, 3], starts(3))
    bad = allocator.slot_key(before[2].slots[-1])
    after = plan([1, 2, 3], starts(3), previous=before, now=50.0, blacklist={(2, bad)})
    assert after[1] is before[1] and after[3] is before[3]
    assert after[2].t_ref == before[2].t_ref
    assert bad not in {allocator.slot_key(p) for p in after[2].slots}


def active_gap_m(result, pos):
    return min(allocator._gap_m(result[a], list(pos[a]), result[b], list(pos[b]))
               for a, b in itertools.combinations(sorted(result), 2))


@pytest.mark.parametrize("layout", ["strips", "lanes"])
def test_active_slots_spaced_in_the_swap_run(layout):
    """gz_swap (run/int): positions at the swap re-plan. The prototype issued agents 1 and 3
    transit slots 0.2 m apart."""
    pos = {1: (74.99, 244.89), 2: (125.0, 275.94), 3: (225.0, 245.3)}
    first = allocate([1, 2, 3], starts(3), set(), layout=layout)
    swapped = allocate([1, 2, 3], pos, set(), first, reverse=True, layout=layout)
    assert active_gap_m(swapped, pos) >= allocator.ACTIVE_GAP_M - 1e-6


def test_tour_entries_spaced_from_the_start_line():
    for n in (3, 5):
        assert active_gap_m(plan(range(1, n + 1), starts(n)), starts(n)) >= allocator.ACTIVE_GAP_M


def test_legs_bounded():
    for n in (2, 3):
        for asg in plan(range(1, n + 1), starts(n)).values():
            loop = asg.slots
            legs = [math.dist(p, q) for p, q in zip(loop, loop[1:])]
            assert max(legs) <= config.SLOT_SPACING_M


# ---- the controller's schedule

def tour_msg(t_ref=100.0):
    return {"epoch": 1, "slots": [[0.0, 0.0], [0.0, 100.0], [50.0, 100.0], [50.0, 0.0]],
            "loop_from": 0, "t_ref": t_ref, "due_s": [0.0, 10.0, 16.0, 26.0],
            "period_s": 32.0}


def test_holds_at_entry_until_due():
    cc = CoverageController()
    cc.on_assign(tour_msg(), 90.0, (0.0, 0.0))
    out = cc.step(90.0, (0.0, 0.0), False)
    assert (out.vx, out.vy) == (0.0, 0.0) and not out.idle
    assert any(e["event"] == "hold" for e in out.events)
    assert cc.step(99.9, (0.0, 0.0), False).vy == 0.0
    out = cc.step(100.0, (0.0, 0.0), False)
    assert out.vy > 0.0 and cc.idx == 1


def test_no_hold_mid_lane_and_lap_count():
    cc = CoverageController()
    msg = tour_msg(t_ref=0.0)
    msg["slots"].insert(1, [0.0, 50.0])
    msg["due_s"] = [0.0, 5.0, 10.0, 16.0, 26.0]
    cc.on_assign(msg, 0.0, (0.0, 0.0))
    cc.step(0.0, (0.0, 0.0), False)
    cc.step(1.0, (0.0, 50.0), False)                 # 4 s early on a straight: no hold
    assert cc.idx == 2 and cc.hold_until is None
    for t, xy in [(2.0, (0.0, 100.0)), (20.0, (50.0, 100.0)), (30.0, (50.0, 0.0))]:
        cc.step(t, xy, False)
        cc.hold_until and cc.step(cc.hold_until, xy, False)
    assert cc.idx == 0 and cc.lap == 1


def test_plain_assignment_has_no_schedule():
    asg = allocate([1, 2], starts(2), set(), layout="lanes")[1]
    assert "due_s" not in asg.to_dict()
