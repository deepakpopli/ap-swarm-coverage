"""Tests for the lane-group layout of swarm/allocator.py: fixed lanes, contiguous groups, one
loop per agent, for every fleet size. Pure functions: no sockets, no clock."""
import itertools
import math
import random

import pytest

from swarm import allocator, config
from swarm.allocator import allocate

LANES = allocator.area_lanes()


def starts(n):
    return {i + 1: config.start_xy(i, n) for i in range(n)}


def plan(ids, positions, previous=None, blacklist=frozenset(), reverse=False):
    return allocate(ids, positions, set(blacklist), previous, reverse=reverse)


def lane_xs(asg):
    """x of every lane end flown in the loop (slots on a lane end's y)."""
    y_ends = {round(allocator._end_inset(), 2), round(300 - allocator._end_inset(), 2)}
    return {p[0] for p in asg.slots[asg.loop_from:] if p[1] in y_ends}


def test_four_lanes_at_50m():
    assert LANES == [37.5, 112.5, 187.5, 262.5]


@pytest.mark.parametrize("n, sizes", [(1, [4]), (2, [2, 2]), (3, [1, 1, 2]), (4, [1, 1, 1, 1]),
                                      (5, [0, 1, 1, 1, 1])])
def test_groups_for_every_n(n, sizes):
    """Contiguous groups that share out all lanes once; N = 5: one spare with no slots."""
    result = plan(range(1, n + 1), starts(n))
    flown = [lane_xs(asg) for asg in result.values()]
    assert sorted(len(g) for g in flown) == sizes
    assert sorted(x for g in flown for x in g) == LANES
    for g in flown:
        assert [x for x in LANES if min(g, default=0) <= x <= max(g, default=0)] == sorted(g)
    assert set(result[1].to_dict()) == {"share", "slots", "loop_from"}   # no schedule


def test_n3_east_pair_to_agent_3():
    result = plan([1, 2, 3], starts(3))
    assert [sorted(lane_xs(result[a])) for a in (1, 2, 3)] == [[37.5], [112.5], [187.5, 262.5]]


def on_loops(result, fallback):
    """Each agent at the first slot of its loop (the spare where it was)."""
    return {a: tuple(asg.slots[asg.loop_from]) if asg.slots else fallback[a]
            for a, asg in result.items()}


@pytest.mark.parametrize("victim", [1, 2, 4, 5])
def test_n5_spare_holds_and_takes_a_freed_lane(victim):
    """The spare (no slots) gets the lane of a lost agent; the others keep their plans."""
    before = plan(range(1, 6), starts(5))
    spare = [a for a, asg in before.items() if not asg.slots]
    assert spare == [3]
    alive = [a for a in before if a != victim]
    after = plan(alive, on_loops(before, starts(5)), previous=before)
    assert lane_xs(after[spare[0]]) == lane_xs(before[victim])
    assert all(after[a] is before[a] for a in alive if a != spare[0])


def test_determinism():
    """Same result for any input order."""
    pos = {1: (40.0, 100.0), 2: (150.0, 20.0), 3: (260.0, 200.0)}
    assert plan([3, 1, 2], pos) == plan([2, 3, 1], dict(reversed(list(pos.items()))))


@pytest.mark.parametrize("victim", [1, 2, 3])
def test_lanes_fixed_across_membership(victim):
    """A loss and a return only change who flies which of the same four lanes."""
    rnd = random.Random(victim)
    before = plan([1, 2, 3], starts(3))
    pos = {a: tuple(rnd.choice(before[a].slots)) for a in before}
    alive = [a for a in (1, 2, 3) if a != victim]
    after = plan(alive, pos, previous=before)
    assert sorted(len(lane_xs(after[a])) for a in alive) == [2, 2]
    assert sorted(x for a in alive for x in lane_xs(after[a])) == LANES
    for a in alive:                                  # an unchanged group keeps its plan
        if lane_xs(after[a]) == lane_xs(before[a]):
            assert after[a] is before[a]
    back = plan([1, 2, 3], pos, previous=after)
    assert sorted(x for asg in back.values() for x in lane_xs(asg)) == LANES


def test_kept_when_nothing_changes():
    before = plan([1, 2, 3], starts(3))
    again = plan([1, 2, 3], on_loops(before, starts(3)), previous=before)
    assert all(again[a] is before[a] for a in before)


def test_blacklist_changes_only_the_reporter():
    before = plan([1, 2, 3], starts(3))
    bad = allocator.slot_key(before[3].slots[-1])
    after = plan([1, 2, 3], starts(3), previous=before, blacklist={(3, bad)})
    assert after[1] is before[1] and after[2] is before[2]
    assert bad not in {allocator.slot_key(p) for p in after[3].slots}


def active_gap_m(result, pos):
    def first(a):
        return allocator._first_slots(result[a], (list(pos[a]), (0.0, 0.0), None))
    return min(allocator._gap_m(first(a), first(b))
               for a, b in itertools.combinations(sorted(result), 2))


def test_active_slots_spaced_in_the_swap_run():
    """Positions at the swap re-plan of a Gazebo swap run, where agents 1 and 3 were once
    issued transit slots 0.2 m apart."""
    pos = {1: (74.99, 244.89), 2: (125.0, 275.94), 3: (225.0, 245.3)}
    first = plan([1, 2, 3], starts(3))
    swapped = plan([1, 2, 3], pos, previous=first, reverse=True)
    assert active_gap_m(swapped, pos) >= allocator.ACTIVE_GAP_M - 1e-6


@pytest.mark.parametrize("n", [2, 3, 4, 5])
def test_active_slots_spaced_from_the_start_line(n):
    assert active_gap_m(plan(range(1, n + 1), starts(n)), starts(n)) >= allocator.ACTIVE_GAP_M


@pytest.mark.parametrize("n", [1, 2, 3, 4, 5])
@pytest.mark.parametrize("where", ["start", "far corner"])
def test_legs_bounded_and_loop_inside_share(n, where):
    positions = starts(n) if where == "start" else {a: (299.0, 299.0) for a in range(1, n + 1)}
    for a, asg in plan(range(1, n + 1), positions).items():
        points = [list(positions[a])] + asg.slots
        legs = list(zip(points, points[1:]))
        if asg.slots:
            legs.append((asg.slots[-1], asg.slots[asg.loop_from]))   # the closing leg
        assert all(math.dist(p, q) <= config.SLOT_SPACING_M for p, q in legs)
        for x, y in asg.slots[asg.loop_from:]:
            assert asg.share[0] <= x <= asg.share[1] and asg.share[2] <= y <= asg.share[3]
