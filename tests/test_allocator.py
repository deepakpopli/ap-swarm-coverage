"""Tests for swarm/allocator.py. Pure functions: no sockets, no clock."""
import math

import pytest

from swarm import allocator, config, sensor
from swarm.allocator import allocate, slot_key

MIDDLE_ZONE = (120.0, 120.0, 180.0, 180.0)   # xmin, ymin, xmax, ymax
CORNER_ZONE = (0.0, 0.0, 60.0, 60.0)


def starts(n):
    return {i + 1: config.start_xy(i, n) for i in range(n)}


def legs(asg, xy):
    """Every leg flown: position -> first slot, slot to slot, last slot -> slots[loop_from]."""
    points = [list(xy[:2])] + asg.slots
    return list(zip(points, points[1:])) + [(asg.slots[-1], asg.slots[asg.loop_from])]


def assert_tiles_area(shares):
    shares = sorted(shares)
    assert shares[0][0] == config.AREA[0] and shares[-1][1] == config.AREA[1]
    for west, east in zip(shares, shares[1:]):
        assert west[1] == east[0]                        # no gap, no overlap
    for share in shares:
        assert share[2:] == list(config.AREA[2:]) and share[1] > share[0]


@pytest.fixture
def at_alt(monkeypatch):
    """Plan lanes for the camera footprint at another altitude."""
    def set_alt(alt_m):
        swath_m, along_m = sensor.lane_footprint(alt_m)
        monkeypatch.setattr(allocator, "SWATH_M", swath_m)
        monkeypatch.setattr(allocator, "ALONG_M", along_m)
    return set_alt


def test_determinism():
    blacklist = {(2, (125, 150))}
    a = allocate([3, 1, 2], starts(3), blacklist)
    b = allocate([2, 3, 1], dict(reversed(starts(3).items())), set(blacklist))
    assert a == b and list(a) == list(b)


@pytest.mark.parametrize("alt_m, lanes, y_ends", [
    (50, [37.5, 112.5, 187.5, 262.5], (19.0, 281.0)),   # 85 m swath: 4 lanes
    (60, [50.0, 150.0, 250.0], (23.5, 276.5)),          # 102 m swath: 3 lanes
])
def test_lanes_from_swath(at_alt, alt_m, lanes, y_ends):
    at_alt(alt_m)
    assert allocator.area_lanes() == lanes
    asg = allocate([1, 2, 3], starts(3), set())[2]
    loop = asg.slots[asg.loop_from:]
    assert (min(y for _, y in loop), max(y for _, y in loop)) == y_ends


def test_four_agents_fly_one_lane_out_and_back(at_alt):
    at_alt(50)
    for asg in allocate(range(1, 5), starts(4), set()).values():
        loop = asg.slots[asg.loop_from:]
        assert len({x for x, _ in loop}) == 1
        assert len({tuple(p) for p in loop}) < len(loop)   # the way back repeats the points


def test_previous_plan_kept_for_unaffected_agents():
    base = allocate([1, 2, 3], starts(3), set())
    moved = {a: (x + 7.0, y + 30.0) for a, (x, y) in starts(3).items()}
    blacklist = {(2, slot_key(base[2].slots[3]))}
    result = allocate([1, 2, 3], moved, blacklist, previous=base)
    assert result[1] is base[1] and result[3] is base[3]
    assert result[2] != base[2]


def test_previous_not_kept_when_share_changes():
    base = allocate([1, 2, 3], starts(3), set())
    result = allocate([1, 2], starts(3), set(), previous=base)
    assert result[1] is not base[1] and result[2] is not base[2]


@pytest.mark.parametrize("zone", [None, MIDDLE_ZONE, CORNER_ZONE])
def test_blacklisted_pair_never_issued(zone):
    positions = starts(3)
    base = allocate([1, 2, 3], positions, set(), priority_zone=zone)
    for a in (1, 2, 3):
        for p in base[a].slots:                          # every slot, one at a time
            blacklist = {(a, slot_key(p))}
            result = allocate([1, 2, 3], positions, blacklist, priority_zone=zone)
            assert slot_key(p) not in [slot_key(q) for q in result[a].slots]
            for q, r in legs(result[a], positions[a]):
                assert math.dist(q, r) <= config.SLOT_SPACING_M
            for other in {1, 2, 3} - {a}:
                assert result[other] == base[other]


def test_whole_zone_lane_blacklisted_means_hold():
    positions = starts(3)
    base = allocate([1, 2, 3], positions, set(), priority_zone=MIDDLE_ZONE)
    blacklist = {(2, slot_key(p)) for p in base[2].slots}
    asg = allocate([1, 2, 3], positions, blacklist, priority_zone=MIDDLE_ZONE)[2]
    assert asg.slots == [list(positions[2])] and asg.loop_from == 0


# ---- Mission B

def revisit_s(asg, y):
    """Revisit period of a point at height y under a single out-and-back lane: twice the
    distance from where the point leaves the footprint to the far lane end, plus the turn."""
    loop_ys = [p[1] for p in asg.slots[asg.loop_from:]]
    gaps_m = [abs(end - y) - allocator.ALONG_M / 2 for end in (min(loop_ys), max(loop_ys))]
    return 2 * max(gaps_m) / config.WP_SPD_MS + allocator.TURN_S


def lane_of(asg):
    loop = asg.slots[asg.loop_from:]
    xs = {x for x, _ in loop}
    assert len(xs) == 1
    return xs.pop(), min(y for _, y in loop), max(y for _, y in loop)


@pytest.mark.parametrize("zone, owner, lane_x", [(MIDDLE_ZONE, 2, 150.0),
                                                   (CORNER_ZONE, 1, 18.75)])
def test_priority_zone(at_alt, zone, owner, lane_x):
    """The owner's lane covers the zone within 30 s, half way between two area lanes; the
    others fly the whole area's lanes in groups, as if they were the whole fleet."""
    at_alt(50)
    result = allocate([1, 2, 3], starts(3), set(), priority_zone=zone)
    asg = result[owner]
    x, y_south, y_north = lane_of(asg)
    half_swath, half_along = allocator.SWATH_M / 2, allocator.ALONG_M / 2
    assert x == lane_x
    assert x - half_swath <= zone[0] and zone[2] <= x + half_swath
    assert y_south - half_along <= zone[1] and zone[3] <= y_north + half_along
    for y in range(int(zone[1]), int(zone[3]) + 1):
        assert revisit_s(asg, y) <= config.PRIORITY_REVISIT_S
    others = allocate([a for a in (1, 2, 3) if a != owner], starts(3), set())
    for a in others:
        assert result[a].share == others[a].share


def test_zone_owner_kept_and_replaced_on_loss():
    before = allocate([1, 2, 3], starts(3), set(), priority_zone=MIDDLE_ZONE)
    after = allocate([2, 3], starts(3), set(), previous=before, priority_zone=MIDDLE_ZONE)
    assert after[2] is before[2]                        # the owner keeps its lane
    assert len(after[3].slots) > len(before[3].slots)   # the other flies all four lanes
    near_zone = {1: (150.0, 100.0), 3: (250.0, 0.0)}
    lost = allocate([1, 3], near_zone, set(), previous=before, priority_zone=MIDDLE_ZONE)
    assert lane_of(lost[1]) == lane_of(before[2])       # the nearest survivor takes the zone
