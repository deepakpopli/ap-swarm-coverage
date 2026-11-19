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


@pytest.mark.parametrize("n", [1, 2, 3, 4, 5, 6])
def test_strips_tile_area(n):
    assert_tiles_area([asg.share for asg in allocate(range(1, n + 1), starts(n), set()).values()])


def test_strips_handed_out_west_to_east():
    positions = {1: (280.0, 0.0), 2: (10.0, 0.0), 3: (150.0, 50.0), 4: (150.0, 0.0)}
    result = allocate([1, 2, 3, 4], positions, set())
    assert sorted(result, key=lambda a: result[a].share[0]) == [2, 3, 4, 1]   # tie: by id


@pytest.mark.parametrize("n", [1, 2, 3, 4, 5])
@pytest.mark.parametrize("where", ["start", "far corner"])
def test_spacing_bound_and_loop_closure(n, where):
    positions = starts(n) if where == "start" else {a: (299.0, 299.0) for a in range(1, n + 1)}
    for a, asg in allocate(range(1, n + 1), positions, set()).items():
        for p, q in legs(asg, positions[a]):            # includes the closing leg of the loop
            assert math.dist(p, q) <= config.SLOT_SPACING_M
        share = asg.share
        for x, y in asg.slots[asg.loop_from:]:
            assert share[0] <= x <= share[1] and share[2] <= y <= share[3]


@pytest.mark.parametrize("alt_m, lanes, y_ends", [
    (50, [125.0, 175.0], (19.0, 281.0)),     # 85 m swath: a 100 m strip needs 2 lanes
    (60, [150.0], (23.5, 276.5)),            # 102 m swath: 1 lane
])
def test_lanes_from_swath(at_alt, alt_m, lanes, y_ends):
    at_alt(alt_m)
    asg = allocate([1, 2, 3], starts(3), set())[2]
    loop = asg.slots[asg.loop_from:]
    assert sorted({x for x, _ in loop}) == lanes
    assert (min(y for _, y in loop), max(y for _, y in loop)) == y_ends


def test_three_to_two():
    result = allocate([1, 3], starts(3), set())
    assert result[1].share == [0.0, 150.0, 0.0, 300.0]
    assert result[3].share == [150.0, 300.0, 0.0, 300.0]


@pytest.mark.parametrize("n, width_m", [(4, 75.0), (5, 60.0)])
def test_four_and_five_agents_fly_one_lane_out_and_back(at_alt, n, width_m):
    at_alt(50)
    for asg in allocate(range(1, n + 1), starts(n), set()).values():
        assert asg.share[1] - asg.share[0] == pytest.approx(width_m)
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
    result = allocate([1, 3], starts(3), set(), previous=base)
    assert result[1] is not base[1] and result[3] is not base[3]


@pytest.mark.parametrize("zone", [None, CORNER_ZONE])
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


def test_whole_plan_blacklisted_means_hold():
    positions = starts(3)
    base = allocate([1, 2, 3], positions, set())
    blacklist = {(2, slot_key(p)) for p in base[2].slots}
    asg = allocate([1, 2, 3], positions, blacklist)[2]
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


@pytest.mark.parametrize("zone, zone_agent, uncovered", [
    (MIDDLE_ZONE, 2, [[107.5, 192.5, 0.0, 16.6], [107.5, 192.5, 283.4, 300.0]]),
    (CORNER_ZONE, 1, [[0.0, 85.0, 163.4, 300.0]]),
])
def test_priority_zone(at_alt, zone, zone_agent, uncovered):
    at_alt(50)
    result = allocate([1, 2, 3], starts(3), set(), priority_zone=zone)
    assert_tiles_area([asg.share for asg in result.values()])
    asg = result[zone_agent]
    lane_x, y_south, y_north = lane_of(asg)
    half_swath, half_along = allocator.SWATH_M / 2, allocator.ALONG_M / 2
    assert lane_x - half_swath <= zone[0] and zone[2] <= lane_x + half_swath
    assert y_south - half_along <= zone[1] and zone[3] <= y_north + half_along
    for y in range(int(zone[1]), int(zone[3]) + 1):
        assert revisit_s(asg, y) <= config.PRIORITY_REVISIT_S
    assert asg.uncovered == uncovered
    for a in {1, 2, 3} - {zone_agent}:
        assert result[a].uncovered == []


def test_priority_zone_sides():
    middle = allocate([1, 2, 3], starts(3), set(), priority_zone=MIDDLE_ZONE)
    assert middle[1].share[1] == middle[2].share[0] and middle[2].share[1] == middle[3].share[0]
    corner = allocate([1, 2, 3], starts(3), set(), priority_zone=CORNER_ZONE)
    assert corner[1].share[0] == 0.0 and corner[2].share[0] == corner[1].share[1]


def test_priority_zone_survives_agent_loss():
    before = allocate([1, 2, 3], starts(3), set(), priority_zone=MIDDLE_ZONE)
    after = allocate([2, 3], starts(3), set(), previous=before, priority_zone=MIDDLE_ZONE)
    assert lane_of(after[2]) == lane_of(before[2])       # the zone agent keeps its lane
    assert_tiles_area([asg.share for asg in after.values()])
    assert after[3].share == [192.5, 300.0, 0.0, 300.0]
    assert after[2].uncovered[0] == [0.0, 107.5, 0.0, 300.0]  # one agent cannot fly both sides
