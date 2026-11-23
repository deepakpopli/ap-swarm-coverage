"""Tests for swarm/allocator.py. Pure functions: no sockets, no clock (a flight test runs the fake
vehicle on a simulated one)."""
import math
import signal

import pytest

from sim.fake_vehicle import FakeVehicle
from swarm import allocator, config, protocol, sensor
from swarm.allocator import allocate, slot_key
from swarm.coverage import CoverageController

MIDDLE_ZONE = (120.0, 120.0, 180.0, 180.0)   # xmin, ymin, xmax, ymax
CORNER_ZONE = (0.0, 0.0, 60.0, 60.0)


def starts(n):
    return {i + 1: config.start_xy(i, n) for i in range(n)}


def legs(asg, xy):
    """Every leg flown: position -> first slot, slot to slot, last slot -> slots[loop_from]."""
    points = [list(xy[:2])] + asg.slots
    return list(zip(points, points[1:])) + [(asg.slots[-1], asg.slots[asg.loop_from])]


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
                                                 (CORNER_ZONE, 1, 18.8)])
def test_priority_zone(at_alt, zone, owner, lane_x):
    """The owner's lane covers the zone within 30 s, half way between two area lanes; the
    others fly lanes outside the owner's share, and the shares together cover the area."""
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
    for a in result:
        if a != owner:
            assert all(abs(x - p[0]) >= half_swath for p in result[a].slots[result[a].loop_from:])
    shares = sorted(asg.share for asg in result.values())    # together they cover the area
    assert shares[0][0] == config.AREA[0] and max(sh[1] for sh in shares) == config.AREA[1]
    assert all(east[0] <= west[1] for west, east in zip(shares, shares[1:]))


def test_zone_owner_kept_and_replaced_on_loss():
    """Where the butterfly does not fit (a corner zone), two agents keep one zone owner."""
    before = allocate([1, 2, 3], starts(3), set(), priority_zone=CORNER_ZONE)
    after = allocate([1, 3], starts(3), set(), previous=before, priority_zone=CORNER_ZONE)
    assert after[1] is before[1]                        # the owner keeps its lane
    assert len(after[3].slots) > len(before[3].slots)   # the other flies the rest alone
    near_zone = {2: (30.0, 100.0), 3: (250.0, 0.0)}
    lost = allocate([2, 3], near_zone, set(), previous=before, priority_zone=CORNER_ZONE)
    assert lane_of(lost[2]) == lane_of(before[1])       # the nearest survivor takes the zone


# ---- Mission B with two agents: the butterfly

def on_loop(asg, i):
    """Mid-way along leg i of the agent's loop, at cruise toward its end: (xy, velocity, the slot
    it heads to)."""
    loop = asg.slots[asg.loop_from:]
    a, b = loop[i], loop[(i + 1) % len(loop)]
    d = math.dist(a, b)
    v = [(b[0] - a[0]) / d * config.WP_SPD_MS, (b[1] - a[1]) / d * config.WP_SPD_MS]
    return [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2], v, tuple(b)


def after_loss(survivors, legs):
    """The three-agent Mission B plan, then a re-plan for the survivors, each mid-way along leg
    legs[a] of its loop. Returns (before, after, motion)."""
    before = allocate([1, 2, 3], starts(3), set(), priority_zone=MIDDLE_ZONE)
    motion = {a: on_loop(before[a], legs[a]) for a in survivors}
    after = allocate(survivors, {a: m[0] for a, m in motion.items()}, set(), before,
                     MIDDLE_ZONE, velocities={a: m[1] for a, m in motion.items()},
                     heading={a: m[2] for a, m in motion.items()})
    return before, after, motion


def fly(plans, motion, before, seconds, dt=0.05, late=None, pace=False):
    """Fly each agent on its old plan, give it the new one ON_TIME_S (+ late[a]) later (fake
    vehicle and coverage controller, simulated clock); pace: the ground's phase hold, once a
    second. Returns [(t, agent)] of its centre-lane passes: northbound over the zone centre with
    a footprint spanning the zone."""
    t = [0.0]
    agents = {}
    for a, (xy, vel, heading) in motion.items():
        vehicle = FakeVehicle(a, (xy[0], xy[1], config.ALT_CRUISE_M), clock=lambda: t[0])
        vehicle.vel = [vel[0], vel[1], 0.0]
        control = CoverageController()
        control.on_assign(1, before[a].slots, before[a].loop_from, xy, vel)
        control.idx = [tuple(p) for p in before[a].slots].index(heading)
        agents[a] = (vehicle, control)
    zone_x, zone_y = 150.0, 150.0
    passes, last_y, speeds, next_pace = [], {}, {}, 0.0
    while t[0] < seconds:
        if pace and t[0] >= next_pace and all(c.epoch == 2 and c.idx >= c.loop_from
                                              for _, c in agents.values()):
            next_pace = t[0] + 1.0
            motion_now = {a: (v.state().pos[:2], v.state().vel[:2], c.slots[c.idx])
                          for a, (v, c) in agents.items()}
            speeds = allocator.butterfly_pace(plans, MIDDLE_ZONE, motion_now, bool(speeds))
            for a, (v, c) in agents.items():
                c.on_assign(2, plans[a].slots, plans[a].loop_from, v.state().pos[:2],
                            v.state().vel[:2], speeds.get(a, config.WP_SPD_MS))
        for a, (vehicle, control) in agents.items():
            st = vehicle.state()
            if t[0] >= allocator.ON_TIME_S + (late or {}).get(a, 0.0) and control.epoch == 1:
                control.on_assign(2, plans[a].slots, plans[a].loop_from, st.pos[:2], st.vel[:2])
            out = control.step(t[0], st.pos[:2], False)
            vehicle.send_setpoint(out.vx, out.vy, config.ALT_CRUISE_M)
            reach_m = allocator.SWATH_M / 2 - 30.0
            if last_y.get(a, zone_y) < zone_y <= st.pos[1] and abs(st.pos[0] - zone_x) <= reach_m:
                passes.append((t[0], a))
            last_y[a] = st.pos[1]
        t[0] += dt
        for vehicle, _ in agents.values():
            vehicle.poll()
    return passes


@pytest.mark.parametrize("survivors, legs", [([2, 3], {2: 1, 3: 2}),   # agent 1 lost
                                             ([1, 3], {1: 0, 3: 4}),   # the zone owner lost
                                             ([1, 2], {1: 6, 2: 5})])  # agent 3 lost
def test_butterfly_after_a_loss(survivors, legs):
    """Two survivors fly two-lane loops, north on a centre lane through the zone and south on a
    wing lane one swath out; the western one west; slots fit in SWARM_ASSIGN, legs bounded."""
    _, after, motion = after_loss(survivors, legs)
    west, east = sorted(survivors, key=lambda a: motion[a][0][0])
    half_swath = allocator.SWATH_M / 2
    for a, side in ((west, -1), (east, 1)):
        asg = after[a]
        loop = asg.slots[asg.loop_from:]
        centre = 150.0 + side * allocator.BUTTERFLY_OFFSET_M
        assert sorted({x for x, _ in loop}) == sorted({centre, centre + side * allocator.SWATH_M,
                                                       centre + side * half_swath})
        assert centre - half_swath <= MIDDLE_ZONE[0] and MIDDLE_ZONE[2] <= centre + half_swath
        assert all(q[1] > p[1] for p, q in zip(loop, loop[1:] + loop[:1])
                   if p[0] == q[0] == centre)                   # the centre lane runs north
        assert len(asg.slots) <= protocol.MAX_SLOTS
        points = [motion[a][0]] + asg.slots + [asg.slots[asg.loop_from]]
        assert all(math.dist(p, q) <= config.SLOT_SPACING_M for p, q in zip(points, points[1:]))


@pytest.mark.parametrize("survivors, legs", [([2, 3], {2: 1, 3: 2}), ([1, 3], {1: 0, 3: 4}),
                                             ([1, 2], {1: 6, 2: 5}), ([1, 3], {1: 2, 3: 7})])
def test_butterfly_phase(survivors, legs):
    """Flown, the two centre-lane passes alternate half a lap apart, within PHASE_BUDGET_S."""
    before, after, motion = after_loss(survivors, legs)
    passes = fly(after, motion, before, 200.0)
    steady = [p for p in passes if p[0] > 60.0]
    assert all(a != b for (_, a), (_, b) in zip(steady, steady[1:]))   # they alternate
    spacing = [b - a for (a, _), (b, _) in zip(steady, steady[1:])]
    assert max(spacing) - min(spacing) <= 2 * allocator.PHASE_BUDGET_S


@pytest.mark.parametrize("late", [{2: 8.0}, {3: 12.0}])
def test_phase_hold_brings_them_back_half_a_lap_apart(late):
    """A plan that reaches one agent late leaves the two off their half lap, and off it stays;
    with the ground's phase hold they are back to half a lap within two laps."""
    before, after, motion = after_loss([2, 3], {2: 1, 3: 2})

    def worst_error_s(passes):         # half the difference of two gaps in a row
        steady = [s for s, _ in passes if s > 150.0]
        gaps = [b - a for a, b in zip(steady, steady[1:])]
        return max(abs(b - a) / 2 for a, b in zip(gaps, gaps[1:]))
    assert worst_error_s(fly(after, motion, before, 260.0, late=late)) > 3.0
    assert worst_error_s(fly(after, motion, before, 260.0, late=late, pace=True)) < 1.5


def test_butterfly_deterministic_and_kept():
    """The same inputs give the same plans; a re-plan keeps both butterfly plans (no churn); a
    blacklisted point re-plans only that agent, on its own side."""
    _, after, motion = after_loss([1, 3], {1: 0, 3: 4})
    assert after_loss([1, 3], {1: 0, 3: 4})[1] == after
    positions = {a: m[0] for a, m in motion.items()}
    kept = allocate([1, 3], positions, set(), after, MIDDLE_ZONE)
    assert kept[1] is after[1] and kept[3] is after[3]
    bad = {(3, slot_key(after[3].slots[-1]))}
    moved = allocate([1, 3], positions, bad, after, MIDDLE_ZONE)
    assert moved[1] is after[1] and moved[3].share == after[3].share
    assert slot_key(after[3].slots[-1]) not in {slot_key(p) for p in moved[3].slots}


def test_butterfly_only_for_two_agents_in_mission_b():
    """Mission A and three-agent Mission B keep their plans; a zone the butterfly cannot fit
    (a corner) keeps one zone owner."""
    shares = {tuple(s.share) for s in allocator._butterfly_strips(MIDDLE_ZONE)}
    assert allocator._butterfly_strips(CORNER_ZONE) is None
    for ids in ([1, 2], [1, 2, 3]):
        assert not shares & {tuple(asg.share) for asg in allocate(ids, starts(3), set()).values()}
    three = allocate([1, 2, 3], starts(3), set(), priority_zone=MIDDLE_ZONE)
    assert not shares & {tuple(asg.share) for asg in three.values()}
    two = allocate([1, 2], starts(2), set(), priority_zone=MIDDLE_ZONE)
    assert {tuple(asg.share) for asg in two.values()} == shares


def test_many_blacklisted_points_on_the_zone_lane_end():
    """These 8 points block every spacing of the zone lane's 126 m middle leg: _leg once
    searched forever."""
    def hang(signum, frame):
        raise TimeoutError("allocate did not return")
    bad = {(2, (150, y)) for y in (94, 98, 129, 141, 150, 188, 204, 205)}
    old = signal.signal(signal.SIGALRM, hang)
    signal.alarm(5)
    try:
        asg = allocate([1, 2, 3], starts(3), bad, priority_zone=MIDDLE_ZONE)[2]
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, old)
    assert asg.share == allocator._zone_strip(MIDDLE_ZONE).share
    assert not {slot_key(p) for p in asg.slots} & {key for _, key in bad}
