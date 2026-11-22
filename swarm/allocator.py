"""Coverage allocator: which agent flies which slots. Pure and deterministic: same inputs, same
output (no randomness, no I/O, sorted ids, ties broken by id). Rules and reasons:
docs/allocation.md.

Layout: lanes fixed over the whole area, cut into contiguous lane groups, one closed loop per
agent. A membership change only changes who flies which lanes; the lanes never move.
"""
import dataclasses
import itertools
import math
from dataclasses import dataclass

from swarm import config, sensor
from swarm.coverage import TURN_PASS_DEG, start_slot

# Camera footprint at the cruise altitude in whole metres (85 x 48 at 50 m). Read at call time,
# so a test can patch them to plan for another altitude.
SWATH_M, ALONG_M = sensor.lane_footprint(config.ALT_CRUISE_M)

# Legs are cut 1 m below SLOT_SPACING_M so rounding slots to 0.1 m can never break the bound.
# 79 m at 11.5 m/s cruise is 6.9 s. From a standing start the vehicle needs 5.2-5.4 s to
# reach 95 % of cruise (measured, WP_SPD 11.5, PSC_NE_JERK 4: results/exp/step1), so at most
# about 2.7 s more: under 10 s against the 15 s SLOT_TIMEOUT_S, a third of it spare.
LEG_M = config.SLOT_SPACING_M - 1.0

# Time lost per corner of a loop, measured for the CoverageController's corners (Gazebo,
# WP_SPD 11.5, PSC_NE_JERK 4: results/exp/step1). Used only to compare loop periods.
TURN90_S = 2.45           # a 90 degree corner
TURN180_S = 3.65          # a lane-end reversal (one-lane out-and-back)

# Mission B. TURN_S: time lost in a 180 degree turn at a lane end, an upper bound (5.9-6.0 s
# measured for an instant reversal at full speed, results/exp/step1). A point on an
# out-and-back lane is revisited after 2 x (distance to the far end) / speed + TURN_S, so lane
# ends at most REACH_M from the zone centre keep the zone centre's revisit under
# PRIORITY_REVISIT_S - REVISIT_MARGIN_S. The margin pays for zone points up to 30 m off centre
# (the footprint's 24 m half-length absorbs all but 6 m = 0.5 s each way) and leaves about 1 s
# for flying below cruise speed near the ends.
TURN_S = 6.0
REVISIT_MARGIN_S = 2.0
REACH_M = (config.PRIORITY_REVISIT_S - REVISIT_MARGIN_S - TURN_S) * config.WP_SPD_MS / 2

# No two agents' simultaneously active slots (transit points and the slot each starts on) may
# be closer than this: both must be within SLOT_REACH_M of their slot while D_MIN_M apart.
ACTIVE_GAP_M = config.D_MIN_M + 2 * config.SLOT_REACH_M

# Loop variants (west lane first?, first lane northbound?), tried nearest entry first.
VARIANTS = [(True, True), (True, False), (False, True), (False, False)]


@dataclass
class Assignment:
    share: list           # [xmin, xmax, ymin, ymax]
    slots: list           # [[x, y], ...]; empty for a spare agent (more agents than lanes)
    loop_from: int        # after the last slot, continue from slots[loop_from]

    def to_dict(self) -> dict:
        """The ASSIGN body and the log record."""
        return dataclasses.asdict(self)


@dataclass
class _Strip:
    """What one agent must cover, before slots are planned."""
    share: list
    lane_xs: list
    y_south: float        # lane ends
    y_north: float


def slot_key(xy) -> tuple:
    """A slot's identity on the blacklist: its position rounded to whole metres."""
    return (round(xy[0]), round(xy[1]))


def _point(x, y):
    """Slots on a 0.1 m grid: SWARM_ASSIGN carries them as int16 decimetres, losslessly."""
    return [round(x, 1), round(y, 1)]


def _end_inset():
    """How far lane ends lie inside the area's south and north edges. The agent counts a slot as
    reached up to SLOT_REACH_M before it and turns there, so half the footprint length minus
    that: the camera still reaches the edge (it missed the outermost 2-3 m in a Gazebo run)."""
    return ALONG_M / 2 - config.SLOT_REACH_M


def area_lanes() -> list:
    """x of every lane of the whole area: ceil(width / swath) lanes, evenly spread."""
    x0, x1 = config.AREA[0], config.AREA[1]
    n_lanes = math.ceil((x1 - x0) / SWATH_M - 1e-9)   # float noise must not add a lane
    return [x0 + (i + 0.5) * (x1 - x0) / n_lanes for i in range(n_lanes)]


def _lanes_strip(lane_xs):
    """A full-height strip holding exactly these lanes (share = their footprints in the area)."""
    x0, x1, y0, y1 = config.AREA
    west = max(x0, min(lane_xs) - SWATH_M / 2)
    east = min(x1, max(lane_xs) + SWATH_M / 2)
    return _Strip([round(west, 2), round(east, 2), y0, y1], list(lane_xs),
                  y0 + _end_inset(), y1 - _end_inset())


# ---- slots ------------------------------------------------------------------------------------

def _even(a, b, pieces):
    return [_point(a[0] + (b[0] - a[0]) * k / pieces, a[1] + (b[1] - a[1]) * k / pieces)
            for k in range(1, pieces)]


def _leg(a, b, bad_keys):
    """Evenly spaced points strictly between a and b, no gap longer than LEG_M.

    A blacklisted point: try n + 1 .. 2n pieces (n and n + 1 share no inner point). If every
    one hits the blacklist, the n points without the blacklisted ones: a gap may then exceed
    LEG_M, but the search ends and the slot count stays bounded.
    """
    n = max(1, math.ceil(math.dist(a, b) / LEG_M))
    for pieces in range(n, 2 * n + 1):
        points = _even(a, b, pieces)
        if not any(slot_key(p) in bad_keys for p in points):
            return points
    return [p for p in _even(a, b, n) if slot_key(p) not in bad_keys]


def _close(points, bad_keys):
    """The loop through points with leg points inserted, including the leg back to the start."""
    loop = []
    for i, p in enumerate(points):
        loop.append(p)
        loop += _leg(p, points[(i + 1) % len(points)], bad_keys)
    return loop


def _loop(strip, west_first, northbound):
    """Boustrophedon over the strip's lanes, closed back to its start.

    With one lane it is an out-and-back, so the same points appear twice; nothing relies on
    slots being unique.
    """
    ends = []
    for x in strip.lane_xs if west_first else strip.lane_xs[::-1]:
        ys = [strip.y_south, strip.y_north] if northbound else [strip.y_north, strip.y_south]
        ends += [_point(x, ys[0]), _point(x, ys[1])]
        northbound = not northbound
    return _close(ends, set())


def _plan(agent_id, xy, strip, blacklist):
    """(slots, loop_from): transit from xy to the nearest lane end, then the loop."""
    bad = {key for a, key in blacklist if a == agent_id}
    loops = [_loop(strip, west, north) for west, north in VARIANTS]
    loops.sort(key=lambda loop: math.dist(xy, loop[0]))   # stable sort: ties keep VARIANTS order
    for loop in loops:   # (1) the first variant without a blacklisted point
        transit = _leg(xy, loop[0], set())
        if not any(slot_key(p) in bad for p in transit + loop):
            return transit + loop, len(transit)
    # (2) drop the blacklisted points from the nearest variant and re-space its legs
    kept = [p for p in loops[0] if slot_key(p) not in bad]
    if not kept:   # a zone lane has 3 distinct points: hold until a pair expires
        return [_point(xy[0], xy[1])], 0
    loop = _close(kept, bad)
    transit = _leg(xy, loop[0], bad)
    return transit + loop, len(transit)


def _keep_or_plan(agent_id, xy, strip, blacklist, previous):
    """The agent's previous Assignment if it has the same share and no blacklisted point (no
    churn), else a new plan for the strip."""
    old = (previous or {}).get(agent_id)
    if (old is not None and old.share == strip.share
            and not any((agent_id, slot_key(p)) in blacklist for p in old.slots)):
        return old
    slots, loop_from = _plan(agent_id, xy, strip, blacklist)
    return Assignment(strip.share, slots, loop_from)


# ---- lane groups -------------------------------------------------------------------------------

def _corner_cost_s(a, b, c) -> float:
    """Time lost turning at b from a->b to b->c."""
    u, w = (b[0] - a[0], b[1] - a[1]), (c[0] - b[0], c[1] - b[1])
    nu, nw = math.hypot(*u), math.hypot(*w)
    if nu < 1e-6 or nw < 1e-6:
        return 0.0
    cos = (u[0] * w[0] + u[1] * w[1]) / (nu * nw)
    if cos >= math.cos(math.radians(TURN_PASS_DEG)):
        return 0.0
    return TURN180_S if cos < math.cos(math.radians(135.0)) else TURN90_S


def loop_period_s(loop) -> float:
    """Seconds to fly a closed loop once at cruise, corners included."""
    n = len(loop)
    legs_m = sum(math.dist(loop[i], loop[(i + 1) % n]) for i in range(n))
    corners_s = sum(_corner_cost_s(loop[i - 1], loop[i], loop[(i + 1) % n]) for i in range(n))
    return legs_m / config.WP_SPD_MS + corners_s


def _contiguous_cuts(n_items, n_groups):
    """Every way to cut range(n_items) into n_groups non-empty contiguous runs, in order."""
    for cuts in itertools.combinations(range(1, n_items), n_groups - 1):
        bounds = (0,) + cuts + (n_items,)
        yield [list(range(bounds[i], bounds[i + 1])) for i in range(n_groups)]


def best_split(n):
    """[lane x groups, west to east] of the contiguous cut into n groups whose longest loop is
    shortest. Ties: the first cut in itertools order. n must not exceed the number of lanes."""
    lanes = area_lanes()
    best_s, best = math.inf, None
    for cut in _contiguous_cuts(len(lanes), n):
        groups = [[lanes[i] for i in g] for g in cut]
        worst_s = max(loop_period_s(_loop(_lanes_strip(g), True, True)) for g in groups)
        if worst_s < best_s - 1e-6:
            best_s, best = worst_s, groups
    return best


def _group_plans(ids, xy, blacklist, previous, reverse):
    """Contiguous lane groups; agent-to-group by the smallest total distance to the group's
    loop (ties: the first permutation of the ids in ascending order). reverse (swap test hook):
    the groups go to the agents in the opposite order, so they must cross.
    More agents than lanes: one lane each; the agents left over get no slots and hold at their
    private hold altitude, out of the lanes' way, until a loss frees a lane."""
    n_flying = min(len(ids), len(area_lanes()))
    strips = [_lanes_strip(g) for g in best_split(n_flying)]
    loops = [_loop(strip, True, True) for strip in strips]

    def cost(perm):
        return sum(min(math.dist(xy[a], p) for p in loop) for a, loop in zip(perm, loops))
    perm = min(itertools.permutations(sorted(ids), n_flying), key=lambda p: round(cost(p), 3))
    if reverse:
        perm = perm[::-1]
    result = {a: Assignment(list(config.AREA), [], 0) for a in ids if a not in perm}
    for a, strip in zip(perm, strips):
        result[a] = _keep_or_plan(a, xy[a], strip, blacklist, previous)
    return result


# ---- Mission B: one agent owns an out-and-back lane through the priority zone ----------------

def _zone_lane_x(zone):
    """x of the zone lane: half way between two area lanes (or a lane and the area edge), so it
    keeps clear of the other agents' lanes; of those, the one whose footprint covers the most of
    the zone's width, then the one nearest the zone centre."""
    edges = [config.AREA[0]] + area_lanes() + [config.AREA[1]]
    middles = [(a + b) / 2 for a, b in zip(edges, edges[1:])]
    zone_x = (zone[0] + zone[2]) / 2

    def covered_m(x):
        return min(zone[2], x + SWATH_M / 2) - max(zone[0], x - SWATH_M / 2)
    return max(middles, key=lambda x: (round(covered_m(x), 3), -abs(x - zone_x)))


def _zone_strip(zone):
    """One lane through the zone, its ends at most REACH_M north and south of the zone centre."""
    x0, x1, y0, y1 = config.AREA
    lane_x = _zone_lane_x(zone)
    zone_y = (zone[1] + zone[3]) / 2
    share = [round(max(x0, lane_x - SWATH_M / 2), 2), round(min(x1, lane_x + SWATH_M / 2), 2),
             y0, y1]
    return _Strip(share, [lane_x], max(y0 + _end_inset(), zone_y - REACH_M),
                  min(y1 - _end_inset(), zone_y + REACH_M))


def _zone_owner(ids, xy, zone, strip, previous):
    """The agent that flew the zone lane before, while it lives; else the one nearest the zone
    centre (ties: lowest id)."""
    for a in ids:
        old = (previous or {}).get(a)
        if old is not None and old.share == strip.share:
            return a
    centre = ((zone[0] + zone[2]) / 2, (zone[1] + zone[3]) / 2)
    return min(ids, key=lambda a: math.dist(xy[a], centre))


# ---- active slots apart --------------------------------------------------------------------------

def _first_slots(asg, motion) -> list:
    """The slots an agent flies to first: its transit and the slot the CoverageController will
    start on. motion: (xy, velocity, slot it heads to or None)."""
    if not asg.slots:                    # a spare agent: it flies nowhere
        return []
    return asg.slots[:asg.loop_from] + [asg.slots[start_slot(asg.slots, asg.loop_from, *motion)]]


def _gap_m(slots_a, slots_b) -> float:
    return min((math.dist(p, q) for p in slots_a for q in slots_b), default=math.inf)


def _clear_leg(a, b, avoid):
    """Points strictly between a and b, gaps at most LEG_M, none closer than ACTIVE_GAP_M to
    a point of `avoid`: walk from a, a full LEG_M step each time, shortened (1 m at a time)
    while it lands too close. None if no step fits."""
    length = math.dist(a, b)
    points, done = [], 0.0

    def at(d):
        return _point(a[0] + (b[0] - a[0]) * d / length, a[1] + (b[1] - a[1]) * d / length)
    while length - done > LEG_M:
        step = LEG_M
        while step > 1.0 and any(math.dist(at(done + step), q) < ACTIVE_GAP_M for q in avoid):
            step -= 1.0
        if step <= 1.0:
            return None
        done += step
        points.append(at(done))
    return points


def _respace_transits(result, previous, motion, bad_pairs):
    """Two agents' active slots (transit points and the slot they start on) must be
    ACTIVE_GAP_M apart (the swap run once issued two transit slots 0.2 m apart and stalled).
    Each new plan that clashes, highest id first, keeps its transit leg but moves the points
    off every other agent's active slots (_clear_leg). A kept plan is already being flown."""
    for move in sorted(result, reverse=True):
        asg = result[move]
        others = [p for b in result if b != move for p in _first_slots(result[b], motion[b])]
        if (asg is (previous or {}).get(move) or asg.loop_from == 0
                or _gap_m(_first_slots(asg, motion[move]), others) >= ACTIVE_GAP_M):
            continue
        transit = _clear_leg(motion[move][0], asg.slots[asg.loop_from], others)
        if transit is not None and not any((move, slot_key(p)) in bad_pairs for p in transit):
            result[move] = dataclasses.replace(
                asg, slots=transit + asg.slots[asg.loop_from:], loop_from=len(transit))
    return result


def allocate(agent_ids, positions: dict, blacklist: set, previous: dict = None,
             priority_zone: tuple = None, reverse: bool = False, velocities: dict = None,
             heading: dict = None) -> dict:
    """{agent_id: Assignment} for agent_ids.

    positions: {id: (x, y, ...)} for every id. blacklist: (agent_id, slot_key) pairs that must
    not be issued. previous: the last result; an agent whose share is unchanged and whose old
    plan has no blacklisted point keeps its old Assignment object (no churn).
    priority_zone: (xmin, ymin, xmax, ymax) for Mission B, else None. Its owner flies the zone
    lane; the other agents share the whole area in lane groups, as if they were the fleet.
    reverse: hand the groups out in reverse order (only the ground's "swap" test hook).
    velocities {id: (vx, vy)} and heading {id: slot xy} (missing: standing, unknown) predict
    where each agent's CoverageController starts (coverage.start_slot).
    """
    xy = {a: [positions[a][0], positions[a][1]] for a in agent_ids}
    ids = sorted(agent_ids)
    result = {}
    if priority_zone is not None:
        strip = _zone_strip(priority_zone)
        owner = _zone_owner(ids, xy, priority_zone, strip, previous)
        result[owner] = _keep_or_plan(owner, xy[owner], strip, blacklist, previous)
        ids.remove(owner)
    if ids:
        result.update(_group_plans(ids, xy, blacklist, previous, reverse))
    motion = {a: (xy[a], (velocities or {}).get(a, (0.0, 0.0)), (heading or {}).get(a))
              for a in xy}
    return _respace_transits(dict(sorted(result.items())), previous, motion, blacklist)
