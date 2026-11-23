"""Coverage allocator: which agent flies which slots. Pure and deterministic (sorted ids, ties
by id). Fixed lanes over the whole area, cut into contiguous groups, one closed loop per agent:
a membership change moves owners, never lanes.
"""
import dataclasses
import itertools
import math
from dataclasses import dataclass

from swarm import config, sensor
from swarm.coverage import A_SLOW_MS2, TURN_PASS_DEG, V_TURN_MIN_MS, start_slot

# Camera footprint at cruise in whole metres; read at call time so a test can patch them.
SWATH_M, ALONG_M = sensor.lane_footprint(config.ALT_CRUISE_M)

# 1 m below SLOT_SPACING_M so 0.1 m rounding cannot break it; under 10 s from a standing start
# against the 15 s SLOT_TIMEOUT_S.
LEG_M = config.SLOT_SPACING_M - 1.0

# Measured corner costs, only to compare loop periods.
TURN90_S = 2.45
TURN180_S = 3.65          # a lane-end reversal (one-lane out-and-back)

# Mission B: zone lane ends at most REACH_M from the zone centre keep its revisit under
# PRIORITY_REVISIT_S; TURN_S bounds a full-speed reversal.
TURN_S = 6.0
REVISIT_MARGIN_S = 2.0
REACH_M = (config.PRIORITY_REVISIT_S - REVISIT_MARGIN_S - TURN_S) * config.WP_SPD_MS / 2

# Mission B with two agents (the butterfly): each centre lane this far from the zone centre;
# the half-lap phase may be PHASE_BUDGET_S off; an agent can join its loop every ENTRY_STEP_M,
# so the phase can be chosen to PHASE_STEP_S. A 7 s budget: lanes 171 m, a 54 s lap (5 s gives
# 194 m and 58 s, but the zone then went over 30 s in a third of the fake-backend runs).
BUTTERFLY_OFFSET_M = config.D_MIN_M
PHASE_BUDGET_S = 7.0
PHASE_STEP_S = 1.0
ENTRY_STEP_M = PHASE_STEP_S * config.WP_SPD_MS
ACCEL_MS2 = 2.6           # measured ArduCopter speed-up: 0 -> 95 % of 10 m/s in 3.5-4.2 s
# A plan reaches its agent this long after the state it was planned from (half a state period
# old, two radio delays); LATE_S if one ASSIGN is lost.
ON_TIME_S = 2 * config.RADIO_DELAY_S + 0.5 / config.STATE_HZ
LATE_S = ON_TIME_S + 1 / config.ASSIGN_HZ

# Simultaneously active slots this far apart: both agents within SLOT_REACH_M while D_MIN_M apart.
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
    """Lane ends inside the south and north edges: the agent turns SLOT_REACH_M early, and the
    camera must still reach the edge."""
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
    """Boustrophedon over the strip's lanes, closed back to its start. One lane: an out-and-back,
    so points repeat; nothing relies on slots being unique."""
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


def best_split(n, lanes=None):
    """[lane x groups, west to east] of the contiguous cut of lanes (default: the area's) into n
    groups whose longest loop is shortest. Ties: the first cut in itertools order. n must not
    exceed the number of lanes."""
    lanes = lanes or area_lanes()
    best_s, best = math.inf, None
    for cut in _contiguous_cuts(len(lanes), n):
        groups = [[lanes[i] for i in g] for g in cut]
        worst_s = max(loop_period_s(_loop(_lanes_strip(g), True, True)) for g in groups)
        if worst_s < best_s - 1e-6:
            best_s, best = worst_s, groups
    return best


def _same_owners(ids, strips, previous):
    """The previous owners of the strips, in strip order, if every strip had one among ids (an
    infeasible re-plan: no reshuffle of the others). Else None."""
    owners = {tuple(asg.share): a for a, asg in (previous or {}).items() if a in ids and asg.slots}
    perm = tuple(owners.get(tuple(strip.share)) for strip in strips)
    return None if None in perm else perm


def _group_plans(ids, xy, blacklist, previous, reverse, lanes=None):
    """Contiguous lane groups. The same strips as before keep their owners; else agent-to-group
    by the smallest total distance to the loops (ties: the first permutation of the sorted ids).
    reverse (swap test hook): the opposite order. More agents than lanes: the spares get no
    slots and hold until a loss frees a lane. lanes: Mission B's remainder lanes, else the area's."""
    lanes = lanes or area_lanes()
    n_flying = min(len(ids), len(lanes))
    strips = [_lanes_strip(g) for g in best_split(n_flying, lanes)]
    loops = [_loop(strip, True, True) for strip in strips]

    def cost(perm):
        return sum(min(math.dist(xy[a], p) for p in loop) for a, loop in zip(perm, loops))
    perm = None if reverse else _same_owners(ids, strips, previous)
    if perm is None:
        perm = min(itertools.permutations(sorted(ids), n_flying), key=lambda p: round(cost(p), 3))
    if reverse:
        perm = perm[::-1]
    result = {a: Assignment(list(config.AREA), [], 0) for a in ids if a not in perm}
    for a, strip in zip(perm, strips):
        result[a] = _keep_or_plan(a, xy[a], strip, blacklist, previous)
    return result


# ---- Mission B: one agent owns an out-and-back lane through the priority zone ----------------

def _zone_lane_x(zone):
    """x of the zone lane: a midpoint between area lanes (or a lane and the edge), clear of the
    other agents' lanes; the one covering most of the zone's width, then nearest its centre."""
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


def _remainder_lanes(zone_share):
    """x of the lanes outside the zone lane's share, west then east: ceil(width / swath) lanes
    per side, the outer ones half a swath in (fully covered, shortest loop); one lane: centred."""
    lanes = []
    for a, b in ((config.AREA[0], zone_share[0]), (zone_share[1], config.AREA[1])):
        n = math.ceil((b - a) / SWATH_M - 1e-9)
        if n == 1:
            lanes.append((a + b) / 2)
        elif n > 1:
            lanes += [a + SWATH_M / 2 + i * (b - a - SWATH_M) / (n - 1) for i in range(n)]
    return lanes


def _zone_owner(ids, xy, zone, strip, previous):
    """The agent that flew the zone lane before, while it lives; else the one nearest the zone
    centre (ties: lowest id)."""
    for a in ids:
        old = (previous or {}).get(a)
        if old is not None and old.share == strip.share:
            return a
    centre = ((zone[0] + zone[2]) / 2, (zone[1] + zone[3]) / 2)
    return min(ids, key=lambda a: math.dist(xy[a], centre))


# ---- Mission B with two agents: the butterfly --------------------------------------------------
# Each agent flies a two-lane loop: north on a centre lane through the zone, across, south on a
# wing lane one swath further out, back. Either centre lane's footprint spans the zone; both are
# flown northbound half a lap apart, so the zone is seen twice a lap.

def _butterfly_strips(zone):
    """(west, east) strips, lane_xs = [centre, wing]; lane ends centred on the zone, for a lap
    that revisits it within PRIORITY_REVISIT_S - PHASE_BUDGET_S (half a lap, less the time a
    pass sees a point). None if a footprint leaves the area or a centre lane misses the zone."""
    x0, x1, y0, y1 = config.AREA
    zone_x, zone_y = (zone[0] + zone[2]) / 2, (zone[1] + zone[3]) / 2
    lap_s = 2 * (config.PRIORITY_REVISIT_S - PHASE_BUDGET_S + ALONG_M / config.WP_SPD_MS)
    lane_m = (lap_s - 4 * TURN90_S) * config.WP_SPD_MS / 2 - SWATH_M
    y_south, y_north = zone_y - lane_m / 2, zone_y + lane_m / 2
    strips = []
    for side in (-1, 1):
        centre = zone_x + side * BUTTERFLY_OFFSET_M
        wing = centre + side * SWATH_M
        if (not x0 + SWATH_M / 2 <= wing <= x1 - SWATH_M / 2
                or centre - SWATH_M / 2 > zone[0] or centre + SWATH_M / 2 < zone[2]
                or y_south < y0 + _end_inset() or y_north > y1 - _end_inset()
                or y_south > zone[1] or y_north < zone[3]):
            return None
        strips.append(_Strip(_lanes_strip([centre, wing]).share, [centre, wing], y_south, y_north))
    return strips


def _legs(asg, start, xy, vel):
    """[(p, q, s)]: the legs flown from xy, moving at vel, at cruise: to slots[start], on through
    the transit and twice round the loop. s: when the agent leaves p, its turn there included."""
    def nxt(i):
        return i + 1 if i + 1 < len(asg.slots) else asg.loop_from
    points, i = [xy, asg.slots[start]], start
    for _ in range(max(0, asg.loop_from - start) + 2 * (len(asg.slots) - asg.loop_from)):
        i = nxt(i)
        points.append(asg.slots[i])
    legs, s = [], 0.0
    before = (xy[0] - vel[0], xy[1] - vel[1])
    for p, q in zip(points, points[1:]):
        s += _turn_s(before, p, q)
        legs.append((p, q, s))
        s += math.dist(p, q) / config.WP_SPD_MS
        before = p
    return legs


def _passes(legs, zone) -> list:
    """[(s, x, northbound)]: when the legs fly over the zone centre on a lane whose footprint
    spans the zone, straight on it since the zone's edge (a whole pass)."""
    zone_x, zone_y = (zone[0] + zone[2]) / 2, (zone[1] + zone[3]) / 2
    reach_m = SWATH_M / 2 - (zone[2] - zone[0]) / 2
    out, run = [], None              # run: (x, direction, y it began at)
    for p, q, s in legs:
        if p[0] != q[0] or p[1] == q[1] or abs(p[0] - zone_x) > reach_m:
            run = None
            continue
        d = 1 if q[1] > p[1] else -1
        if run is None or run[:2] != (p[0], d):
            run = (p[0], d, p[1])
        edge = zone[1] if d > 0 else zone[3]
        if (run[2] - edge) * d <= 0 and (p[1] - zone_y) * d <= 0 < (q[1] - zone_y) * d:
            out.append((s + abs(zone_y - p[1]) / config.WP_SPD_MS, p[0], d > 0))
    return out


def _turn_s(a, b, c) -> float:
    """Time lost turning at b from a->b to b->c: the CoverageController slows to its corner
    speed at A_SLOW_MS2, the vehicle speeds up again at ACCEL_MS2. By angle, unlike the two
    measured costs of _corner_cost_s: a phase depends on every turn of a transit."""
    u, w = (b[0] - a[0], b[1] - a[1]), (c[0] - b[0], c[1] - b[1])
    nu, nw = math.hypot(*u), math.hypot(*w)
    if nu < 1e-6 or nw < 1e-6:
        return 0.0
    cos = (u[0] * w[0] + u[1] * w[1]) / (nu * nw)
    if cos >= math.cos(math.radians(TURN_PASS_DEG)):
        return 0.0
    lost_ms = config.WP_SPD_MS - max(V_TURN_MIN_MS, config.WP_SPD_MS * cos)
    return lost_ms ** 2 / (2 * config.WP_SPD_MS) * (1 / A_SLOW_MS2 + 1 / ACCEL_MS2)


def _after(old, heading, vel):
    """The slot after `heading` in the agent's old plan (the one flown into along vel where a
    point repeats), or None."""
    if old is None or heading is None:
        return None
    n = len(old.slots)
    for i, p in enumerate(old.slots):
        before = old.slots[i - 1] if i > 0 else old.slots[n - 1]
        if (tuple(p) == tuple(heading)
                and (p[0] - before[0]) * vel[0] + (p[1] - before[1]) * vel[1] >= 0):
            return old.slots[i + 1 if i + 1 < n else old.loop_from]
    return None


def _entries(agent_id, strip, zone, motion, blacklist, previous):
    """[(Assignment, [its _passes if it gets the plan ON_TIME_S, LATE_S after the state], s until
    it is on the loop)], one per distinct outcome, for every ENTRY_STEP_M along the loop: a
    transit ends there and the loop goes on from the next slot. start_slot starts a plan on the
    slot the agent heads to, so a transit can begin there, or there and on the old plan's next
    slot (reached before the plan arrives): the only ways to join behind a loop slot ahead. The
    loop is _loop's, so corners keep their slow-down, less its blacklisted points (as _plan). The
    previous plan only, if it is this strip's and has no blacklisted point."""
    xy, vel, heading = motion
    old = (previous or {}).get(agent_id)
    after = _after(old, heading, vel)
    bad = {key for a, key in blacklist if a == agent_id}

    def heads_to(lag_s):
        """The slot the agent heads to when the plan arrives lag_s late (None: unknown)."""
        near_m = math.hypot(*vel) * lag_s + config.SLOT_REACH_M
        return heading if heading is None or math.dist(xy, heading) > near_m else after
    if old is not None and old.share == strip.share and not {slot_key(p) for p in old.slots} & bad:
        plans = [old]
    else:
        prefixes = [[]] + [[_point(*p) for p in heads] for heads in ([heading], [heading, after])
                           if None not in heads]
        loop, plans = _loop(strip, True, True), []
        if any(slot_key(p) in bad for p in loop):
            loop = _close([p for p in loop if slot_key(p) not in bad], bad)
        for j, (a, b) in enumerate(zip(loop, loop[1:] + loop[:1])):
            leg_m = math.dist(a, b)
            for k in range(math.ceil(leg_m / ENTRY_STEP_M)):
                f = k * ENTRY_STEP_M / leg_m
                entry = _point(a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)
                for prefix in prefixes:
                    transit = prefix + _leg(prefix[-1] if prefix else xy, entry, set()) + [entry]
                    slots = transit + loop[j + 1:] + loop[:j + 1]
                    if not any(slot_key(p) in bad for p in slots):
                        plans.append(Assignment(strip.share, slots, len(transit)))
    out, seen = [], set()
    for asg in plans:
        passes = []
        for lag_s in (ON_TIME_S, LATE_S):
            at = (xy[0] + vel[0] * lag_s, xy[1] + vel[1] * lag_s)
            first = start_slot(asg.slots, asg.loop_from, at, vel, heads_to(lag_s))
            legs = _legs(asg, first, at, vel)
            passes.append([(round(lag_s + s, 1), x, north) for s, x, north in _passes(legs, zone)])
        start = start_slot(asg.slots, asg.loop_from, *motion)
        on_s = round(_legs(asg, start, xy, vel)[max(0, asg.loop_from - start) + 1][2], 1)
        outcome = (str(passes), on_s)
        if outcome not in seen:
            seen.add(outcome)
            out.append((asg, passes, on_s))
    return out


def _since_seen_s(ids, zone, motion, previous) -> float:
    """Seconds since the zone centre was last flown over, from the zone lane's out-and-back (the
    agent flying it, LOST_S if it was lost; the zone owner of Mission B with three agents)."""
    strip = _zone_strip(zone)
    zone_y = (zone[1] + zone[3]) / 2
    for a in ids:
        if (previous or {}).get(a) is not None and previous[a].share == strip.share:
            (_, y), (_, vy), _ = motion[a]
            end = strip.y_north if vy < 0 else strip.y_south     # the end it came from
            if (y - zone_y) * vy >= 0:                            # past the centre
                return abs(y - zone_y) / config.WP_SPD_MS
            return (abs(zone_y - end) + abs(y - end)) / config.WP_SPD_MS + TURN180_S
    return config.LOST_S


def _worst_gap_s(west, east, centres, since_s, lap_s):
    """(longest zone gap, phase error) from two agents' _passes: the gap until the first pass
    (since_s before now), between passes, and between centre-lane passes half a lap apart."""
    last = [max((s for s, x, north in passes if x == centre and north), default=None)
            for passes, centre in zip((west, east), centres)]
    if None in last:              # blacklisted points broke a centre lane
        return math.inf, math.inf
    error_s = abs((last[0] - last[1]) % lap_s - lap_s / 2)
    horizon_s = min(last)
    times = sorted(s for s, _, _ in west + east if s <= horizon_s)
    gaps = [since_s + times[0], lap_s / 2 + error_s] + [b - a for a, b in zip(times, times[1:])]
    return max(gaps), error_s


def _lap_s(strip) -> float:
    loop = _loop(strip, True, True)
    return _legs(Assignment([], loop, 0), 0, loop[-1], (0.0, 0.0))[len(loop)][2]


def _butterfly_plans(ids, zone, motion, blacklist, previous):
    """Two agents on the butterfly, or None (it does not fit, or every entry is blacklisted).
    Sides: the previous ones, else the western agent west (ties: lower id); no crossing. An
    agent with its previous butterfly plan keeps it. Choice: no clash for _respace_transits to
    move (it would drop an entry), the shortest predicted zone gap in whole PHASE_STEP_S (each
    plan on time or late), the smallest phase error, both on their loops soonest; ties: the
    first in loop order."""
    strips = _butterfly_strips(zone)
    if strips is None:
        return None
    lap_s = _lap_s(strips[0])
    shares = [strip.share for strip in strips]
    perm = sorted(ids, key=lambda a: (motion[a][0][0], a))
    if any(old.share in shares and shares.index(old.share) != perm.index(a)
           for a, old in (previous or {}).items() if a in ids):
        perm = perm[::-1]
    sides = [_entries(a, strip, zone, motion[a], blacklist, previous)
             for a, strip in zip(perm, strips)]
    centres = [_point(strip.lane_xs[0], 0)[0] for strip in strips]
    since_s = _since_seen_s(ids, zone, motion, previous)
    best_key, best = None, None
    for (west, west_passes, west_on), (east, east_passes, east_on) in itertools.product(*sides):
        moved = [asg for a, asg in zip(perm, (west, east)) if asg is not (previous or {}).get(a)]
        clash = bool(moved) and _gap_m(_first_slots(west, motion[perm[0]]),
                                       _first_slots(east, motion[perm[1]])) < ACTIVE_GAP_M
        gaps, errors = zip(*(_worst_gap_s(w, e, centres, since_s, lap_s)
                             for w in west_passes for e in east_passes))
        if math.inf in gaps:
            continue
        key = (clash, round(max(gaps) / PHASE_STEP_S), round(max(errors), 1),
               round(max(west_on, east_on), 1))
        if best_key is None or key < best_key:
            best_key, best = key, dict(zip(perm, (west, east)))
    return best


# The half lap between the two agents is chosen once, at the re-plan, from a predicted transit;
# in Gazebo it came out up to 8 s off and stayed off (zone gaps of 20 and 36 s instead of 2 x 27).
# The phase hold: the agent whose next centre pass comes less than half a lap after the other's
# flies slower until the two are half a lap apart again.
PACE_SLOW = 0.8           # x cruise for the agent that is too close behind ...
PACE_SLOWER = 0.6         # ... while it is more than PACE_FAR_S off: 11 s back in about 45 s
PACE_FAR_S = 3.0
PACE_ON_S = 1.5           # phase error that starts the hold (the ground's states are up to
PACE_OFF_S = 0.5          # ~0.3 s old) and that ends it


def butterfly_pace(assignments, zone, motion, holding=False, settled=True) -> dict:
    """{agent: speed limit} for the phase hold, {} with nothing to correct or no butterfly.
    assignments: the current plan; motion: {id: (xy, velocity, slot it heads to)}. holding: a
    hold is on (it ends below PACE_OFF_S, not PACE_ON_S). settled: both agents are on their
    loops; before that (the change-over) the agent next over the zone is never slowed: its pass
    is the zone's next look, while slowing the other one only fixes the half lap."""
    strips = _butterfly_strips(zone) if zone is not None and len(assignments) == 2 else None
    ids = [next((a for a, asg in assignments.items() if asg.share == strip.share), None)
           for strip in strips or []]
    if not ids or None in ids:
        return {}
    upcoming = []
    for a, strip in zip(ids, strips):
        asg, (xy, vel, heading) = assignments[a], motion[a]
        legs = _legs(asg, start_slot(asg.slots, asg.loop_from, xy, vel, heading), xy, vel)
        centre = _point(strip.lane_xs[0], 0)[0]
        passes = [s for s, x, north in _passes(legs, zone) if x == centre and north]
        if not passes:
            return {}
        upcoming.append(passes[0])
    lap_s = _lap_s(strips[0])
    error_s = (upcoming[0] - upcoming[1]) % lap_s - lap_s / 2   # < 0: west too soon after east
    if abs(error_s) < (PACE_OFF_S if holding else PACE_ON_S):
        return {}
    k = 0 if error_s < 0 else 1
    if not settled and upcoming[k] <= upcoming[1 - k]:
        return {}
    factor = PACE_SLOWER if abs(error_s) > PACE_FAR_S else PACE_SLOW
    return {ids[k]: round(factor * config.WP_SPD_MS, 2)}


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
    """Points strictly between a and b, gaps at most LEG_M, none within ACTIVE_GAP_M of
    `avoid`: LEG_M steps from a, each shortened 1 m at a time while too close. None if stuck."""
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
    """Keep active slots ACTIVE_GAP_M apart (a swap run once got transit slots 0.2 m apart and
    stalled): each clashing new plan, highest id first, moves its transit points along the leg.
    A kept plan is already being flown."""
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
    lane; the other agents share the rest of the area (_remainder_lanes) in lane groups. Two
    agents fly the butterfly instead, where it fits (_butterfly_plans).
    reverse: hand the groups out in reverse order (only the ground's "swap" test hook).
    velocities {id: (vx, vy)} and heading {id: slot xy} (missing: standing, unknown) predict
    where each agent's CoverageController starts (coverage.start_slot).
    """
    xy = {a: [positions[a][0], positions[a][1]] for a in agent_ids}
    ids = sorted(agent_ids)
    motion = {a: (xy[a], (velocities or {}).get(a, (0.0, 0.0)), (heading or {}).get(a))
              for a in xy}
    result, lanes = {}, None
    if priority_zone is not None and len(ids) == 2:
        result = _butterfly_plans(ids, priority_zone, motion, blacklist, previous) or {}
        ids = [] if result else ids
    if priority_zone is not None and ids:
        strip = _zone_strip(priority_zone)
        owner = _zone_owner(ids, xy, priority_zone, strip, previous)
        result[owner] = _keep_or_plan(owner, xy[owner], strip, blacklist, previous)
        ids.remove(owner)
        lanes = _remainder_lanes(strip.share)
    if ids:
        result.update(_group_plans(ids, xy, blacklist, previous, reverse, lanes))
    return _respace_transits(dict(sorted(result.items())), previous, motion, blacklist)
