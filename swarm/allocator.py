"""Coverage allocator: which agent flies which slots. Pure and deterministic: same inputs, same
output (no randomness, no I/O, sorted ids, ties broken by id). Rules and reasons:
docs/allocation.md.
"""
import dataclasses
import functools
import itertools
import math
from dataclasses import dataclass

from swarm import config, sensor

# Camera footprint at the cruise altitude in whole metres (85 x 48 at 50 m). Read at call time,
# so a test can patch them to plan for another altitude.
SWATH_M, ALONG_M = sensor.lane_footprint(config.ALT_CRUISE_M)

# Legs are cut 1 m below SLOT_SPACING_M so rounding slots to 1 cm can never break the bound.
# 79 m at 11.5 m/s cruise is 6.9 s. From a standing start the vehicle needs 5.2-5.4 s to
# reach 95 % of cruise (measured, WP_SPD 11.5, PSC_NE_JERK 4: results/exp/step1), so at most
# about 2.7 s more: under 10 s against the 15 s SLOT_TIMEOUT_S, a third of it spare.
LEG_M = config.SLOT_SPACING_M - 1.0

# Mission B. TURN_S: time lost in a 180 degree turn at a lane end. Measured at WP_SPD 11.5,
# PSC_NE_JERK 4: 5.9-6.0 s for an instant reversal at full speed, about 3.6 s for the
# CoverageController's slowed lane-end reversal (results/exp/step1), so 6 s still bounds it.
# A point on an out-and-back lane is revisited after 2 x (distance to the far end) / speed +
# TURN_S, so lane ends at most REACH_M from the zone centre keep the zone centre's
# revisit under PRIORITY_REVISIT_S - REVISIT_MARGIN_S. The margin pays for zone points up to
# 30 m off centre (the footprint's 24 m half-length absorbs all but 6 m = 0.5 s each way) and
# leaves about 1 s for flying below cruise speed near the ends.
TURN_S = 6.0
REVISIT_MARGIN_S = 2.0
REACH_M = (config.PRIORITY_REVISIT_S - REVISIT_MARGIN_S - TURN_S) * config.WP_SPD_MS / 2

# Loop variants (west lane first?, first lane northbound?), tried nearest entry first.
VARIANTS = [(True, True), (True, False), (False, True), (False, False)]


@dataclass
class Assignment:
    share: list           # [xmin, xmax, ymin, ymax]
    slots: list           # [[x, y], ...]
    loop_from: int        # after the last slot, continue from slots[loop_from]
    uncovered: list       # rectangles of the share the camera never sweeps (Mission B only)
    team: int = 1         # agents flying this loop (> 1: a shared tour, see _tour_plans)
    t_ref: float = None   # shared tour only: wall time the schedule counts from
    due_s: list = None    # ... loop slot i is due at t_ref + due_s[i] + lap x period_s
    period_s: float = None

    def to_dict(self) -> dict:
        """The ASSIGN body and the log record (uncovered is logged separately)."""
        body = {"share": self.share, "slots": self.slots, "loop_from": self.loop_from}
        if self.due_s is not None:
            body.update(t_ref=self.t_ref, due_s=self.due_s, period_s=self.period_s)
        return body


@dataclass
class _Strip:
    """What one agent must cover, before slots are planned."""
    share: list
    lane_xs: list
    y_south: float        # lane ends
    y_north: float
    uncovered: list


def slot_key(xy) -> tuple:
    """A slot's identity on the blacklist: its position rounded to whole metres."""
    return (round(xy[0]), round(xy[1]))


def _point(x, y):
    return [round(x, 2), round(y, 2)]


def _edges(x_west, x_east, n):
    """n equal strips [xmin, xmax] from x_west to x_east, west to east, tiling exactly."""
    cuts = [x_west + i * (x_east - x_west) / n for i in range(n)] + [x_east]
    return [[cuts[i], cuts[i + 1]] for i in range(n)]


def _end_inset():
    """How far lane ends lie inside the area's south and north edges. The agent counts a slot as
    reached up to SLOT_REACH_M before it and turns there, so half the footprint length minus
    that: the camera still reaches the edge (it missed the outermost 2-3 m in a Gazebo run)."""
    return ALONG_M / 2 - config.SLOT_REACH_M


def _strip(x_west, x_east):
    """A full-height strip: lanes one swath apart, ends inset by _end_inset()."""
    x0, x1, y0, y1 = config.AREA
    n_lanes = math.ceil((x_east - x_west) / SWATH_M - 1e-9)   # float noise must not add a lane
    width = (x_east - x_west) / n_lanes
    lane_xs = [x_west + (i + 0.5) * width for i in range(n_lanes)]
    return _Strip([x_west, x_east, y0, y1], lane_xs, y0 + _end_inset(), y1 - _end_inset(), [])


def _leg(a, b, bad_keys):
    """Evenly spaced points strictly between a and b, no gap longer than LEG_M.

    If one would be blacklisted, cut the leg into one more piece and try again: n and n + 1
    pieces share no inner point, so the blacklisted point is gone.
    """
    pieces = max(1, math.ceil(math.dist(a, b) / LEG_M))
    while True:
        points = [_point(a[0] + (b[0] - a[0]) * k / pieces, a[1] + (b[1] - a[1]) * k / pieces)
                  for k in range(1, pieces)]
        if not any(slot_key(p) in bad_keys for p in points):
            return points
        pieces += 1


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
    if not kept:   # a short zone lane has 3 distinct points: hold until a pair expires
        return [_point(xy[0], xy[1])], 0
    loop = _close(kept, bad)
    transit = _leg(xy, loop[0], bad)
    return transit + loop, len(transit)


def _even_split(order):
    """Mission A: equal full-height strips, handed out west to east."""
    edges = _edges(config.AREA[0], config.AREA[1], len(order))
    return {a: _strip(x0, x1) for a, (x0, x1) in zip(order, edges)}


def _side_counts(n_others, west_m, east_m):
    """How many of the other agents go west and east of the zone strip (by width)."""
    if west_m < SWATH_M / 2:
        return 0, n_others
    if east_m < SWATH_M / 2:
        return n_others, 0
    n_west = round(n_others * west_m / (west_m + east_m))
    if n_others >= 2:
        n_west = min(max(n_west, 1), n_others - 1)   # both sides are wide enough: one each
    return n_west, n_others - n_west


def _uncovered(share, sweep):
    """The parts of `share` outside the camera's `sweep` rectangle."""
    sx0, sx1, sy0, sy1 = share
    wx0, wx1, wy0, wy1 = sweep
    rects = [[sx0, wx0, sy0, sy1], [wx1, sx1, sy0, sy1], [wx0, wx1, sy0, wy0], [wx0, wx1, wy1, sy1]]
    return [[round(v, 2) for v in r] for r in rects if r[1] - r[0] > 0.01 and r[3] - r[2] > 0.01]


def _zone_split(order, xy, zone):
    """Mission B: the agent nearest the zone flies one short lane through it; the others split
    the rest west and east. Sides nobody gets are absorbed into the zone agent's share."""
    x0, x1, y0, y1 = config.AREA
    zx, zy = (zone[0] + zone[2]) / 2, (zone[1] + zone[3]) / 2
    zone_agent = min(order, key=lambda a: (math.dist(xy[a], (zx, zy)), a))
    half = SWATH_M / 2
    lane_x = min(max(zx, x0 + half), x1 - half)          # lane footprint stays inside the area
    y_south = max(y0 + _end_inset(), zy - REACH_M)
    y_north = min(y1 - _end_inset(), zy + REACH_M)
    others = [a for a in order if a != zone_agent]
    n_west, n_east = _side_counts(len(others), lane_x - half - x0, x1 - lane_x - half)
    share = [x0 if n_west == 0 else lane_x - half, x1 if n_east == 0 else lane_x + half, y0, y1]
    turn = config.SLOT_REACH_M - ALONG_M / 2           # the agent turns this far before an end
    sweep = [lane_x - half, lane_x + half, y_south + turn, y_north - turn]
    strips = {zone_agent: _Strip(share, [lane_x], y_south, y_north, _uncovered(share, sweep))}
    for a, (sx0, sx1) in zip(others[:n_west], _edges(x0, lane_x - half, n_west)):
        strips[a] = _strip(sx0, sx1)
    for a, (sx0, sx1) in zip(others[n_west:], _edges(lane_x + half, x1, n_east)):
        strips[a] = _strip(sx0, sx1)
    return strips


# ---- Mission A, layout "lanes" (config.LAYOUT): lanes fixed over the whole area --------------
# The lane set is computed once for the whole area (4 lanes at 50 m) and never moves. With N
# agents alive the allocator takes whichever gives the shorter worst revisit period: one shared
# closed tour over all lanes with the agents evenly spaced in time along it (period / N), or
# contiguous lane groups with one loop per agent (the longest loop). A loss only changes who
# flies which lanes. Periods use the measured cost of the CoverageController's corners
# (Gazebo, WP_SPD 11.5, PSC_NE_JERK 4: results/exp/step1).
TURN90_S = 2.45           # time lost per 90 degree corner of a lane loop
TURN180_S = 3.65          # time lost per lane-end reversal (a one-lane out-and-back)
# Even spacing on the tour: every tour slot has a due time on a shared clock. The schedule
# gives each corner TOUR_SLACK_S more than it costs, so an agent on time arrives at a corner
# that much early and holds there (it is slow at corners anyway); one that fell behind holds
# less and catches up by TOUR_SLACK_S per corner. Nobody can be told to fly faster.
TOUR_SLACK_S = 0.5
TOUR_CLEAR_M = 50.0        # agents on the tour never closer than this on schedule: lanes 75 m
                          # apart pass without the safety filter acting (docs/allocation.md)
JOIN_START_S = 3.0        # transit estimate: time to turn and reach cruise from a hover
# No two agents' simultaneously active slots (transit points and tour entries) may be closer
# than this: both must be within SLOT_REACH_M of their slot while D_MIN_M apart.
ACTIVE_GAP_M = config.D_MIN_M + 2 * config.SLOT_REACH_M


def area_lanes() -> list:
    """x of every lane of the whole area: ceil(width / swath), evenly spread."""
    return _strip(config.AREA[0], config.AREA[1]).lane_xs


def _corner_cost_s(a, b, c) -> float:
    """Time lost turning at b from a->b to b->c (CoverageController flies gentle turns at
    full speed: coverage.TURN_PASS_DEG = 30)."""
    u, w = (b[0] - a[0], b[1] - a[1]), (c[0] - b[0], c[1] - b[1])
    nu, nw = math.hypot(*u), math.hypot(*w)
    if nu < 1e-6 or nw < 1e-6:
        return 0.0
    cos = (u[0] * w[0] + u[1] * w[1]) / (nu * nw)
    if cos >= math.cos(math.radians(30.0)):
        return 0.0
    return TURN180_S if cos < math.cos(math.radians(135.0)) else TURN90_S


def _corner_costs(loop) -> list:
    n = len(loop)
    return [_corner_cost_s(loop[i - 1], loop[i], loop[(i + 1) % n]) for i in range(n)]


def loop_period_s(loop) -> float:
    """Seconds to fly a closed loop once at cruise, corners included."""
    n = len(loop)
    legs_m = sum(math.dist(loop[i], loop[(i + 1) % n]) for i in range(n))
    return legs_m / config.WP_SPD_MS + sum(_corner_costs(loop))


def tour_schedule(loop):
    """(due_s, period_s): when an agent that starts at loop[0] at time 0 is due at each slot.

    A corner is due half way through its own cost (it slows down before, speeds up after),
    plus the slack of every corner up to and including it.
    """
    costs = [c + TOUR_SLACK_S if c > 0 else 0.0 for c in _corner_costs(loop)]
    due, t = [], 0.0
    for i, p in enumerate(loop):
        due.append(t + costs[i] / 2)
        t += costs[i] + math.dist(p, loop[(i + 1) % len(loop)]) / config.WP_SPD_MS
    return [d - due[0] for d in due], t


def _lanes_strip(lane_xs):
    """A strip holding exactly these lanes (share = the area between their footprints)."""
    x0, x1, y0, y1 = config.AREA
    west = max(x0, min(lane_xs) - SWATH_M / 2)
    east = min(x1, max(lane_xs) + SWATH_M / 2)
    return _Strip([round(west, 2), round(east, 2), y0, y1], list(lane_xs),
                  y0 + _end_inset(), y1 - _end_inset(), [])


def tour_loop() -> list:
    """One closed tour over every lane: the boustrophedon plus its closing leg."""
    return _loop(_lanes_strip(area_lanes()), True, True)


def _contiguous_cuts(n_items, n_groups):
    """Every way to cut range(n_items) into n_groups non-empty contiguous runs, in order."""
    for cuts in itertools.combinations(range(1, n_items), n_groups - 1):
        bounds = (0,) + cuts + (n_items,)
        yield [list(range(bounds[i], bounds[i + 1])) for i in range(n_groups)]


def _group_loop(lane_xs) -> list:
    return _loop(_lanes_strip(lane_xs), True, True)


def best_split(n):
    """(worst loop period, [lane groups west to east]) of the best contiguous cut, or
    (inf, None) with more agents than lanes. Ties: the first cut in itertools order."""
    lanes = area_lanes()
    if n > len(lanes):
        return math.inf, None
    best = (math.inf, None)
    for cut in _contiguous_cuts(len(lanes), n):
        groups = [[lanes[i] for i in g] for g in cut]
        worst = max(loop_period_s(_group_loop(g)) for g in groups)
        if worst < best[0] - 1e-6:
            best = (worst, groups)
    return best


def _tour_point(loop, due, period, t):
    """Where an agent on schedule is t seconds after it was due at loop[0] (corners as points)."""
    t %= period
    i = max(k for k in range(len(loop)) if due[k] <= t)
    t1 = due[i + 1] if i + 1 < len(loop) else period
    a, b = loop[i], loop[(i + 1) % len(loop)]
    f = (t - due[i]) / (t1 - due[i])
    return a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f


@functools.lru_cache(maxsize=None)
def tour_clearance_m(n) -> float:
    """Closest approach of n agents evenly spaced on the tour, on schedule (0.5 s steps).
    The closing leg runs back along the lane ends, so some n meet head-on there (4 lanes:
    n = 2 and 4)."""
    loop = tour_loop()
    due, period = tour_schedule(loop)
    best = math.inf
    for k in range(int(period / 0.5)):
        pts = [_tour_point(loop, due, period, k * 0.5 + r * period / n) for r in range(n)]
        best = min([best] + [math.dist(p, q) for p, q in itertools.combinations(pts, 2)])
    return best


def use_tour(n) -> bool:
    """The shared tour if its revisit period (period / n) is no longer than the split's and
    agents on it stay TOUR_CLEAR_M apart; always when there are more agents than lanes."""
    split_s = best_split(n)[0]
    if split_s == math.inf:
        return True
    return (tour_schedule(tour_loop())[1] / n <= split_s
            and (n == 1 or tour_clearance_m(n) >= TOUR_CLEAR_M))


def _split_plans(order, xy, blacklist, previous, reverse):
    """Contiguous lane groups; agent-to-group by the smallest total distance to the group's
    loop (ties: the first permutation of the agents sorted by id). reverse (swap test hook):
    the groups go to the agents in the opposite order, so they must cross."""
    groups = best_split(len(order))[1]
    strips = [_lanes_strip(g) for g in groups]
    loops = [_loop(st, True, True) for st in strips]

    def cost(perm):
        return sum(min(math.dist(xy[a], p) for p in loop) for a, loop in zip(perm, loops))
    perm = min(itertools.permutations(sorted(order)), key=lambda pm: round(cost(pm), 3))
    if reverse:
        perm = perm[::-1]
    result = {}
    for a, strip in zip(perm, strips):
        old = (previous or {}).get(a)
        if (old is not None and old.team == 1 and old.share == strip.share
                and not any((a, slot_key(p)) in blacklist for p in old.slots)):
            result[a] = old
            continue
        slots, loop_from = _plan(a, xy[a], strip, blacklist)
        result[a] = Assignment(strip.share, slots, loop_from, [])
    return result



def _own_tour(a, loop, due, blacklist):
    """(points, due) of the tour for agent a: the shared tour without a's blacklisted points.
    Dropping a corner shortens the loop; the agent then arrives early and holds, so it keeps
    the shared period and its spacing."""
    bad = {key for b, key in blacklist if b == a}
    if not any(slot_key(p) in bad for p in loop):
        return loop, due
    kept = [i for i, p in enumerate(loop) if slot_key(p) not in bad]
    if not kept:
        return [], []
    own = _close([loop[i] for i in kept], bad)
    own_due, _ = tour_schedule(own)
    return own, [due[kept[0]] + d for d in own_due]


def _tour_assignment(xy, loop, due, period, t_zero, now, entry, team, fresh):
    """Join `loop` at index `entry` with phase origin t_zero: slot i is due at
    t_zero + due[i] + k period. Fresh joins take the first due time the agent can make;
    re-entries the nearest one (a late agent then holds less and catches up)."""
    n = len(loop)
    arrive_s = now + math.dist(xy, loop[entry]) / config.WP_SPD_MS + JOIN_START_S
    laps = (arrive_s - t_zero - due[entry]) / period
    first = due[entry] + period * (math.ceil(laps) if fresh else round(laps))
    due_s = [round(first + (due[(entry + k) % n] - due[entry]) % period, 2) for k in range(n)]
    transit = _leg(xy, loop[entry], set())
    x0, x1, y0, y1 = config.AREA
    return Assignment([x0, x1, y0, y1], transit + loop[entry:] + loop[:entry], len(transit),
                      [], team, round(t_zero, 3), due_s, round(period, 2))


def _entry_wait(xy, loop, due, period, t_zero, now):
    """(seconds until on schedule, entry index) of the best fresh entry."""
    best = None
    for i, p in enumerate(loop):
        travel = math.dist(xy, p) / config.WP_SPD_MS + JOIN_START_S
        t = travel + (t_zero + due[i] - now - travel) % period
        if best is None or t < best[0] - 1e-6:
            best = (t, i)
    return best


def _spaced(plans) -> bool:
    """No two agents' transit points and entries closer than ACTIVE_GAP_M."""
    active = [[tuple(p) for p in asg.slots[:asg.loop_from + 1]] for asg in plans]
    return all(math.dist(p, q) >= ACTIVE_GAP_M for i, j in itertools.combinations(
        range(len(active)), 2) for p in active[i] for q in active[j])


def _tour_plans(order, xy, blacklist, previous, now, reverse):
    """Every agent on the shared tour, evenly spaced in time.

    Same team as before: agents keep their plan; one with a newly blacklisted point re-enters
    its own tour with its old phase. Otherwise the phases are new: rank r of the agents gets
    the phase origin now + offset + r x period / m. The offset (1 s steps) and the ranking
    minimise the time until the last agent is on schedule, among plans whose transits and
    entries respect ACTIVE_GAP_M. reverse (swap test hook): every agent takes the next
    agent's phase, so they must cross.
    """
    loop = tour_loop()
    due, period = tour_schedule(loop)
    m, prev = len(order), previous or {}
    own = {a: _own_tour(a, loop, due, blacklist) for a in order}
    if not reverse and all(prev.get(a) is not None and prev[a].team == m for a in order):
        result = {}
        for a in order:
            if not any((a, slot_key(p)) in blacklist for p in prev[a].slots):
                result[a] = prev[a]
                continue
            pts, pdue = own[a]
            if not pts:
                result[a] = Assignment(prev[a].share, [_point(*xy[a])], 0, [])
                continue
            t_zero = prev[a].t_ref

            def lateness(i):
                arrive = now + math.dist(xy[a], pts[i]) / config.WP_SPD_MS + JOIN_START_S
                return abs((arrive - t_zero - pdue[i] + period / 2) % period - period / 2)
            entry = min(range(len(pts)), key=lambda i: (round(lateness(i), 3), i))
            result[a] = _tour_assignment(xy[a], pts, pdue, period, t_zero, now, entry, m, False)
        return result
    ids = sorted(order)
    candidates = []
    for offset in range(int(period / m) + 1):
        for k, perm in enumerate(itertools.permutations(ids)):
            zeros = {a: now + offset + r * period / m for r, a in enumerate(perm)}
            entries = {a: _entry_wait(xy[a], own[a][0], own[a][1], period, zeros[a], now)
                       for a in ids if own[a][0]}
            worst = max((t for t, _ in entries.values()), default=0.0)
            candidates.append((round(worst, 3), offset, k, zeros, entries))
    candidates.sort(key=lambda c: c[:3])
    for _, _, _, zeros, entries in candidates:
        plans = {a: _tour_assignment(xy[a], own[a][0], own[a][1], period, zeros[a], now,
                                     entries[a][1], m, True) for a in entries}
        if _spaced(list(plans.values())):
            break
    if reverse and len(plans) == m:              # take the next agent's entry and phase
        swapped = {}
        for a, b in zip(ids, ids[1:] + ids[:1]):
            entry = plans[b].slots[plans[b].loop_from]
            i = next(i for i, p in enumerate(own[a][0]) if p == entry)
            swapped[a] = _tour_assignment(xy[a], own[a][0], own[a][1], period,
                                          plans[b].t_ref, now, i, m, True)
        plans = swapped
    for a in ids:
        if a not in plans:                       # every tour point blacklisted: hold
            plans[a] = Assignment(list(config.AREA), [_point(*xy[a])], 0, [])
    return plans


def _first_slots(asg, xy) -> list:
    """The slots an agent flies to first: its transit and the slot it starts on (a scheduled
    plan starts at slot 0; otherwise CoverageController starts at the nearest slot)."""
    if asg.due_s is not None:
        return asg.slots[:asg.loop_from + 1]
    start = min(range(len(asg.slots)), key=lambda i: (math.dist(xy, asg.slots[i]), i))
    return asg.slots[:asg.loop_from] + [asg.slots[start]]


def _gap_m(asg_a, xy_a, asg_b, xy_b) -> float:
    return min(math.dist(p, q) for p in _first_slots(asg_a, xy_a)
               for q in _first_slots(asg_b, xy_b))


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


def _respace_transits(result, previous, xy, bad_pairs):
    """Fix the swap-run stall: two agents' active slots (transit points and the slot they start
    on) must be ACTIVE_GAP_M apart. For each clashing pair (by id), the higher id's transit
    keeps its leg but its points move off the other agent's active slots (_clear_leg)."""
    changed = sorted(a for a in result if result[a] is not (previous or {}).get(a))
    for pair in itertools.combinations(changed, 2):
        for keep, move in (pair, pair[::-1]):        # the higher id moves first
            if _gap_m(result[keep], xy[keep], result[move], xy[move]) >= ACTIVE_GAP_M:
                break
            asg = result[move]
            if asg.loop_from == 0:
                continue
            transit = _clear_leg(xy[move], asg.slots[asg.loop_from],
                                 _first_slots(result[keep], xy[keep]))
            if transit is not None and not any((move, slot_key(p)) in bad_pairs
                                               for p in transit):
                result[move] = dataclasses.replace(
                    asg, slots=transit + asg.slots[asg.loop_from:], loop_from=len(transit))
    return result

def allocate(agent_ids, positions: dict, blacklist: set, previous: dict = None,
             priority_zone: tuple = None, reverse: bool = False, layout: str = None,
             now: float = 0.0) -> dict:
    """{agent_id: Assignment} for agent_ids.

    positions: {id: (x, y, ...)} for every id. blacklist: (agent_id, slot_key) pairs that must
    not be issued. previous: the last result; an agent whose share is unchanged and whose old
    plan has no blacklisted point keeps its old Assignment object (no churn).
    priority_zone: (xmin, ymin, xmax, ymax) for Mission B, else None.
    reverse: hand the strips out east to west instead (only the ground's "swap" test hook).
    layout: Mission A layout, "strips" (equal strips per agent) or "lanes" (fixed lanes over
    the whole area, shared tour or lane groups); default config.LAYOUT. Mission B always
    uses strips. now: wall time of the plan (the shared tour's schedule counts from it).
    """
    xy = {a: [positions[a][0], positions[a][1]] for a in agent_ids}
    order = sorted(agent_ids, key=lambda a: (xy[a][0], a))   # west to east, ties by id
    if (layout or config.LAYOUT) == "lanes" and priority_zone is None:
        if use_tour(len(order)):
            result = _tour_plans(order, xy, blacklist, previous, now, reverse)
        else:
            result = _split_plans(order, xy, blacklist, previous, reverse)
        return _respace_transits(result, previous, xy, blacklist)
    if reverse:
        order.reverse()
    if priority_zone is None:
        strips = _even_split(order)
    else:
        strips = _zone_split(order, xy, priority_zone)
    result = {}
    for a in sorted(strips):
        old = (previous or {}).get(a)
        if (old is not None and old.share == strips[a].share
                and not any((a, slot_key(p)) in blacklist for p in old.slots)):
            result[a] = old
            continue
        slots, loop_from = _plan(a, xy[a], strips[a], blacklist)
        result[a] = Assignment(strips[a].share, slots, loop_from, strips[a].uncovered)
    return _respace_transits(result, previous, xy, blacklist)
