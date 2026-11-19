"""Coverage allocator: which agent flies which slots. Pure and deterministic: same inputs, same
output (no randomness, no I/O, sorted ids, ties broken by id). Rules and reasons:
docs/allocation.md.
"""
import math
from dataclasses import dataclass

from swarm import config, sensor

# Camera footprint at the cruise altitude in whole metres (85 x 48 at 50 m). Read at call time,
# so a test can patch them to plan for another altitude.
SWATH_M, ALONG_M = sensor.lane_footprint(config.ALT_CRUISE_M)

# Legs are cut 1 m below SLOT_SPACING_M so rounding slots to 1 cm can never break the bound.
# 80 m at 11.5 m/s cruise is 7.0 s. From a standing start at 2 m/s^2 or more, add at most
# 11.5 / (2 x 2) = 2.9 s: under 10 s against the 15 s SLOT_TIMEOUT_S, a third of it spare.
LEG_M = config.SLOT_SPACING_M - 1.0

# Mission B. TURN_S: time lost in a 180 degree turn at a lane end. Placeholder: to be measured
# in SITL. A point on an out-and-back lane is revisited after 2 x (distance to the far end) /
# speed + TURN_S, so lane ends at most REACH_M from the zone centre keep the zone centre's
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

    def to_dict(self) -> dict:
        """The ASSIGN body and the log record (uncovered is logged separately)."""
        return {"share": self.share, "slots": self.slots, "loop_from": self.loop_from}


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


def _strip(x_west, x_east):
    """A full-height strip: lanes one swath apart, ends inset by half the footprint length."""
    x0, x1, y0, y1 = config.AREA
    n_lanes = math.ceil((x_east - x_west) / SWATH_M - 1e-9)   # float noise must not add a lane
    width = (x_east - x_west) / n_lanes
    lane_xs = [x_west + (i + 0.5) * width for i in range(n_lanes)]
    return _Strip([x_west, x_east, y0, y1], lane_xs, y0 + ALONG_M / 2, y1 - ALONG_M / 2, [])


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
    y_south = max(y0 + ALONG_M / 2, zy - REACH_M)
    y_north = min(y1 - ALONG_M / 2, zy + REACH_M)
    others = [a for a in order if a != zone_agent]
    n_west, n_east = _side_counts(len(others), lane_x - half - x0, x1 - lane_x - half)
    share = [x0 if n_west == 0 else lane_x - half, x1 if n_east == 0 else lane_x + half, y0, y1]
    sweep = [lane_x - half, lane_x + half, y_south - ALONG_M / 2, y_north + ALONG_M / 2]
    strips = {zone_agent: _Strip(share, [lane_x], y_south, y_north, _uncovered(share, sweep))}
    for a, (sx0, sx1) in zip(others[:n_west], _edges(x0, lane_x - half, n_west)):
        strips[a] = _strip(sx0, sx1)
    for a, (sx0, sx1) in zip(others[n_west:], _edges(lane_x + half, x1, n_east)):
        strips[a] = _strip(sx0, sx1)
    return strips


def allocate(agent_ids, positions: dict, blacklist: set, previous: dict = None,
             priority_zone: tuple = None) -> dict:
    """{agent_id: Assignment} for agent_ids.

    positions: {id: (x, y, ...)} for every id. blacklist: (agent_id, slot_key) pairs that must
    not be issued. previous: the last result; an agent whose share is unchanged and whose old
    plan has no blacklisted point keeps its old Assignment object (no churn).
    priority_zone: (xmin, ymin, xmax, ymax) for Mission B, else None.
    """
    xy = {a: [positions[a][0], positions[a][1]] for a in agent_ids}
    order = sorted(agent_ids, key=lambda a: (xy[a][0], a))   # west to east, ties by id
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
    return result
