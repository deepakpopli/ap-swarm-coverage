"""Deterministic coverage allocator. Pure functions, no I/O, no randomness.

    allocate(alive_ids, positions, blacklist, params) -> {agent_id: Assignment}

Same inputs => identical output: ids are sorted, ties broken by id, nothing depends on dict
or set iteration order, coordinates are rounded to 1 cm.

1. Shares. AREA is cut into len(alive) vertical strips (full height). Widths are equal by
   default, or proportional to `params.weights` (one positive number per strip, left to right).
   Strips are handed out left-to-right to agents sorted by current x (ties by id), so
   neighbours stay neighbours and paths do not cross.
   Mission B (priority zone) would use weights: give each strip an equal *workload*, where
   ground inside the priority zone counts k times; the ground computes the weights for the
   current number of agents and passes them in. The allocator itself stays the same.

2. Slots. An ordered list of goal points: TRANSIT points from the agent's last known position to
   the entry of its lawnmower, then the repeating boustrophedon LOOP (loop_from = index of the
   first loop slot; after the last slot the agent goes back to slots[loop_from]).
   Lanes run north-south: n_lanes = ceil(strip_width / swath_across), one lane centred in each of
   n_lanes equal sub-strips; lane ends inset by footprint_along / 2 from the south/north edges.
   Consecutive slots (and the agent's position -> first slot, and last slot -> slots[loop_from])
   are at most max_slot_spacing apart; longer legs get evenly spaced intermediate points.
   The default entry is the strip corner nearest to the agent.

3. Infeasible (agent, slot) pairs. `blacklist` is a set of (agent_id, slot_key) where
   slot_key(p) = (round(x), round(y)) in whole metres. A blacklisted pair is never issued.
   For an agent whose plan contains a blacklisted point, the rule is, in this order:
     (1) Variants: try the other loop variants (4 = entry lane west/east x first lane
         northbound/southbound, ordered by distance from the agent to the entry). The first
         variant with no blacklisted point wins. This changes entry point and transit route.
     (2) Nudge/skip, per variant in the same order: a blacklisted TRANSIT point is dropped; a
         blacklisted LOOP point is moved nudge_step metres toward the share centre (repeated
         while still blacklisted, up to max_nudges). Legs that become too long get intermediate
         points again; an intermediate point that is blacklisted is moved sideways by nudge_step
         (left, right, 2x left, 2x right). The first variant that ends clean wins.
     (3) Swap strips with the right neighbour, else the left one; accepted only if both agents
         get a clean plan in the swapped strips (each via steps 1-2).
     (4) Last resort (should not happen in practice): blacklisted points are simply removed;
         the spacing bound may then be violated. Assignment.note says so.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from swarm import config, sensor

# Sensor footprint at config.ALT_NOMINAL (SIYI A8 mini, swarm.sensor), whole metres rounded down:
# 102 x 57 at the default 60 m, 85 x 48 at 50 m. Defaults only: AllocParams can override them.
DEFAULT_SWATH_ACROSS_M, DEFAULT_FOOTPRINT_ALONG_M = sensor.lane_footprint(config.ALT_NOMINAL)
DEFAULT_MAX_SLOT_SPACING_M = 90.0

_SPACING_MARGIN_M = 0.5     # legs are cut at spacing - margin, so 1 cm rounding never breaks the bound
_EPS = 1e-9


@dataclass(frozen=True)
class AllocParams:
    area: Tuple[float, float, float, float] = config.AREA
    swath_across: float = DEFAULT_SWATH_ACROSS_M
    footprint_along: float = DEFAULT_FOOTPRINT_ALONG_M
    max_slot_spacing: float = DEFAULT_MAX_SLOT_SPACING_M
    weights: Optional[Tuple[float, ...]] = None    # per strip, left to right; None = equal
    nudge_step: float = 10.0
    max_nudges: int = 3
    reach_m: float = config.SLOT_REACH_M           # closer than this to the entry -> no transit


@dataclass
class Assignment:
    share: List[float]                 # [xmin, xmax, ymin, ymax]
    slots: List[List[float]]           # [[x, y], ...]
    loop_from: int
    note: str = ""                     # "" normally; explains steps 1-4 if any were used

    def to_dict(self):
        return {"share": list(self.share), "slots": [list(p) for p in self.slots],
                "loop_from": self.loop_from}


# ------------------------------------------------------------------ small geometry helpers

def slot_key(p):
    """Geometric identity of a slot: position rounded to whole metres (round half up)."""
    return (int(math.floor(p[0] + 0.5)), int(math.floor(p[1] + 0.5)))


def _r(v):
    return round(v + 0.0, 2) + 0.0     # 1 cm, and no -0.0


def _pt(x, y):
    return [_r(x), _r(y)]


def _dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _leg_points(a, b, spacing):
    """Evenly spaced points strictly between a and b so that no gap exceeds `spacing`."""
    d = _dist(a, b)
    n = max(1, math.ceil(d / spacing - _EPS))
    return [_pt(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n)]


# ------------------------------------------------------------------ shares

def compute_strips(n, area=config.AREA, weights=None):
    """n vertical strips [xmin, xmax, ymin, ymax] tiling `area` exactly, left to right."""
    xmin, xmax, ymin, ymax = area
    if n <= 0:
        return []
    if weights is None:
        weights = [1.0] * n
    if len(weights) != n or any(w <= 0 for w in weights):
        raise ValueError("weights must be n positive numbers")
    total = float(sum(weights))
    cuts = [xmin]
    acc = 0.0
    for w in weights[:-1]:
        acc += w
        cuts.append(xmin + (xmax - xmin) * acc / total)
    cuts.append(xmax)          # exact right edge, no float drift
    return [[cuts[i], cuts[i + 1], ymin, ymax] for i in range(n)]


def order_agents(alive_ids, positions):
    """Agents sorted west to east by current x, ties by id; unknown positions last (by id)."""
    def key(a):
        p = positions.get(a)
        return (0, p[0], a) if p is not None else (1, 0.0, a)
    return sorted(alive_ids, key=key)


# ------------------------------------------------------------------ lawnmower loop

def lane_xs(share, swath_across):
    x0, x1 = share[0], share[1]
    n = max(1, math.ceil((x1 - x0) / swath_across - _EPS))
    w = (x1 - x0) / n
    return [x0 + (i + 0.5) * w for i in range(n)]


def lane_ys(share, footprint_along):
    y0, y1 = share[2] + footprint_along / 2.0, share[3] - footprint_along / 2.0
    if y1 < y0:                    # strip shorter than one footprint: a single row in the middle
        y0 = y1 = (share[2] + share[3]) / 2.0
    return y0, y1


def loop_corners(share, params, east_first, south_first):
    """Lane end points in flying order for one variant (before densification)."""
    xs = lane_xs(share, params.swath_across)
    if east_first:
        xs = xs[::-1]
    y_s, y_n = lane_ys(share, params.footprint_along)
    pts = []
    northbound = south_first
    for x in xs:
        a, b = (y_s, y_n) if northbound else (y_n, y_s)
        pts.append(_pt(x, a))
        if b != a:
            pts.append(_pt(x, b))
        northbound = not northbound
    return pts


def densify_loop(corners, spacing):
    """Closed loop through `corners` with intermediate points, including the closing leg."""
    out = []
    n = len(corners)
    for i in range(n):
        out.append(corners[i])
        if n > 1:
            out.extend(_leg_points(corners[i], corners[(i + 1) % n], spacing))
    return out


def variants(share, pos, params):
    """The 4 loop variants, nearest entry first (ties in fixed order). Each: (name, loop)."""
    spacing = params.max_slot_spacing - _SPACING_MARGIN_M
    out = []
    for i, (east_first, south_first) in enumerate([(False, True), (False, False),
                                                    (True, True), (True, False)]):
        loop = densify_loop(loop_corners(share, params, east_first, south_first), spacing)
        name = ("E" if east_first else "W") + ("N" if south_first else "S")
        d = _dist(pos, loop[0]) if pos is not None else 0.0
        out.append((round(d, 6), i, name, loop))
    out.sort(key=lambda v: (v[0], v[1]))
    return [(name, loop) for _, _, name, loop in out]


def transit_points(pos, entry, params):
    """Points from pos (exclusive) to entry (exclusive)."""
    if pos is None or _dist(pos, entry) <= params.reach_m:
        return []
    return _leg_points(pos, entry, params.max_slot_spacing - _SPACING_MARGIN_M)


# ------------------------------------------------------------------ feasibility repair

def _is_bad(agent, p, blacklist):
    return (agent, slot_key(p)) in blacklist


def _toward(p, target, step):
    dx, dy = target[0] - p[0], target[1] - p[1]
    d = math.hypot(dx, dy)
    if d < _EPS:
        return _pt(p[0] + step, p[1])
    return _pt(p[0] + dx / d * step, p[1] + dy / d * step)


def _sideways_candidates(a, b, p, step):
    """p moved left/right of the a->b direction: left, right, 2x left, 2x right."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = math.hypot(dx, dy) or 1.0
    lx, ly = -dy / d, dx / d
    return [_pt(p[0] + lx * k * step, p[1] + ly * k * step) for k in (1, -1, 2, -2)]


def _fill_leg(agent, a, b, spacing, step, blacklist):
    """Intermediate points a->b, each blacklisted one moved sideways if possible."""
    out = []
    for p in _leg_points(a, b, spacing):
        if _is_bad(agent, p, blacklist):
            for c in _sideways_candidates(a, b, p, step):
                if not _is_bad(agent, c, blacklist):
                    p = c
                    break
        out.append(p)
    return out


def _respace(agent, pos, transit, loop, params, blacklist):
    """Re-insert intermediate points wherever a leg exceeds the spacing bound."""
    spacing = params.max_slot_spacing - _SPACING_MARGIN_M
    limit = params.max_slot_spacing
    new_transit = []
    prev = pos
    for p in transit + [loop[0]]:          # legs pos -> transit... -> loop entry
        if prev is not None and _dist(prev, p) > limit:
            new_transit.extend(_fill_leg(agent, prev, p, spacing, params.nudge_step, blacklist))
        new_transit.append(p)
        prev = p
    new_transit.pop()                      # that last point was the loop entry itself
    new_loop = []
    n = len(loop)
    for i in range(n):
        new_loop.append(loop[i])
        nxt = loop[(i + 1) % n]
        if n > 1 and _dist(loop[i], nxt) > limit:
            new_loop.extend(_fill_leg(agent, loop[i], nxt, spacing, params.nudge_step, blacklist))
    return new_transit, new_loop


def _repair(agent, share, pos, transit, loop, params, blacklist):
    """Step 2 on one variant. Returns (transit, loop) with no blacklisted point, or None."""
    centre = [(share[0] + share[1]) / 2.0, (share[2] + share[3]) / 2.0]
    for _ in range(params.max_nudges + 1):
        transit = [p for p in transit if not _is_bad(agent, p, blacklist)]
        loop = [_toward(p, centre, params.nudge_step) if _is_bad(agent, p, blacklist) else p
                for p in loop]
        transit, loop = _respace(agent, pos, transit, loop, params, blacklist)
        if not any(_is_bad(agent, p, blacklist) for p in transit + loop):
            return transit, loop
    return None


def plan_agent(agent, share, pos, blacklist, params):
    """Steps 1-2 for one agent in one share. Returns an Assignment or None if no clean plan."""
    cands = []
    for name, loop in variants(share, pos, params):
        transit = transit_points(pos, loop[0], params)
        cands.append((name, transit, loop))
        if not any(_is_bad(agent, p, blacklist) for p in transit + loop):
            note = "" if len(cands) == 1 else "variant %s" % name   # first variant = default
            return Assignment(list(share), transit + loop, len(transit), note)
    for name, transit, loop in cands:
        fixed = _repair(agent, share, pos, transit, loop, params, blacklist)
        if fixed is not None:
            t, lp = fixed
            return Assignment(list(share), t + lp, len(t), "variant %s + nudge" % name)
    return None


def _degraded(agent, share, pos, blacklist, params):
    """Step 4: drop every blacklisted point outright (spacing bound may break)."""
    name, loop = variants(share, pos, params)[0]
    transit = [p for p in transit_points(pos, loop[0], params) if not _is_bad(agent, p, blacklist)]
    loop = [p for p in loop if not _is_bad(agent, p, blacklist)]
    note = "degraded: blacklisted points removed, spacing not guaranteed"
    if not loop:      # nothing left in the share: hold at the last known position if allowed
        transit = []
        loop = [_pt(*pos)] if pos is not None and not _is_bad(agent, pos, blacklist) else []
        note = "degraded: no feasible slot in share, holding"
    return Assignment(list(share), transit + loop, len(transit) if loop else 0, note)


# ------------------------------------------------------------------ entry point

def allocate(alive_ids, positions, blacklist=frozenset(), params=AllocParams()):
    """{agent_id: Assignment} for the live agents. See module docstring.

    alive_ids: iterable of agent ids. positions: {id: (x, y[, z])} last known (may miss ids).
    blacklist: set of (agent_id, slot_key) pairs that must not be issued.
    """
    alive = sorted(set(alive_ids))
    pos = {a: [float(positions[a][0]), float(positions[a][1])] for a in alive if a in positions}
    order = order_agents(alive, pos)
    strips = compute_strips(len(order), params.area, params.weights)
    share_of = {a: strips[i] for i, a in enumerate(order)}
    bl = set(blacklist)
    plans = {a: plan_agent(a, share_of[a], pos.get(a), bl, params) for a in order}

    # step 3: swap with a neighbour (right first, then left)
    for i, a in enumerate(order):
        if plans[a] is not None:
            continue
        for j in (i + 1, i - 1):
            if not 0 <= j < len(order):
                continue
            b = order[j]
            pa = plan_agent(a, share_of[b], pos.get(a), bl, params)
            pb = plan_agent(b, share_of[a], pos.get(b), bl, params)
            if pa is not None and pb is not None:
                share_of[a], share_of[b] = share_of[b], share_of[a]
                pa.note = ("swapped strips with %d; " % b) + pa.note
                pb.note = ("swapped strips with %d; " % a) + pb.note
                plans[a], plans[b] = pa, pb
                break

    for a in order:   # step 4
        if plans[a] is None:
            plans[a] = _degraded(a, share_of[a], pos.get(a), bl, params)
    return {a: plans[a] for a in alive}
