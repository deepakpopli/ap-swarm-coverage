"""Safety filter: nominal velocity -> safe setpoint (vx, vy, z). Pure logic: no I/O, no clock.

Invariant: any two agents stay >= D_MIN_M apart in 3D, from the last STATE of each neighbour
only. Per neighbour, either the altitude intervals are D_MIN_M apart, or we stay out of its
keep-out disc (growing with the age of its sample) and close on it no faster than we can stop.
Both agents of a pair apply the same rules, each taking half.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from swarm import config

TAKEOFF_CLEAR_M = config.D_MIN_M + 1.0   # D_MIN plus the position error of both agents
SIDESTEP_BELOW_MS = 1.0   # head-on: less tangential speed than this -> also step to the right
FILTERED_MS = 0.1         # log "filtered" when the output differs from the nominal by more
OBSTACLE_ID = config.GROUND_ID   # never a neighbour's id, so free for the test obstacle


@dataclass(frozen=True)
class SafetyParams:
    """Each value and what it covers."""
    margin_m: float = 1.0       # position error of both agents (estimate and sample timing)
    a_brake_ms2: float = 3.0    # braking we count on
    t_react_s: float = 0.4      # autopilot reaction + one 50 ms tick (in the braking model)
    stop_offset_m: float = 2.0  # measured back-track after a stop <= 0.45 m
    a_max_ms2: float = 11.0     # drift envelope, not the physical peak: reported velocity lags
    dv_max_ms: float = 14.0     # more than a full stop: 12.5 missed a measured reversal
    t_gap_s: float = 1.0        # 5 STATEs lost in a row at 20 % loss: p = 3e-4
    z_track_m: float = 1.0      # measured altitude error <= 0.08 m cruising, 0.94 m after takeoff
    plan_ahead_s: float = 0.6   # design: plan as if the next 3 STATEs are lost
    clear_gap_m: float = 5.0    # design: return to cruise when every disc is this far ...
    clear_s: float = 1.0        # ... for this long

    @property
    def t_go_s(self) -> float:
        """Age by which a silent neighbour has surely started to brake (its deaf rule)."""
        return self.t_gap_s + config.RADIO_DELAY_S + config.DEAF_STOP_S + self.t_react_s

    def radius_m(self, age_s: float) -> float:
        """Keep-out radius around a neighbour's predicted position, age_s after its sample."""
        t = min(age_s, self.t_go_s)
        t_full = self.dv_max_ms / self.a_max_ms2      # time to reach the full velocity change
        if t <= t_full:
            drift = 0.5 * self.a_max_ms2 * t * t
        else:
            drift = 0.5 * self.a_max_ms2 * t_full ** 2 + self.dv_max_ms * (t - t_full)
        if age_s > self.t_go_s:                        # braked, but still sliding to a stop
            brake_m = config.V_MAX_MS ** 2 / (2.0 * self.a_brake_ms2)
            drift += min(config.V_MAX_MS * (age_s - self.t_go_s), brake_m)
        return config.D_MIN_M + self.margin_m + drift

    def v_stop_ms(self, gap_m: float) -> float:
        """Largest speed that still stops within gap_m: offset + v*t_react + v^2/2a <= gap."""
        room = gap_m - self.stop_offset_m
        if room <= 0.0:
            return 0.0
        a, tr = self.a_brake_ms2, self.t_react_s
        return a * (math.sqrt(tr * tr + 2.0 * room / a) - tr)


@dataclass
class Neighbour:
    pos: Tuple[float, float, float]
    vel: Tuple[float, float, float]
    t_pos: float            # wall time its position was sampled
    failsafe: bool          # its autopilot left GUIDED: its altitude is unknown
    size_m: float = 0.0     # added to the keep-out radius (the static obstacle's radius)


@dataclass
class Disc:
    n: Tuple[float, float]  # unit vector from us to the disc centre
    gap_m: float            # horizontal distance to the disc's edge (< 0: we are inside)
    limit_ms: float         # our share of the closing speed that can still stop in time


@dataclass
class SafeCommand:
    vx: float
    vy: float
    z: float
    holding: bool
    flags: List[str] = field(default_factory=list)   # filtered, brake, deaf
    events: List[dict] = field(default_factory=list)
    nbr_dist: Optional[float] = None   # 3D distance to the nearest neighbour as last reported
    nbr_age: Optional[float] = None


def interval(z: float, targets, z_track_m: float) -> Tuple[float, float]:
    """Altitudes an agent can be at: within z_track_m of a target, or between z and them."""
    return min(z, min(targets) - z_track_m), max(z, max(targets) + z_track_m)


def neighbour_interval(nid: int, nb: Neighbour, z_track_m: float) -> Tuple[float, float]:
    """A neighbour only ever targets cruise or its own hold altitude. In failsafe it lands (every
    failsafe action is Land, mav_vehicle), so it only goes down: below its last height. The test
    obstacle has no known height."""
    if nb.failsafe:
        return -math.inf, (math.inf if nid == OBSTACLE_ID else nb.pos[2] + z_track_m)
    return interval(nb.pos[2], (config.ALT_CRUISE_M, config.hold_alt(nid)), z_track_m)


def separated(a: Tuple[float, float], b: Tuple[float, float]) -> bool:
    return a[0] - b[1] >= config.D_MIN_M or b[0] - a[1] >= config.D_MIN_M


def clear_for_takeoff(own_pos, neighbours: Dict[int, Neighbour]) -> bool:
    """Climbing straight up is safe when nobody is horizontally close."""
    return all(math.hypot(nb.pos[0] - own_pos[0], nb.pos[1] - own_pos[1]) >= TAKEOFF_CLEAR_M
               for nb in neighbours.values())


class SafetyFilter:
    """One per agent. Feed it heard() and update_neighbour(); call step() every control tick."""

    def __init__(self, agent_id: int, params: SafetyParams = SafetyParams()):
        self.p = params
        self.hold_z = config.hold_alt(agent_id)
        self.nbrs: Dict[int, Neighbour] = {}
        self.holding = True                 # take-off ends at the hold altitude
        self.deaf = False
        self.last_heard: Optional[float] = None
        self.clear_since: Optional[float] = None
        self.obstacle: Optional[Neighbour] = None

    def add_obstacle(self, x: float, y: float, radius_m: float) -> None:
        """Test hook (scenario "obstacle"): a static disc of unknown height, the allocator
        unaware; the filter treats it as a hovering failsafe drone that is always fresh."""
        self.obstacle = Neighbour((x, y, 0.0), (0.0, 0.0, 0.0), 0.0, True, radius_m)

    def heard(self, now: float) -> None:
        self.last_heard = now

    def update_neighbour(self, nid: int, pos, vel, t_pos: float, status: str) -> None:
        old = self.nbrs.get(nid)
        if old is None or t_pos > old.t_pos:       # ignore late, out-of-order states
            self.nbrs[nid] = Neighbour(tuple(pos), tuple(vel), t_pos, status == "failsafe")

    def is_deaf(self, now: float) -> bool:
        return self.last_heard is not None and now - self.last_heard > config.DEAF_STOP_S

    # ---- one control tick -------------------------------------------------------------------
    def step(self, now: float, pos, vel, v_nom: Tuple[float, float],
             want_hold: bool) -> SafeCommand:
        """pos, vel: own state from the autopilot. v_nom: what coverage asks for (m/s)."""
        if self.obstacle is not None:
            self.obstacle.t_pos = now
            self.nbrs[OBSTACLE_ID] = self.obstacle
        events = self._forget_lost(now)
        if self._update_deaf(now, events):
            return self._output(0.0, 0.0, ["deaf"], events, now, pos)
        if want_hold:
            self.holding = True
        elif self.holding and self._clear_for_cruise(now, pos):
            self.holding = False
            events.append({"event": "cruise"})
        z = self.hold_z if self.holding else config.ALT_CRUISE_M
        discs = self._discs(now, pos, interval(pos[2], (z,), self.p.z_track_m))
        vx, vy = self._limit(v_nom, discs)
        flags = []
        if math.hypot(vx - v_nom[0], vy - v_nom[1]) > FILTERED_MS:
            flags.append("filtered")
        if self._must_brake(vel, discs):
            vx, vy = 0.0, 0.0
            flags.append("brake")
        return self._output(vx, vy, flags, events, now, pos)

    def _update_deaf(self, now: float, events: List[dict]) -> bool:
        deaf = self.is_deaf(now)
        if deaf != self.deaf:
            self.deaf = deaf
            events.append({"event": "deaf_start" if deaf else "deaf_end"})
        if deaf:
            self.holding = True
            self.clear_since = None
        return deaf

    def _forget_lost(self, now: float) -> List[dict]:
        events = []
        for nid in sorted(self.nbrs):
            if now - self.nbrs[nid].t_pos > config.LOST_S:
                del self.nbrs[nid]
                events.append({"event": "neighbour_lost", "nbr": nid})
        return events

    def _discs(self, now: float, pos, own: Tuple[float, float]) -> List[Disc]:
        """Keep-out discs of the neighbours we are not vertically separated from, by id."""
        discs = []
        for nid in sorted(self.nbrs):
            nb = self.nbrs[nid]
            if separated(own, neighbour_interval(nid, nb, self.p.z_track_m)):
                continue
            age = now - nb.t_pos
            t = min(age, self.p.t_go_s)                  # it brakes by t_go_s at the latest
            dx = nb.pos[0] + nb.vel[0] * t - pos[0]
            dy = nb.pos[1] + nb.vel[1] * t - pos[1]
            dist = math.hypot(dx, dy)
            n = (dx / dist, dy / dist) if dist > 1e-6 else (1.0, 0.0)
            plan_gap = dist - self.p.radius_m(age + self.p.plan_ahead_s) - nb.size_m
            gap = dist - self.p.radius_m(age) - nb.size_m
            discs.append(Disc(n, gap, self.p.v_stop_ms(plan_gap) / 2))
        return discs

    def _limit(self, v_nom, discs: List[Disc]) -> Tuple[float, float]:
        """Cut the closing speed toward every disc to our share of what can still stop."""
        vx, vy = v_nom
        for d in discs:
            closing = vx * d.n[0] + vy * d.n[1]
            if closing <= d.limit_ms:
                continue
            excess = closing - d.limit_ms
            vx, vy = vx - excess * d.n[0], vy - excess * d.n[1]
            if math.hypot(vx - d.limit_ms * d.n[0], vy - d.limit_ms * d.n[1]) < SIDESTEP_BELOW_MS:
                vx, vy = vx + excess * d.n[1], vy - excess * d.n[0]   # head-on: step right
        scale = 1.0                         # a sidestep can break another limit: scale down
        for d in discs:
            closing = vx * d.n[0] + vy * d.n[1]
            if closing > d.limit_ms:
                scale = min(scale, d.limit_ms / closing)
        return vx * scale, vy * scale

    def _must_brake(self, vel, discs: List[Disc]) -> bool:
        """The vehicle, not just the command, must be able to stop before every disc."""
        return any(vel[0] * d.n[0] + vel[1] * d.n[1] > self.p.v_stop_ms(d.gap_m) / 2
                   for d in discs)

    def _clear_for_cruise(self, now: float, pos) -> bool:
        """Every neighbour on our way down (or up) to cruise is clear_gap_m outside its disc."""
        path = interval(pos[2], (config.ALT_CRUISE_M,), self.p.z_track_m)
        if any(d.gap_m < self.p.clear_gap_m for d in self._discs(now, pos, path)):
            self.clear_since = None
            return False
        if self.clear_since is None:
            self.clear_since = now
        return now - self.clear_since >= self.p.clear_s

    def _output(self, vx, vy, flags, events, now, pos) -> SafeCommand:
        speed = math.hypot(vx, vy)
        if speed > config.WP_SPD_MS:
            vx, vy = vx * config.WP_SPD_MS / speed, vy * config.WP_SPD_MS / speed
        z = self.hold_z if self.holding else config.ALT_CRUISE_M
        cmd = SafeCommand(vx, vy, z, self.holding, flags, events)
        if self.nbrs:
            nid = min(self.nbrs, key=lambda k: math.dist(pos, self.nbrs[k].pos))
            cmd.nbr_dist = math.dist(pos, self.nbrs[nid].pos)
            cmd.nbr_age = now - self.nbrs[nid].t_pos
        return cmd
