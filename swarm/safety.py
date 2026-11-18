"""Safety filter: nominal command -> safe setpoint. Pure logic, no I/O, no clock (pass `now`).

Every setpoint the agent sends to its autopilot goes through SafetyFilter.step(), including hold,
takeoff-complete and fallback setpoints.

What is enforced
----------------
For every pair of agents: 3D Euclidean distance >= D_MIN (8 m). The filter guarantees this in one of
two ways, per neighbour:
  (a) VERTICAL: the altitude interval we can be in and the altitude interval the neighbour can be in
      are >= D_MIN apart. Then 3D distance >= vertical gap >= D_MIN, whatever the horizontal positions.
  (b) HORIZONTAL: otherwise we stay outside the neighbour's inflated keep-out disc (radius R >= D_MIN
      + margin around its predicted horizontal position), so horizontal distance >= D_MIN.

Altitude intervals (the swarm rule that makes (a) work)
--------------------------------------------------------
An agent only ever targets two altitudes: ALT_NOMINAL (60 m) in normal flight, or its own LAYER in
hold/fallback/deaf/pre-start. Layers are spaced LAYER_SPACING = 10 m inside ALT_BAND, assigned by ID:
id 1 -> 50, id 2 -> 60, id 3 -> 70, id 4 -> 50, ... (index (id-1) mod n_layers). With ALT_BAND
(50, 70) there are exactly 3 layers; for N > 3 layers are re-used, and agents sharing a layer get NO
vertical separation from each other: they rely on the horizontal rule only (both are commanded
zero horizontal speed in fallback, so they do not close, but nothing pushes them apart).
So a neighbour last seen at z0 is always within hull(z0, ALT_NOMINAL, layer_j) +- Z_TRACK (1 m
altitude tracking error); no age term is needed. Our own interval is hull(z, our target) +- Z_TRACK.
Two agents at different layers: gap = 10 - 2*1 = 8 m >= D_MIN, separated. Anyone whose interval
contains 60 is never vertically separated from an agent flying at 60.

Keep-out radius derivation (numbers with the default parameters)
----------------------------------------------------------------
Neighbour j last sent its state at t_s (message stamp "t"; we use t_ref = min(t_s, t_recv) so a
clock skew can only make us more conservative). age = now - t_ref; it includes the 200 ms radio delay.
Predicted centre: c = p0 + v0 * min(age, T_GO)  (horizontal).
  uncertainty(age): how far j can be from c.
  * Up to T_GO, j may fly anything with |v(t) - v0| <= min(A_MAX*t, DV_MAX) (A_MAX = 8 m/s^2,
    DV_MAX = 12.5 m/s, both from measurements, see SafetyParams). The deviation from the
    constant-velocity prediction is then <= deviation(age): 0.5*A_MAX*age^2 up to t1 = DV_MAX/A_MAX
    = 1.56 s, then growing by DV_MAX per second.
  * T_GO = T_GAP + DEAF_STOP_S + T_REACT = 1.0 + 3.0 + 0.4 = 4.4 s is the latest time at which a
    silent neighbour must have started to brake:
      T_GAP = 1.0 s: we stop hearing j because its radio failed (blackouts are two-way), but the last
        few of its messages before the failure may have been lost by chance. At 5 Hz and 20 % loss,
        5 consecutive losses (1 s) has probability 0.2^5 = 3e-4. So j went deaf at most ~1 s after
        its last message that reached us.
      DEAF_STOP_S = 3.0 s: j then hears nobody for 3 s and commands zero horizontal speed.
      T_REACT = 0.4 s: one 10 Hz control period + 0.2-0.3 s measured autopilot response.
    deviation(4.4) = 9.77 + 12.5 * 2.84 = 45.23 m.
  * After T_GO the centre is frozen and j can add at most its braking distance from <= V_MAX:
    V_MAX^2 / (2*A_BRAKE) = 144 / 6 = 24 m, travelled at <= V_MAX, i.e. the term grows at <= 12 m/s.
  uncertainty(age) = deviation(age)                                    for age <= T_GO
                   = 45.23 + min(V_MAX*(age - T_GO), 24.0)              for age >  T_GO
  The bound saturates at 69.23 m from age 6.4 s on.
  R(age) = D_MIN + MARGIN + uncertainty(age), MARGIN = 1.0 m (position estimate error of both agents).
    R(0.2 s) = 9.16 m, R(1 s) = 13.0 m, R(3 s) = 36.7 m, R(10 s) = 78.23 m (saturated).
  Assumptions behind the bound: neighbours obey A_MAX and the deaf rule, and a radio failure is
  two-way. A neighbour whose transmitter dies but whose receiver works keeps flying: residual risk.

Own braking and reaction (the speed-dependent part of the margin)
-----------------------------------------------------------------
We do not add our own stopping distance to R as a constant (24 m + 3.6 m would make R >= 37 m even
when stationary). Instead the allowed closing speed toward a disc depends on the gap to its edge:
  v_stop(gap) = largest v with  STOP_OFFSET + v*T_REACT + v^2/(2*A_BRAKE) <= gap
  (STOP_OFFSET = 4 m, T_REACT = 0.4 s, A_BRAKE = 3 m/s^2)
i.e. at any moment we could react and brake before the edge. At 12 m/s this reserves 4 + 4.8 + 24 =
32.8 m. STOP_OFFSET exists because MavVehicle flies a position target ("rabbit") that leads the
vehicle; on a stop command the vehicle still flies to the rabbit. Its computed lead (shaper_lag()
+ 1 m trailing, WP_ACC 6, jerk 12) is 2.9 / 6.0 / 10.6 / 14.6 / 19.2 m at 2 / 5 / 8 / 10 / 12 m/s,
plus up to LEAD_MARGIN_M = 2 m when the vehicle lags the command. Our model gives 5.5 / 10.2 /
17.9 / 24.7 / 32.8 m. Measured (results/spike/speed_cap.md): 13.9-15.0 m from 10 m/s, overshoot
past the rest point <= 1.23 m.
Responsibility is shared: each agent closes at no more than v_stop(gap)/2 toward each neighbour, so together they close at no more than v_stop(gap). The braking curve is the
"short horizon": a neighbour only constrains us once our stopping distance reaches its disc.

Planning radius
---------------
Being inside R(age) is a violation (-> fallback). The velocity filter plans against the slightly
larger R_plan = R(age + PLAN_AHEAD), PLAN_AHEAD = 0.6 s: the disc as it will be if the next three
STATE messages (5 Hz) are lost. Otherwise an agent that brakes exactly onto the edge of R would be
pushed "inside" by the first lost message (the disc grows while the info ages). Fresh (age 0.2 s):
R = 9.16 m, R_plan = 11.56 m, so the filter stops about 2.4 m outside the violation radius.

Filtering (NORMAL mode)
-----------------------
For each neighbour not vertically separated, in ascending ID order (deterministic):
  n = unit vector from us to its predicted centre, lim = v_stop(d - R_plan) / 2.
  If v.n > lim: remove the excess along n (keep the tangential component).
  Tie-break / anti-mirroring: if the remaining tangential speed is < 1 m/s (head-on), add the removed
  speed as a sidestep to OUR RIGHT of n. Both agents of a head-on pair turn right, so they diverge
  (pass left-side to left-side) instead of mirroring each other.
Three passes over the list, then a final uniform scale-down that is guaranteed to satisfy all
constraints (zero velocity always does, because lim >= 0 outside every disc).

Fallback
--------
Triggers (in NORMAL mode): we are inside R(age) of a non-vertically-separated neighbour, or "boxed
in": nominal speed >= 1 m/s, the filter is cutting the command, and our ACTUAL horizontal speed has
stayed < 0.5 m/s for 3 s (a stall with no allowed direction).
Action: mode LAYERED, raise one "safety" infeasible event. In LAYERED mode: z target = own layer;
horizontal speed is ZERO while not yet within Z_TRACK of the layer or while inside any
non-separated disc; once at the layer, the nominal velocity is allowed again through the same
filter (other layers are vertically separated, so agents at different layers can pass over each
other; this is what removes deadlock). Exit (back to ALT_NOMINAL) with hysteresis: every neighbour,
separated or not, at horizontal distance >= R + 5 m continuously for 1 s.
The filter starts in LAYERED mode (reason "start"): before START every agent holds at its own
layer; agents leave the layer once horizontally clear of everyone (start line is 10 m apart, so not
before they have spread out).

Deaf rule
---------
No message from anyone (agent or ground) for DEAF_STOP_S: mode DEAF = zero horizontal, own layer
(status "hold"). When any message arrives again: mode LAYERED (reason "deaf") and the normal exit
rule. Note: with N = 1 and a silent ground this also triggers (nobody else to hear).

Neighbour table housekeeping
----------------------------
Silent > SUSPECT_S: "suspect" event (kept, disc keeps inflating). Silent > LOST_S (12 s): dropped,
"lost" event. The mission model is that a dead drone falls; until it is dropped its disc grows to
the 69 m bound (R = 78 m) around its last predicted position. Residual risks: (1) if the companion dies but
the airframe keeps flying, MavVehicle's last position target is at most ~20 m ahead (its rabbit
lead), so the vehicle should stop there and hover silently: covered by the disc until LOST_S, then
invisible; (2) anything that keeps flying silently after 12 s is invisible to the filter.

Ground / takeoff neighbours: treated exactly like airborne ones (disc + altitude interval
hull(z0, 60, layer) which covers the climb). They do not move horizontally by rule (autopilot
takeoff holds position). The start line (10 m) is just above R(fresh) = 9.1 m but below
R_plan = 11.6 m, which is why the filter starts LAYERED: after takeoff the agents spread
vertically (50/60/70) before anyone moves horizontally, and no fallback is raised at the start.

Known weakness: the middle layer equals ALT_NOMINAL (60 m), so agent 2's fallback gives it no
vertical escape from agents flying at 60; it stops horizontally and relies on them to avoid it.

Hard output clamp: |v_xy| <= V_MAX, z inside ALT_BAND.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from swarm import config as C

NORMAL = "normal"
LAYERED = "layered"
DEAF = "deaf"


@dataclass(frozen=True)
class SafetyParams:
    """All distances in m, speeds in m/s, accelerations in m/s^2, times in s."""
    d_min: float = C.D_MIN
    margin: float = 1.0          # static: position estimate error of both agents
    v_max: float = C.V_MAX
    # Values checked against results/spike/speed_cap.md and its logs (copied to
    # results/spike/speedcap_{gazebo,sitl}_b_log.json, final MavVehicle parameters); the numbers
    # below are in results/spike/motion_bounds.json (tools/motion_bounds.py):
    # a_brake: measured stop from 10 m/s takes 13.9-15.0 m (2.4-2.9 s); the model below with
    #   a_brake 3 gives 4 + 4 + 16.7 = 24.7 m, so 3 stays a safe lower bound.
    a_brake: float = 3.0         # deceleration every agent achieves when told to stop (lower bound)
    # a_max / dv_max bound how far a neighbour can deviate from its constant-velocity prediction.
    #   The old a_max = 4 was a guess and is NOT an upper bound: measured deviation in plain
    #   cruise/stop was 3.7 m after 1 s (model 2.0) and 11.8 m after 2 s (model 8.0); the peak
    #   acceleration (0.5 s windows) is 7.8 m/s^2 (gazebo) / 6.6 (sitl) when starting from rest.
    #   The deviation then grows only linearly because the speed is capped (WP_SPD 12, peak 12.34
    #   measured), so: |dv| <= min(a_max*t, dv_max). a_max 8, dv_max 12.5 envelope every measured
    #   horizon 0.8-4.4 s, including a hard stop from 12 m/s (42.9 m at 4.4 s; model 45.2 m); at
    #   0.3 / 0.5 s the measured excess over the model is 0.17 / 0.02 m, inside `margin`.
    #   Not measured: a full-speed reversal (|dv| up to 24); the controller never commands one
    #   (coverage slows to 2 m/s before turns > 30 deg), but a filter sidestep can turn by 90 deg.
    a_max: float = 8.0           # horizontal acceleration no agent exceeds (upper bound, measured 7.8)
    dv_max: float = 12.5         # largest velocity change vs the last report (speed cap 12 + 0.35 peak)
    # t_react: measured, speed starts to fall 0.2-0.3 s after the zero setpoint is sent; + one 0.1 s tick.
    t_react: float = 0.4         # command period + autopilot response
    # stop_offset: the vehicle flies to the rabbit, up to shaper_lag(v) + 2 m ahead, plus the measured
    #   overshoot (1.23 m sitl). For 0.5-4 m/s that needs up to 3.8 m on top of v*t_react + v^2/2a.
    stop_offset: float = 4.0     # constant part of the stopping distance (position-target lag, see docstring)
    t_gap: float = 1.0           # last delivered message -> radio failure (see docstring)
    deaf_stop_s: float = C.DEAF_STOP_S
    suspect_s: float = C.SUSPECT_S
    lost_s: float = C.LOST_S
    z_track: float = 1.0         # altitude tracking error
    layer_spacing: float = 10.0
    alt_nominal: float = C.ALT_NOMINAL
    alt_band: Tuple[float, float] = C.ALT_BAND
    plan_ahead: float = 0.6      # plan against R(age + plan_ahead): the disc if the next 3 STATEs are lost
    hyst: float = 5.0            # fallback exit: distance >= R + hyst ...
    clear_s: float = 1.0         # ... continuously for this long
    boxed_s: float = 3.0
    boxed_speed: float = 0.5
    boxed_nominal: float = 1.0
    sidestep_min: float = 1.0    # tangential speed below which we add the right-hand sidestep

    @property
    def t_go(self) -> float:
        """Latest time after its last delivered message at which a silent neighbour starts braking."""
        return self.t_gap + self.deaf_stop_s + self.t_react

    @property
    def brake_dist_vmax(self) -> float:
        return self.v_max ** 2 / (2.0 * self.a_brake)

    def deviation(self, t: float) -> float:
        """Max distance from the constant-velocity prediction after t s with |dv| <= min(a_max t, dv_max)."""
        t = max(t, 0.0)
        t1 = self.dv_max / self.a_max
        if t <= t1:
            return 0.5 * self.a_max * t * t
        return 0.5 * self.a_max * t1 * t1 + self.dv_max * (t - t1)

    def uncertainty(self, age: float) -> float:
        """How far (m) a neighbour can be from its predicted centre after `age` s of silence."""
        age = max(age, 0.0)
        if age <= self.t_go:
            return self.deviation(age)
        return self.deviation(self.t_go) + min(self.v_max * (age - self.t_go), self.brake_dist_vmax)

    def radius(self, age: float) -> float:
        """Keep-out radius (m) around a neighbour's predicted centre."""
        return self.d_min + self.margin + self.uncertainty(age)

    def v_stop(self, gap: float) -> float:
        """Largest speed v (m/s) with stop_offset + v*t_react + v^2/(2*a_brake) <= gap (m)."""
        gap -= self.stop_offset
        if gap <= 0.0:
            return 0.0
        a, tr = self.a_brake, self.t_react
        return a * (-tr + math.sqrt(tr * tr + 2.0 * gap / a))

    def layers(self) -> List[float]:
        lo, hi = self.alt_band
        n = int(math.floor((hi - lo) / self.layer_spacing + 1e-9)) + 1
        return [lo + i * self.layer_spacing for i in range(n)]

    def layer_of(self, agent_id: int) -> float:
        """Own fallback altitude (m). Static by ID so all agents agree without communicating."""
        ls = self.layers()
        return ls[(agent_id - 1) % len(ls)]


@dataclass
class Neighbour:
    nid: int
    pos: Tuple[float, float, float]
    vel: Tuple[float, float, float]
    status: str
    t_sent: float
    t_recv: float
    suspect: bool = False

    def age(self, now: float) -> float:
        return now - min(self.t_sent, self.t_recv)


@dataclass
class Threat:
    nid: int
    age: float
    centre: Tuple[float, float]
    dist_h: float             # horizontal distance to predicted centre, m
    radius: float             # R(age): being inside this is a violation -> fallback
    radius_plan: float        # R(age + plan_ahead): the velocity filter keeps us outside this
    n: Tuple[float, float]    # unit vector us -> centre
    vert_sep: bool
    dist_3d: float

    @property
    def gap(self) -> float:
        return self.dist_h - self.radius

    @property
    def gap_plan(self) -> float:
        return self.dist_h - self.radius_plan


@dataclass
class SafeCommand:
    vx: float
    vy: float
    z: float
    mode: str
    flags: List[str] = field(default_factory=list)
    events: List[dict] = field(default_factory=list)
    infeasible: bool = False             # True on the tick fallback is entered from NORMAL
    min_nbr_dist: Optional[float] = None  # 3D distance to the nearest neighbour as believed
    min_nbr_age: Optional[float] = None   # age of that neighbour's state


def _norm(x: float, y: float) -> float:
    return math.hypot(x, y)


def _right_of(n: Tuple[float, float]) -> Tuple[float, float]:
    """Unit vector 90 deg clockwise from n (our right when facing along n)."""
    return (n[1], -n[0])


class SafetyFilter:
    """One per agent. Feed it heard()/update_neighbour() as messages arrive, call step() each tick."""

    def __init__(self, agent_id: int, params: SafetyParams = SafetyParams()):
        self.id = agent_id
        self.p = params
        self.layer = params.layer_of(agent_id)
        self.nbrs: Dict[int, Neighbour] = {}
        self.mode = LAYERED
        self.layer_reason = "start"
        self._last_heard: Optional[float] = None
        self._clear_since: Optional[float] = None
        self._stall_since: Optional[float] = None

    # ---- inputs -----------------------------------------------------------------------------
    def heard(self, now: float) -> None:
        """Any message from anyone (agent or ground) was received at `now`."""
        self._last_heard = now

    def update_neighbour(self, nid: int, pos, vel, status: str, t_sent: float, t_recv: float) -> None:
        """Store a received STATE. Out-of-order (older) states are ignored."""
        if nid == self.id:
            return
        old = self.nbrs.get(nid)
        if old is not None and t_sent <= old.t_sent:
            return
        self.nbrs[nid] = Neighbour(nid, tuple(pos), tuple(vel), status, t_sent, t_recv)

    # ---- geometry ---------------------------------------------------------------------------
    def _nbr_interval(self, nb: Neighbour) -> Tuple[float, float]:
        zs = (nb.pos[2], self.p.alt_nominal, self.p.layer_of(nb.nid))
        return min(zs) - self.p.z_track, max(zs) + self.p.z_track

    def _own_interval(self, z: float, z_target: float) -> Tuple[float, float]:
        return min(z, z_target) - self.p.z_track, max(z, z_target) + self.p.z_track

    def _threats(self, now: float, pos, z_target: float) -> List[Threat]:
        own_iv = self._own_interval(pos[2], z_target)
        out = []
        for nid in sorted(self.nbrs):
            nb = self.nbrs[nid]
            age = nb.age(now)
            t_pred = min(max(age, 0.0), self.p.t_go)
            cx = nb.pos[0] + nb.vel[0] * t_pred
            cy = nb.pos[1] + nb.vel[1] * t_pred
            dx, dy = cx - pos[0], cy - pos[1]
            d = _norm(dx, dy)
            n = (dx / d, dy / d) if d > 1e-6 else (1.0, 0.0)
            iv = self._nbr_interval(nb)
            vgap = max(own_iv[0] - iv[1], iv[0] - own_iv[1])
            d3 = math.sqrt(d * d + (nb.pos[2] - pos[2]) ** 2)
            out.append(Threat(nid, age, (cx, cy), d, self.p.radius(age), self.p.radius(age + self.p.plan_ahead),
                              n, vgap >= self.p.d_min, d3))
        return out

    # ---- velocity filter --------------------------------------------------------------------
    def filter_velocity(self, v_nom: Tuple[float, float], threats: List[Threat]) -> Tuple[Tuple[float, float], List[str]]:
        """Remove closing speed beyond each neighbour's share of the braking curve. Returns (v, flags)."""
        flags = []
        vx, vy = v_nom
        active = [t for t in threats if not t.vert_sep]
        for _ in range(3):
            changed = False
            for t in active:
                lim = 0.5 * self.p.v_stop(t.gap_plan)
                vn = vx * t.n[0] + vy * t.n[1]
                if vn <= lim + 1e-9:
                    continue
                changed = True
                tx, ty = vx - vn * t.n[0], vy - vn * t.n[1]
                removed = vn - lim
                vx, vy = tx + lim * t.n[0], ty + lim * t.n[1]
                if _norm(tx, ty) < self.p.sidestep_min:
                    r = _right_of(t.n)
                    vx, vy = vx + removed * r[0], vy + removed * r[1]
                    if "sidestep" not in flags:
                        flags.append("sidestep")
            if not changed:
                break
        # Final guarantee: uniform scale-down until every constraint holds (s = 0 always works).
        s = 1.0
        for t in active:
            lim = 0.5 * self.p.v_stop(t.gap_plan)
            vn = vx * t.n[0] + vy * t.n[1]
            if vn > lim + 1e-9:
                s = min(s, max(lim, 0.0) / vn)
        vx, vy = vx * s, vy * s
        if _norm(vx - v_nom[0], vy - v_nom[1]) > 0.1:
            flags.append("filtered")
        return (vx, vy), flags

    # ---- housekeeping -----------------------------------------------------------------------
    def _expire(self, now: float, events: List[dict]) -> None:
        for nid in sorted(self.nbrs):
            nb = self.nbrs[nid]
            silent = now - nb.t_recv
            if silent > self.p.lost_s:
                del self.nbrs[nid]
                events.append({"event": "neighbour_lost", "nbr": nid, "silent_s": round(silent, 2)})
            elif silent > self.p.suspect_s and not nb.suspect:
                nb.suspect = True
                events.append({"event": "neighbour_suspect", "nbr": nid, "silent_s": round(silent, 2)})
            elif silent <= self.p.suspect_s and nb.suspect:
                nb.suspect = False
                events.append({"event": "neighbour_back", "nbr": nid})

    def _clamp(self, vx: float, vy: float, z: float, flags: List[str]) -> Tuple[float, float, float]:
        s = _norm(vx, vy)
        if s > self.p.v_max:
            vx, vy = vx * self.p.v_max / s, vy * self.p.v_max / s
            flags.append("clamp")
        lo, hi = self.p.alt_band
        if not lo <= z <= hi:
            z = min(max(z, lo), hi)
            flags.append("zclamp")
        return vx, vy, z

    # ---- main entry -------------------------------------------------------------------------
    def step(self, now: float, pos, vel, nominal: Tuple[float, float, float]) -> SafeCommand:
        """pos, vel: own true state (m, m/s, ENU). nominal: (vx, vy, z) from the coverage layer."""
        events: List[dict] = []
        flags: List[str] = []
        infeasible = False
        if self._last_heard is None:
            self._last_heard = now
        self._expire(now, events)

        # Deaf rule first: it overrides everything.
        deaf = now - self._last_heard > self.p.deaf_stop_s
        if deaf and self.mode != DEAF:
            self.mode = DEAF
            events.append({"event": "deaf_enter", "silent_s": round(now - self._last_heard, 2)})
        elif not deaf and self.mode == DEAF:
            self.mode, self.layer_reason, self._clear_since = LAYERED, "deaf", None
            events.append({"event": "deaf_exit"})

        z_target = nominal[2] if self.mode == NORMAL else self.layer
        threats = self._threats(now, pos, z_target)
        nearest = min(threats, key=lambda t: t.dist_3d) if threats else None
        v_nom = (nominal[0], nominal[1])

        if self.mode == NORMAL:
            inside = [t for t in threats if not t.vert_sep and t.gap < 0.0]
            v, f = self.filter_velocity(v_nom, threats)
            boxed = self._update_stall(now, v_nom, v, vel)
            if inside or boxed:
                self.mode, self.layer_reason, self._clear_since = LAYERED, "fallback", None
                self._stall_since = None
                infeasible = True
                ev = {"event": "fallback_enter", "why": "inside" if inside else "boxed"}
                if inside:
                    ev["nbr"] = inside[0].nid
                    ev["gap_m"] = round(inside[0].gap, 2)
                events.append(ev)
            else:
                flags += f
                vx, vy = v
                z = nominal[2]

        if self.mode == LAYERED:
            if self._exit_clear(now, threats):
                self.mode = NORMAL
                events.append({"event": "fallback_exit", "reason": self.layer_reason})
                v, f = self.filter_velocity(v_nom, threats)
                flags += f
                vx, vy = v
                z = nominal[2]
            else:
                z = self.layer
                flags.append("fallback" if self.layer_reason == "fallback" else "layer")
                at_layer = abs(pos[2] - self.layer) <= self.p.z_track
                inside = any(not t.vert_sep and t.gap < 0.0 for t in threats)
                if at_layer and not inside:
                    v, f = self.filter_velocity(v_nom, threats)
                    flags += f
                    vx, vy = v
                else:
                    vx, vy = 0.0, 0.0
        elif self.mode == DEAF:
            vx, vy, z = 0.0, 0.0, self.layer
            flags.append("deaf")

        vx, vy, z = self._clamp(vx, vy, z, flags)
        return SafeCommand(vx, vy, z, self.mode, flags, events, infeasible,
                           nearest.dist_3d if nearest else None, nearest.age if nearest else None)

    def _update_stall(self, now: float, v_nom, v_safe, vel) -> bool:
        """Boxed in: we want to move (|nominal| >= 1 m/s), the filter is cutting the command, and our
        ACTUAL horizontal speed has stayed below boxed_speed for boxed_s. (Actual speed, not the
        command: the command can chatter around the threshold while the vehicle goes nowhere.)"""
        wants = _norm(*v_nom) >= self.p.boxed_nominal
        cut = _norm(v_safe[0] - v_nom[0], v_safe[1] - v_nom[1]) > 0.1
        if wants and cut and _norm(vel[0], vel[1]) < self.p.boxed_speed:
            if self._stall_since is None:
                self._stall_since = now
            return now - self._stall_since >= self.p.boxed_s
        self._stall_since = None
        return False

    def _exit_clear(self, now: float, threats: List[Threat]) -> bool:
        """Hysteresis: all neighbours horizontally beyond R + hyst for clear_s."""
        clear = all(t.gap >= self.p.hyst for t in threats)
        if not clear:
            self._clear_since = None
            return False
        if self._clear_since is None:
            self._clear_since = now
        return now - self._clear_since >= self.p.clear_s
