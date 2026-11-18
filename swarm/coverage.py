"""Coverage controller: assigned slot list -> nominal command (vx, vy, z). Pure logic, no I/O, no clock.

Slot sequence: slots[0..] are flown once (transit), then slots[loop_from:] repeat forever.
If loop_from is out of range it is clamped to the last slot (the agent then station-keeps there).

Speed profile: aim straight at the active slot at V_CRUISE (10 m/s). Decelerate only into a slot where
the path turns sharply: turn angle <= 30 deg -> pass through at cruise; otherwise the corner speed is
max(2 m/s, V_CRUISE*cos(turn)), approached on a braking curve with A_COV = 1.5 m/s^2. A loop of one
slot (or the final station-keeping slot) is approached to a stop and then held with a P-controller.

Slot reached (either):
  * horizontal distance <= SLOT_REACH_M (5 m), or
  * passed along-track: projected onto the segment (previous slot -> active slot) we are beyond the
    slot, and the cross-track error is <= 2*SLOT_REACH_M (10 m; the safety filter may have pushed us
    sideways). This avoids turning back for a slot we flew past while sidestepping.
The first segment after an ASSIGN starts at our position at activation.

Epochs: ASSIGN with epoch <= current is ignored. A newer epoch restarts at slot 0, unless the new list
contains the currently active slot (same x, y within 1 m): then we continue from that index.

Timeout: the active slot must be reached within SLOT_TIMEOUT_S (15 s) of becoming active (activation =
slot switch, new epoch, or mission start). Otherwise: one infeasible event (epoch, slot, "timeout"), and
nominal velocity zero (hold) until a newer epoch. INFEASIBLE is re-sent at 1 Hz via
infeasible_message(). At most one infeasible per epoch, whatever the reason ("timeout" or "safety").
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional

from swarm import config as C

SAME_SLOT_M = 1.0
TURN_PASS_DEG = 30.0
V_TURN_MIN = 2.0
A_COV = 1.5
STATION_KP = 0.5
INFEASIBLE_RESEND_S = 1.0


@dataclass
class CoverageOutput:
    vx: float
    vy: float
    z: float
    slot: int                 # active slot index, -1 if none
    holding: bool             # True when coverage itself asks to hold (no plan, or timed out)
    events: List[dict] = field(default_factory=list)


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _norm(v):
    return math.hypot(v[0], v[1])


def _angle_deg(u, v) -> float:
    nu, nv = _norm(u), _norm(v)
    if nu < 1e-6 or nv < 1e-6:
        return 180.0
    c = (u[0] * v[0] + u[1] * v[1]) / (nu * nv)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


class CoverageController:
    def __init__(self, agent_id: int, v_cruise: float = C.V_CRUISE, z: float = C.ALT_NOMINAL,
                 reach_m: float = C.SLOT_REACH_M, timeout_s: float = C.SLOT_TIMEOUT_S):
        self.id = agent_id
        self.v_cruise, self.z, self.reach_m, self.timeout_s = v_cruise, z, reach_m, timeout_s
        self.epoch: Optional[int] = None
        self.slots: List[tuple] = []
        self.loop_from = 0
        self.idx = 0
        self._active_since: Optional[float] = None   # None = activate on next step()
        self._seg_start: Optional[tuple] = None
        self._timed_out = False
        self._infeasible: Optional[dict] = None       # body of the pending INFEASIBLE for this epoch
        self._infeasible_sent_at: Optional[float] = None

    # ---- inputs -----------------------------------------------------------------------------
    def on_assign(self, body: dict, now: float) -> Optional[dict]:
        """Apply an ASSIGN body. Returns an event dict if accepted, None if ignored (old epoch)."""
        epoch = int(body["epoch"])
        if self.epoch is not None and epoch <= self.epoch:
            return None
        slots = [(float(s[0]), float(s[1])) for s in body.get("slots", [])]
        old_active = self.active_slot()
        new_idx = 0
        continued = False
        if old_active is not None:
            for i, s in enumerate(slots):
                if _norm(_sub(s, old_active)) <= SAME_SLOT_M:
                    new_idx, continued = i, True
                    break
        self.epoch = epoch
        self.slots = slots
        lf = int(body.get("loop_from", 0))
        self.loop_from = min(max(lf, 0), max(len(slots) - 1, 0))
        self.idx = new_idx
        self._active_since = None
        if not continued:
            self._seg_start = None
        self._timed_out = False
        self._infeasible = None
        self._infeasible_sent_at = None
        return {"event": "assign", "epoch": epoch, "n_slots": len(slots), "loop_from": self.loop_from,
                "start_idx": new_idx, "continued": continued}

    def active_slot(self) -> Optional[tuple]:
        if not self.slots:
            return None
        return self.slots[self.idx]

    def _next_idx(self, i: int) -> int:
        return i + 1 if i + 1 < len(self.slots) else self.loop_from

    def _station_keeping(self) -> bool:
        """True if the active slot is a loop of one (next slot is itself)."""
        return self._next_idx(self.idx) == self.idx

    # ---- infeasible handling ----------------------------------------------------------------
    def _raise(self, now: float, reason: str, pos) -> Optional[dict]:
        if self.epoch is None or self._infeasible is not None:
            return None
        self._infeasible = {"epoch": self.epoch, "slot": self.idx, "reason": reason,
                            "pos": [round(p, 2) for p in pos]}
        return dict(self._infeasible, event="infeasible")

    def report_safety(self, now: float, pos) -> Optional[dict]:
        """The safety filter entered fallback. Returns an event if this is the first report this epoch."""
        return self._raise(now, "safety", pos)

    def infeasible_message(self, now: float) -> Optional[dict]:
        """INFEASIBLE body to send now (first time immediately, then every 1 s), or None."""
        if self._infeasible is None:
            return None
        if self._infeasible_sent_at is not None and now - self._infeasible_sent_at < INFEASIBLE_RESEND_S:
            return None
        self._infeasible_sent_at = now
        return dict(self._infeasible)

    # ---- main entry -------------------------------------------------------------------------
    def step(self, now: float, pos) -> CoverageOutput:
        """pos: own (x, y, z) in m. Returns the nominal command."""
        events: List[dict] = []
        if not self.slots:
            return CoverageOutput(0.0, 0.0, self.z, -1, True, events)
        xy = (pos[0], pos[1])
        if self._active_since is None:
            self._active_since = now
            if self._seg_start is None:
                self._seg_start = xy
        if self._timed_out:
            return CoverageOutput(0.0, 0.0, self.z, self.idx, True, events)

        if not self._station_keeping():
            for _ in range(len(self.slots)):
                if not self._reached(xy):
                    break
                events.append({"event": "slot_reached", "epoch": self.epoch, "slot": self.idx,
                               "dt_s": round(now - self._active_since, 2)})
                self._seg_start = self.slots[self.idx]
                self.idx = self._next_idx(self.idx)
                self._active_since = now
                if self._station_keeping():
                    break
            if now - self._active_since > self.timeout_s:
                self._timed_out = True
                ev = self._raise(now, "timeout", pos)
                if ev:
                    events.append(ev)
                return CoverageOutput(0.0, 0.0, self.z, self.idx, True, events)

        vx, vy = self._velocity(xy)
        return CoverageOutput(vx, vy, self.z, self.idx, False, events)

    def _reached(self, xy) -> bool:
        slot = self.slots[self.idx]
        if _norm(_sub(slot, xy)) <= self.reach_m:
            return True
        seg = _sub(slot, self._seg_start)
        L = _norm(seg)
        if L < 1e-6:
            return False
        u = (seg[0] / L, seg[1] / L)
        rel = _sub(xy, self._seg_start)
        along = rel[0] * u[0] + rel[1] * u[1]
        cross = abs(rel[0] * u[1] - rel[1] * u[0])
        return along >= L and cross <= 2.0 * self.reach_m

    def _velocity(self, xy):
        slot = self.slots[self.idx]
        to = _sub(slot, xy)
        d = _norm(to)
        if d < 1e-6:
            return 0.0, 0.0
        if self._station_keeping():
            speed = min(self.v_cruise, STATION_KP * d, math.sqrt(2.0 * A_COV * d))
        else:
            nxt = self.slots[self._next_idx(self.idx)]
            turn = _angle_deg(_sub(slot, self._seg_start), _sub(nxt, slot))
            if turn <= TURN_PASS_DEG:
                v_corner = self.v_cruise
            else:
                v_corner = max(V_TURN_MIN, self.v_cruise * math.cos(math.radians(turn)))
            speed = min(self.v_cruise, math.sqrt(v_corner ** 2 + 2.0 * A_COV * d))
        return speed * to[0] / d, speed * to[1] / d
