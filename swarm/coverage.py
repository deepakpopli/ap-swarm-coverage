"""Coverage controller: the assigned slot list -> nominal horizontal velocity. Pure logic.

Slots 0.. are flown once, then slots[loop_from:] repeat. The agent flies straight at the active
slot at WP_SPD_MS, keeps full speed through gentle turns and slows before sharp ones. The
safety filter picks the altitude (cruise or hold) and may change the velocity.

Feasibility feedback: a slot not reached within SLOT_TIMEOUT_S of becoming active is reported
once (event "infeasible"); the agent then holds and re-sends INFEASIBLE every second until a
newer epoch arrives. The timer pauses only while the agent is deaf: it could not report, and a
blackout says nothing about the slot. It keeps running while the safety filter blocks us,
because a slot we are kept away from is exactly what the ground has to hear about.
A newer epoch with the same slot list keeps the active slot, its timer and its report.

Shared tour (ASSIGN with t_ref, due_s, period_s; allocator._tour_plans): the agent starts at
slot 0 (its transit, then its tour entry), and loop slot i is due at
t_ref + due_s[i] + lap x period_s. Reaching its entry or a corner early, it holds there until
due, which keeps the agents on one tour evenly spaced. It never tries to make up lost time:
a late agent simply holds less (the schedule has slack at every corner). The slot timer
pauses while holding.
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from swarm import config

TURN_PASS_DEG = 30.0        # gentler turns are flown at full speed
V_TURN_MIN_MS = 2.0         # slowest corner speed
A_SLOW_MS2 = 1.5            # deceleration into sharp corners, well below the measured 4.2
INFEASIBLE_RESEND_S = 1.0
HOLD_MIN_S = 0.2            # shared tour: hold only when at least this early


@dataclass
class CoverageOutput:
    vx: float
    vy: float
    idle: bool              # nothing to fly: no plan yet, or the active slot timed out
    events: List[dict] = field(default_factory=list)


def _dist(a, b) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


class CoverageController:
    def __init__(self):
        self.epoch: Optional[int] = None
        self.slots: List[Tuple[float, float]] = []
        self.loop_from = 0
        self.idx = -1                      # active slot, -1 without a plan
        self.seg_start = (0.0, 0.0)        # where the leg to the active slot began
        self.active_s = 0.0                # time the active slot has been active (not deaf)
        self.reported = False              # the active slot timed out and was reported
        self.infeasible: Optional[dict] = None   # INFEASIBLE body, re-sent until a new epoch
        self.last_sent: Optional[float] = None
        self.last_now: Optional[float] = None
        self.schedule: Optional[Tuple[float, List[float], float]] = None  # t_ref, due_s, period
        self.lap = 0                       # completed loops under the current schedule
        self.entered = False               # reached the first loop slot of this plan
        self.hold_until: Optional[float] = None

    def on_assign(self, msg: dict, now: float, xy) -> Optional[dict]:
        """Apply an ASSIGN. Returns an event to log, or None for an old epoch."""
        if self.epoch is not None and msg["epoch"] <= self.epoch:
            return None
        self.epoch = msg["epoch"]
        self.infeasible = None
        slots = [(s[0], s[1]) for s in msg["slots"]]
        schedule = None
        if msg.get("due_s") is not None:
            schedule = (msg["t_ref"], msg["due_s"], msg["period_s"])
        if (slots == self.slots and msg["loop_from"] == self.loop_from
                and schedule == self.schedule):
            return {"event": "assign", "epoch": self.epoch, "same_plan": True}
        self.slots, self.loop_from, self.schedule = slots, msg["loop_from"], schedule
        self.lap, self.entered, self.hold_until = 0, False, None
        self.idx = 0 if schedule is not None and slots else self._nearest_slot(xy)
        self.seg_start = (xy[0], xy[1])
        self.active_s = 0.0
        self.reported = False
        return {"event": "assign", "epoch": self.epoch, "same_plan": False, "start": self.idx}

    def _nearest_slot(self, xy) -> int:
        """Start where we are: the nearest slot, the lowest index on a tie (-1 if none)."""
        if not self.slots:
            return -1
        return min(range(len(self.slots)), key=lambda i: (_dist(xy, self.slots[i]), i))

    def infeasible_due(self, now: float) -> Optional[dict]:
        """The INFEASIBLE body to send now (about once a second), or None."""
        if self.infeasible is None:
            return None
        if self.last_sent is not None and now - self.last_sent < INFEASIBLE_RESEND_S:
            return None
        self.last_sent = now
        return self.infeasible

    def step(self, now: float, xy, deaf: bool) -> CoverageOutput:
        """xy: own horizontal position (m). deaf: the safety filter heard nobody lately."""
        dt = 0.0 if self.last_now is None else now - self.last_now
        self.last_now = now
        if not self.slots or self.infeasible is not None:
            return CoverageOutput(0.0, 0.0, True)
        events = []
        if self.hold_until is not None:
            if now < self.hold_until:
                return CoverageOutput(0.0, 0.0, False)   # the slot timer pauses while holding
            self.hold_until = None
            self._next_slot(events)
        if not deaf:
            self.active_s += dt
        if self._reached(xy):
            early_s = self._early_s(now)
            if early_s >= HOLD_MIN_S:
                self.hold_until = now + early_s
                events.append({"event": "hold", "epoch": self.epoch, "slot": self.idx,
                               "s": round(early_s, 2)})
                return CoverageOutput(0.0, 0.0, False, events)
            self._next_slot(events)
        if self.active_s > config.SLOT_TIMEOUT_S and not self.reported:
            self.reported = True
            self.infeasible = {"epoch": self.epoch, "slot": self.idx,
                               "slot_xy": list(self.slots[self.idx])}
            events.append(dict(self.infeasible, event="infeasible"))
            return CoverageOutput(0.0, 0.0, True, events)
        vx, vy = self._velocity(xy)
        return CoverageOutput(vx, vy, False, events)

    def _early_s(self, now: float) -> float:
        """Shared tour: how early we reached the active slot, if we may hold there (our entry,
        or a corner, where we are slow anyway); else 0."""
        if self.schedule is None or self.idx < self.loop_from:
            return 0.0
        if self.entered and self._corner_speed_ms() >= config.WP_SPD_MS:
            return 0.0
        t_ref, due_s, period_s = self.schedule
        return max(0.0, t_ref + due_s[self.idx - self.loop_from] + self.lap * period_s - now)

    def _next_index(self, i: int) -> int:
        return i + 1 if i + 1 < len(self.slots) else self.loop_from

    def _next_slot(self, events: List[dict]) -> None:
        self.active_s = 0.0
        self.reported = False
        nxt = self._next_index(self.idx)
        if nxt == self.idx:                # a loop of one slot: stay on it
            return
        events.append({"event": "slot_reached", "epoch": self.epoch, "slot": self.idx})
        if self.idx >= self.loop_from:
            self.entered = True
        if nxt <= self.idx:                # wrapped to loop_from: the next lap begins
            self.lap += 1
        self.seg_start = self.slots[self.idx]
        self.idx = nxt

    def _reached(self, xy) -> bool:
        """Within SLOT_REACH_M, or flown past it (the filter may have pushed us sideways)."""
        slot = self.slots[self.idx]
        if _dist(xy, slot) <= config.SLOT_REACH_M:
            return True
        leg = _dist(self.seg_start, slot)
        if leg < 1.0:
            return False
        ux = (slot[0] - self.seg_start[0]) / leg
        uy = (slot[1] - self.seg_start[1]) / leg
        rx, ry = xy[0] - self.seg_start[0], xy[1] - self.seg_start[1]
        along = rx * ux + ry * uy
        across = abs(rx * uy - ry * ux)
        return along >= leg and across <= 2.0 * config.SLOT_REACH_M

    def _velocity(self, xy) -> Tuple[float, float]:
        slot = self.slots[self.idx]
        d = _dist(xy, slot)
        if d < 0.01:
            return 0.0, 0.0
        corner = self._corner_speed_ms()
        speed = min(config.WP_SPD_MS, math.sqrt(corner ** 2 + 2.0 * A_SLOW_MS2 * d))
        return speed * (slot[0] - xy[0]) / d, speed * (slot[1] - xy[1]) / d

    def _corner_speed_ms(self) -> float:
        """Speed at the active slot: full through gentle turns, slower the sharper the turn."""
        slot = self.slots[self.idx]
        nxt = self.slots[self._next_index(self.idx)]
        leg_in, leg_out = _dist(self.seg_start, slot), _dist(slot, nxt)
        if leg_out < 1.0:                  # a loop of one slot: stop on it
            return 0.0
        if leg_in < 1.0:
            return V_TURN_MIN_MS
        cos_turn = ((slot[0] - self.seg_start[0]) * (nxt[0] - slot[0])
                    + (slot[1] - self.seg_start[1]) * (nxt[1] - slot[1])) / (leg_in * leg_out)
        if cos_turn >= math.cos(math.radians(TURN_PASS_DEG)):
            return config.WP_SPD_MS
        return max(V_TURN_MIN_MS, config.WP_SPD_MS * cos_turn)
