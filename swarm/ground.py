"""Ground station: tracks liveness, runs the allocator, sends assignments (never velocities).

    python -m swarm.ground --n 3 --port-base 0 --log run/ground.jsonl [--mission-s 180]
                           [--priority-zone xmin,ymin,xmax,ymax] [--swap-at T] [--headon-at T]

Plans when all n agents are ready (or READY_DEADLINE_S after the first). Once every assigned
agent has reached a slot of its loop (or ON_STATION_DEADLINE_S), START sets t0 one coverage
window ahead, so coverage at t = 0 already comes from a full window.
Liveness from STATE receive times: alive -> suspect (SUSPECT_S) -> lost (LOST_S); a status other
than ready/cover/hold is lost at once; rejoin after REJOIN_S of steady contact.
Re-plan on: lost, rejoin, accepted INFEASIBLE. Restart: see Ground.maybe_plan_first.

ground.jsonl: one JSON object per line with "t_wall", "t_mission" (null before start), "event":
  ready       {agent}
  assign      {epoch, trigger, assignments: {id: {share, slots, loop_from}}, changed: [ids]};
              trigger: start, restart, lost, rejoin, infeasible, swap,
              headon_stage, headon, headon_end (test hooks)
  start       {t0, late: [ids not on station]}
  liveness    {agent, from, to, status}
  infeasible  {agent, epoch, slot_xy, accepted, why}
  blacklist   {pairs: [{agent, slot_key, expires_t_wall}]}
  pace        {epoch, speeds: {id: m/s}}: the butterfly's phase hold ({} = all at cruise)
  end         {epoch}
"""
import argparse
import json
import math
import os
import signal
import time
from dataclasses import dataclass

from swarm import config, protocol
from swarm.allocator import Assignment, allocate, butterfly_pace, slot_key
from swarm.net import RadioLink

READY_DEADLINE_S = 120.0       # plan without the agents not ready this long after the first
ON_STATION_DEADLINE_S = 120.0  # start without the agents not on their loop this long after
REJOIN_S = 3.0                 # a lost agent must be heard steadily this long to rejoin
RESTART_LISTEN_S = 3.0         # a restarted ground listens this long before planning
EPOCH_STRIDE = 1000            # a restarted ground jumps this far above any epoch it hears
FLYING = ("ready", "cover", "hold")
NO_PLAN = {"share": [], "slots": [], "loop_from": 0}   # ASSIGN body for an agent not in the plan
LOOP_S = 0.02
HEADON_GAP_M = 80.0            # "headon" test hook: the pair faces each other this far apart
HEADON_PHASE_S = 40.0          # each of its phases ends after this at the latest


class JsonLines:
    """Appends one JSON object per line; appending lets a restarted ground keep the old log."""

    def __init__(self, path):
        self.file = open(path, "a")
        self.t0 = None

    def write(self, now, event, **fields):
        t_mission = None if self.t0 is None else round(now - self.t0, 3)
        record = {"t_wall": now, "t_mission": t_mission, "event": event, **fields}
        self.file.write(json.dumps(record) + "\n")
        self.file.flush()

    def close(self):
        self.file.close()


def last_t0(path) -> float:
    """t0 of the last "start" record in an existing ground log, or None."""
    if not os.path.exists(path):
        return None
    t0 = None
    with open(path) as f:
        for line in f:
            try:
                record = json.loads(line)
            except ValueError:   # a killed ground can leave a half-written last line
                continue
            if record["event"] == "start":
                t0 = record["t0"]
    return t0


@dataclass
class AgentRecord:
    last_heard: float          # receive time of the last STATE
    good_since: float          # heard without a gap >= SUSPECT_S, and FLYING, since then
    xy: list = None
    vel: tuple = (0.0, 0.0)
    status: str = ""
    epoch: int = 0
    slot: int = -1
    on_loop: bool = False      # has reached a slot of its loop under its reported epoch
    liveness: str = "lost"     # alive | suspect | lost; lost = not in the plan


class Ground:
    def __init__(self, n, link, log, mission_s=config.MISSION_S, priority_zone=None,
                 previous_t0=None, swap_at=None, headon_at=None):
        self.n = n
        self.link = link
        self.log = log
        self.mission_s = mission_s
        self.priority_zone = priority_zone
        self.swap_at = swap_at           # test hook, see maybe_swap
        self.headon = None if headon_at is None else {"at": headon_at, "phase": "wait"}
        self.previous_t0 = previous_t0   # from the log of a ground that ran before this one
        self.agents = {}                 # id -> AgentRecord
        self.first_ready = None          # when the first agent reported "ready"
        self.restart_at = None           # when a restarted ground will plan
        self.planned_at = None           # when the first ASSIGN went out
        self.t0 = None
        self.epoch = 0
        self.assignments = {}            # id -> Assignment of the current epoch
        self.blacklist = {}              # (agent, slot_key) -> expiry wall time
        self.next_assign = 0.0
        self.next_start = 0.0
        self.pace = {}                   # id -> speed limit (allocator.butterfly_pace)
        self.next_pace = 0.0

    # ---- one tick
    def step(self, now) -> bool:
        """Handle messages and timers. False once the mission is over."""
        for msg in self.link.recv_all():
            if msg.get_type() == "SWARM_STATE":
                self.on_state(msg, now)
            elif msg.get_type() == "SWARM_INFEASIBLE":
                self.on_infeasible(msg, now)
        if self.planned_at is None:
            self.maybe_plan_first(now)
            return True
        self.update_liveness(now)
        self.expire_blacklist(now)
        if self.t0 is None:
            self.maybe_start(now)
        self.maybe_swap(now)
        self.maybe_headon(now)
        self.update_pace(now)
        self.send_due(now)
        return self.t0 is None or now < self.t0 + self.mission_s

    def run(self):
        try:
            while self.step(time.time()):
                time.sleep(LOOP_S)
        finally:
            self.log.write(time.time(), "end", epoch=self.epoch)

    # ---- messages
    def on_state(self, msg, now):
        a = msg.get_srcSystem()
        status = protocol.STATUSES[msg.status]
        rec = self.agents.setdefault(a, AgentRecord(last_heard=now, good_since=now))
        if now - rec.last_heard >= config.SUSPECT_S or status not in FLYING:
            rec.good_since = now
        if status == "ready" and rec.status != "ready":
            self.log.write(now, "ready", agent=a)
            if self.first_ready is None:
                self.first_ready = now
        rec.last_heard = now
        rec.xy = [msg.x, msg.y]
        rec.vel = (msg.vx, msg.vy)
        rec.status = status
        epoch = msg.epoch                # -1 before the agent's first ASSIGN
        self.note_slot(a, rec, epoch, msg.slot)
        rec.epoch = epoch
        rec.slot = msg.slot

    def note_slot(self, a, rec, epoch, slot):
        """A slot change under the same epoch means the previous slot was reached; a loop slot
        puts the agent on station. A lost STATE can only delay this."""
        if epoch != rec.epoch:
            rec.on_loop = False
            return
        plan = self.assignments.get(a)
        if plan is not None and epoch == self.epoch and slot != rec.slot:
            rec.on_loop = rec.on_loop or rec.slot >= plan.loop_from

    def on_infeasible(self, msg, now):
        """Blacklist and re-plan if the point is in the agent's current plan, whatever epoch the
        report names: an unrelated re-plan must not swallow it."""
        a, xy = msg.get_srcSystem(), [msg.slot_x / 10, msg.slot_y / 10]
        key = slot_key(xy)
        plan = self.assignments.get(a)
        if plan is None or key not in [slot_key(p) for p in plan.slots]:
            why = "not in the current plan"
        elif (a, key) in self.blacklist:
            why = "already blacklisted"   # the agent repeats its report until a new epoch
        else:
            why = "new"
        self.log.write(now, "infeasible", agent=a, epoch=msg.epoch, slot_xy=xy,
                       accepted=why == "new", why=why)
        if why == "new":
            self.blacklist[(a, key)] = now + config.BLACKLIST_COOLDOWN_S
            self.log_blacklist(now)
            self.replan(now, "infeasible")

    # ---- start
    def maybe_plan_first(self, now):
        """Plan once everyone is ready, or on the deadline.

        Restart (agents already report an epoch): listen RESTART_LISTEN_S, jump EPOCH_STRIDE
        above any epoch heard so no agent sees one twice, keep t0 from the old log, plan for
        everyone flying.
        """
        epochs = [r.epoch for r in self.agents.values() if r.epoch > 0]
        if epochs:
            if self.restart_at is None:
                self.restart_at = now + RESTART_LISTEN_S
            if now >= self.restart_at:
                self.epoch = max(epochs) + EPOCH_STRIDE
                if self.previous_t0 is not None:
                    self.set_t0(self.previous_t0)
                flying = [a for a, r in self.agents.items()
                          if r.status in FLYING and now - r.last_heard < config.SUSPECT_S]
                self.plan_first(now, flying, "restart")
            return
        ready = [a for a, r in self.agents.items() if r.status == "ready"]
        if len(ready) == self.n or (ready and now >= self.first_ready + READY_DEADLINE_S):
            self.plan_first(now, ready, "start")

    def plan_first(self, now, members, trigger):
        for a in members:
            self.agents[a].liveness = "alive"
        self.planned_at = now
        self.replan(now, trigger)

    def maybe_start(self, now):
        """Start the clock once every assigned agent flies its loop, or on the deadline."""
        late = sorted(a for a in self.assignments if not self.on_station(a))
        if late and now < self.planned_at + ON_STATION_DEADLINE_S:
            return
        self.set_t0(now + config.COVERAGE_WINDOW_S)
        self.log.write(now, "start", t0=self.t0, late=late)

    def on_station(self, a) -> bool:
        """On a slot of its loop under the current epoch; a spare agent (no slots) always."""
        rec = self.agents[a]
        if not self.assignments[a].slots:
            return True
        return rec.epoch == self.epoch and rec.on_loop

    def set_t0(self, t0):
        self.t0 = t0
        self.log.t0 = t0
        self.next_start = 0.0            # send START now

    # ---- test hook for the "swap" scenario
    def maybe_swap(self, now):
        """At swap_at, re-plan once with the groups reversed: the agents cross at cruise."""
        if self.swap_at is None or self.t0 is None or now < self.t0 + self.swap_at:
            return
        self.swap_at = None
        self.replan(now, "swap", reverse=True)

    # ---- test hook for the "headon" scenario
    def maybe_headon(self, now):
        """From headon_at: the two closest agents each get a one-slot plan, HEADON_GAP_M apart
        on the line through them (stage); once both hover there, each gets the other's slot, so
        they fly straight at each other at cruise (headon); once both arrive, a normal re-plan
        (headon_end). Each phase ends after HEADON_PHASE_S at the latest; any other re-plan in
        between ends the hook. Only the plans are made up: they go out as ASSIGN like any other."""
        h = self.headon
        if h is None or h["phase"] == "done" or self.t0 is None or now < self.t0 + h["at"]:
            return
        if h["phase"] != "wait" and self.epoch != h["epoch"]:
            h["phase"] = "done"          # someone else re-planned: lost, infeasible, ...
            return
        if h["phase"] == "wait":
            alive = sorted(a for a in self.assignments if self.agents[a].liveness == "alive")
            if len(alive) < 2:
                return
            pairs = [(a, b) for i, a in enumerate(alive) for b in alive[i + 1:]]
            a, b = min(pairs, key=lambda p: math.dist(self.agents[p[0]].xy, self.agents[p[1]].xy))
            (xa, ya), (xb, yb) = self.agents[a].xy, self.agents[b].xy
            d = max(math.hypot(xb - xa, yb - ya), 1e-6)
            ux, uy = (xb - xa) / d, (yb - ya) / d
            inset = HEADON_GAP_M / 2 + 2 * config.SLOT_REACH_M
            mx = min(max((xa + xb) / 2, config.AREA[0] + inset), config.AREA[1] - inset)
            my = min(max((ya + yb) / 2, config.AREA[2] + inset), config.AREA[3] - inset)
            half = HEADON_GAP_M / 2
            h["slots"] = {a: [round(mx - ux * half, 1), round(my - uy * half, 1)],
                          b: [round(mx + ux * half, 1), round(my + uy * half, 1)]}
            self.headon_plan(now, h, "stage", h["slots"])
        elif h["phase"] == "stage":
            if self.headon_arrived(h["slots"]) or now > h["since"] + HEADON_PHASE_S:
                a, b = sorted(h["slots"])
                h["slots"] = {a: h["slots"][b], b: h["slots"][a]}
                self.headon_plan(now, h, "cross", h["slots"])
        elif self.headon_arrived(h["slots"]) or now > h["since"] + HEADON_PHASE_S:
            h["phase"] = "done"
            self.replan(now, "headon_end")

    def headon_plan(self, now, h, phase, slots):
        """One-slot plans for the pair; a point as its share, so no later plan keeps them."""
        fixed = {a: Assignment([x, x, y, y], [[x, y]], 0) for a, (x, y) in slots.items()}
        self.replan(now, "headon" if phase == "cross" else "headon_stage", fixed=fixed)
        h.update(phase=phase, since=now, epoch=self.epoch)

    def headon_arrived(self, slots) -> bool:
        return all(math.dist(self.agents[a].xy, xy) <= config.SLOT_REACH_M
                   and math.hypot(*self.agents[a].vel) < 1.0 for a, xy in slots.items())

    # ---- liveness
    def update_liveness(self, now):
        for a in sorted(self.agents):
            rec = self.agents[a]
            silent_s = now - rec.last_heard
            if rec.status not in FLYING or silent_s >= config.LOST_S:
                new = "lost"
            elif rec.liveness == "lost" and not self.heard_steadily(rec, now):
                new = "lost"
            elif silent_s >= config.SUSPECT_S:
                new = "suspect"
            else:
                new = "alive"
            if new != rec.liveness:
                self.log.write(now, "liveness", agent=a, status=rec.status,
                               **{"from": rec.liveness, "to": new})
                was_lost = rec.liveness == "lost"
                rec.liveness = new
                if new == "lost":
                    self.replan(now, "lost")
                elif was_lost:
                    self.replan(now, "rejoin")

    @staticmethod
    def heard_steadily(rec, now) -> bool:
        """Flying, without a SUSPECT_S gap, for the last REJOIN_S: no flapping."""
        return now - rec.last_heard < config.SUSPECT_S and now - rec.good_since >= REJOIN_S

    # ---- blacklist
    def expire_blacklist(self, now):
        """Expiry only frees the pair for later plans: no re-plan."""
        expired = [pair for pair, t in self.blacklist.items() if t <= now]
        for pair in expired:
            del self.blacklist[pair]
        if expired:
            self.log_blacklist(now)

    def log_blacklist(self, now):
        pairs = [{"agent": a, "slot_key": list(key), "expires_t_wall": t}
                 for (a, key), t in sorted(self.blacklist.items())]
        self.log.write(now, "blacklist", pairs=pairs)

    # ---- planning and sending
    def replan(self, now, trigger, reverse=False, fixed=None):
        """fixed: test hooks only, {id: Assignment} put over the current plan."""
        members = sorted(a for a, r in self.agents.items() if r.liveness != "lost")
        new = {}
        if fixed is not None:
            new = {**self.assignments, **fixed}
        elif members:
            positions = {a: self.agents[a].xy for a in members}
            velocities = {a: self.agents[a].vel for a in members}
            heading = {a: self.heading_slot(a) for a in members}
            new = allocate(members, positions, set(self.blacklist), self.assignments,
                           self.priority_zone, reverse, velocities, heading)
        old = self.assignments
        changed = sorted(a for a in set(old) | set(new) if old.get(a) != new.get(a))
        self.epoch += 1
        self.assignments = new
        self.pace = {}
        self.log.write(now, "assign", epoch=self.epoch, trigger=trigger, changed=changed,
                       assignments={str(a): new[a].to_dict() for a in sorted(new)})
        self.next_assign = 0.0           # send the new plan now

    def update_pace(self, now):
        """Once a second, with both butterfly agents on their loops and heard steadily: the
        phase hold's speed limits (allocator.butterfly_pace), sent at once when they change."""
        if self.priority_zone is None or now < self.next_pace:
            return
        self.next_pace = now + 1.0
        pace = {}
        if self.assignments and all(self.on_station(a) and self.agents[a].liveness == "alive"
                                    for a in self.assignments):
            motion = {a: (self.agents[a].xy, self.agents[a].vel, self.heading_slot(a))
                      for a in self.assignments}
            pace = butterfly_pace(self.assignments, self.priority_zone, motion, bool(self.pace))
        if pace != self.pace:
            self.pace = pace
            self.log.write(now, "pace", epoch=self.epoch, speeds={str(a): v for a, v in pace.items()})
            self.next_assign = 0.0

    def heading_slot(self, a):
        """The slot agent a flies to, if it reports one of our current plan (unknown after a
        restart)."""
        rec, plan = self.agents[a], self.assignments.get(a)
        if plan is None or rec.epoch != self.epoch or not 0 <= rec.slot < len(plan.slots):
            return None
        return tuple(plan.slots[rec.slot])

    def send_due(self, now):
        """ASSIGN at ASSIGN_HZ, START once a second (the radio is lossy). A lost agent gets an
        empty ASSIGN: if it still hears it must leave lanes now someone else's,
        since the others forget it after LOST_S; with no slots it holds."""
        if now >= self.next_assign:
            for a in sorted(self.agents):
                body = self.assignments[a].to_dict() if a in self.assignments else NO_PLAN
                self.link.send(protocol.assign(a, self.epoch, **body, speed_ms=self.pace.get(a)))
            self.next_assign = now + 1.0 / config.ASSIGN_HZ
        if self.t0 is not None and now >= self.next_start:
            self.link.send(protocol.start(self.t0))
            self.next_start = now + 1.0


def parse_zone(text) -> tuple:
    """'xmin,ymin,xmax,ymax' -> a tuple of 4 floats."""
    return tuple(float(v) for v in text.split(","))


def _exit_on_signal(signum, frame):
    raise SystemExit(0)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Ground station: liveness + allocator")
    ap.add_argument("--n", type=int, required=True)
    ap.add_argument("--port-base", type=int, default=0)
    ap.add_argument("--log", required=True)
    ap.add_argument("--mission-s", type=float, default=config.MISSION_S)
    ap.add_argument("--priority-zone", type=parse_zone, default=None,
                    help="Mission B: xmin,ymin,xmax,ymax")
    ap.add_argument("--swap-at", type=float, default=None,
                    help="test hook: re-plan with the groups reversed at this mission time")
    ap.add_argument("--headon-at", type=float, default=None,
                    help="test hook: fly the two closest agents head-on from this mission time")
    args = ap.parse_args(argv)
    signal.signal(signal.SIGTERM, _exit_on_signal)
    signal.signal(signal.SIGINT, _exit_on_signal)
    previous_t0 = last_t0(args.log)
    log = JsonLines(args.log)
    link = RadioLink(config.GROUND_ID, args.port_base)
    try:
        Ground(args.n, link, log, args.mission_s, args.priority_zone, previous_t0,
               args.swap_at, args.headon_at).run()
    finally:
        log.close()
        link.close()


if __name__ == "__main__":
    main()
