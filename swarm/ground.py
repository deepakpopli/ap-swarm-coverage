"""Ground station: tracks liveness, runs the allocator, sends assignments (never velocities).

    python -m swarm.ground --n 3 --port-base 0 --log run/ground.jsonl [--mission-s 180]
                           [--priority-zone xmin,ymin,xmax,ymax] [--swap-at T]

Start: when all n agents report "ready" (or READY_DEADLINE_S after the first did), plan for the
ready ones; they fly to their loops at once. When every assigned agent has reached a slot of its
loop under the current epoch (or ON_STATION_DEADLINE_S after the first plan), START is sent with
t0 = now + COVERAGE_WINDOW_S: the swarm observes for one full coverage window before the
mission clock reaches 0, so coverage at t = 0 already comes from a full window.
Liveness, from STATE receive times: alive -> suspect (SUSPECT_S) -> lost (LOST_S). A status
other than ready/cover/hold (e.g. "failsafe") counts as lost at once. A lost agent rejoins once
it has been heard steadily for REJOIN_S. Re-plan on: lost, rejoin, accepted INFEASIBLE.
Restart: see Ground.maybe_plan_first.

ground.jsonl: one JSON object per line, each with "t_wall", "t_mission" (null before start) and
"event", one of:
  ready       {agent}
  assign      {epoch, trigger, assignments: {id: {share, slots, loop_from}}, changed: [ids],
               uncovered: [[xmin, xmax, ymin, ymax], ...] or null}
  start       {t0, late: [ids not on station]}   (t_mission = -COVERAGE_WINDOW_S)
  liveness    {agent, from, to, status}       alive | suspect | lost; status = last reported
  infeasible  {agent, epoch, slot_xy, accepted, why}
  blacklist   {pairs: [{agent, slot_key, expires_t_wall}]}
  end         {epoch}
"""
import argparse
import json
import os
import signal
import time
from dataclasses import dataclass

from swarm import config
from swarm.allocator import allocate, slot_key
from swarm.net import RadioLink

READY_DEADLINE_S = 120.0       # plan without the agents not ready this long after the first
ON_STATION_DEADLINE_S = 120.0  # start without the agents not on their loop this long after
REJOIN_S = 3.0                 # a lost agent must be heard steadily this long to rejoin
RESTART_LISTEN_S = 3.0         # a restarted ground listens this long before planning
EPOCH_STRIDE = 1000            # a restarted ground jumps this far above any epoch it hears
FLYING = ("ready", "cover", "hold")
LOOP_S = 0.02


class JsonLines:
    """Appends one JSON object per line, stamped with the caller's `now`."""

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
    status: str = ""
    epoch: int = 0
    slot: int = -1
    on_loop: bool = False      # has reached a slot of its loop under its reported epoch
    liveness: str = "lost"     # alive | suspect | lost; lost = not in the plan


class Ground:
    def __init__(self, n, link, log, mission_s=config.MISSION_S, priority_zone=None,
                 previous_t0=None, swap_at=None):
        self.n = n
        self.link = link
        self.log = log
        self.mission_s = mission_s
        self.priority_zone = priority_zone
        self.swap_at = swap_at           # test hook, see maybe_swap
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

    # ---- one tick
    def step(self, now) -> bool:
        """Handle messages and timers. False once the mission is over."""
        for msg in self.link.recv_all():
            if msg["type"] == "STATE":
                self.on_state(msg, now)
            elif msg["type"] == "INFEASIBLE":
                self.on_infeasible(msg, now)
        if self.planned_at is None:
            self.maybe_plan_first(now)
            return True
        self.update_liveness(now)
        self.expire_blacklist(now)
        if self.t0 is None:
            self.maybe_start(now)
        self.maybe_swap(now)
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
        a = msg["src"]
        rec = self.agents.setdefault(a, AgentRecord(last_heard=now, good_since=now))
        if now - rec.last_heard >= config.SUSPECT_S or msg["status"] not in FLYING:
            rec.good_since = now
        if msg["status"] == "ready" and rec.status != "ready":
            self.log.write(now, "ready", agent=a)
            if self.first_ready is None:
                self.first_ready = now
        rec.last_heard = now
        rec.xy = msg["pos"][:2]
        rec.status = msg["status"]
        epoch = msg["epoch"] or 0        # null or 0 before the agent's first ASSIGN
        self.note_slot(a, rec, epoch, msg["slot"])
        rec.epoch = epoch
        rec.slot = msg["slot"]

    def note_slot(self, a, rec, epoch, slot):
        """The active slot only moves on when the agent reaches it. So when the slot changes
        under the same epoch, the previously reported slot was reached; if it is a loop slot,
        the agent is on station. A lost STATE can only delay this, never make it early."""
        if epoch != rec.epoch:
            rec.on_loop = False
            return
        plan = self.assignments.get(a)
        if plan is not None and epoch == self.epoch and slot != rec.slot:
            rec.on_loop = rec.on_loop or rec.slot >= plan.loop_from

    def on_infeasible(self, msg, now):
        """Blacklist the slot and re-plan, whatever epoch the report names, as long as the point
        is in the agent's current plan (an unrelated re-plan must not swallow the report)."""
        a, xy = msg["src"], msg["slot_xy"]
        key = slot_key(xy)
        plan = self.assignments.get(a)
        if plan is None or key not in [slot_key(p) for p in plan.slots]:
            why = "not in the current plan"
        elif (a, key) in self.blacklist:
            why = "already blacklisted"   # the agent repeats its report until a new epoch
        else:
            why = "new"
        self.log.write(now, "infeasible", agent=a, epoch=msg["epoch"], slot_xy=xy,
                       accepted=why == "new", why=why)
        if why == "new":
            self.blacklist[(a, key)] = now + config.BLACKLIST_COOLDOWN_S
            self.log_blacklist(now)
            self.replan(now, "infeasible")

    # ---- start
    def maybe_plan_first(self, now):
        """Plan once everyone is ready, or on the deadline.

        Restart: agents that already report an epoch are flying a plan from a ground that died.
        Listen RESTART_LISTEN_S, jump EPOCH_STRIDE above any epoch heard (so no agent ever sees
        an epoch number twice), keep t0 from the old log, and plan for everyone flying.
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
        rec = self.agents[a]
        return rec.epoch == self.epoch and rec.on_loop

    def set_t0(self, t0):
        self.t0 = t0
        self.log.t0 = t0
        self.next_start = 0.0            # send START now

    # ---- test hook for the "swap" scenario (run_mission.py --scenario swap)
    def maybe_swap(self, now):
        """At mission time swap_at, re-plan once with the strip order reversed, so every
        agent has to cross the others at cruise altitude: a stress test of the safety filter."""
        if self.swap_at is None or self.t0 is None or now < self.t0 + self.swap_at:
            return
        self.swap_at = None
        self.replan(now, "swap", reverse=True)

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
        """Heard, flying and without a gap of SUSPECT_S for the last REJOIN_S: no flapping."""
        return now - rec.last_heard < config.SUSPECT_S and now - rec.good_since >= REJOIN_S

    # ---- blacklist
    def expire_blacklist(self, now):
        """Expiry only frees the pair for later plans; it does not trigger a re-plan."""
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
    def replan(self, now, trigger, reverse=False):
        members = sorted(a for a, r in self.agents.items() if r.liveness != "lost")
        new = {}
        if members:
            positions = {a: self.agents[a].xy for a in members}
            new = allocate(members, positions, set(self.blacklist), self.assignments,
                           self.priority_zone, reverse)
        old = self.assignments
        changed = sorted(a for a in set(old) | set(new) if old.get(a) != new.get(a))
        uncovered = [rect for a in sorted(new) for rect in new[a].uncovered]
        self.epoch += 1
        self.assignments = new
        self.log.write(now, "assign", epoch=self.epoch, trigger=trigger, changed=changed,
                       assignments={str(a): new[a].to_dict() for a in sorted(new)},
                       uncovered=uncovered or None)
        self.next_assign = 0.0           # send the new plan now

    def send_due(self, now):
        """ASSIGN at ASSIGN_HZ, START once a second: repeated because the radio is lossy."""
        if now >= self.next_assign:
            for a in sorted(self.assignments):
                self.link.send(a, "ASSIGN", epoch=self.epoch, **self.assignments[a].to_dict())
            self.next_assign = now + 1.0 / config.ASSIGN_HZ
        if self.t0 is not None and now >= self.next_start:
            self.link.send("all", "START", t0=self.t0)
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
                    help="test hook: re-plan with the strip order reversed at this mission time")
    args = ap.parse_args(argv)
    signal.signal(signal.SIGTERM, _exit_on_signal)
    signal.signal(signal.SIGINT, _exit_on_signal)
    previous_t0 = last_t0(args.log)
    log = JsonLines(args.log)
    link = RadioLink(config.GROUND_ID, args.port_base)
    try:
        Ground(args.n, link, log, args.mission_s, args.priority_zone, previous_t0,
               args.swap_at).run()
    finally:
        log.close()
        link.close()


if __name__ == "__main__":
    main()
