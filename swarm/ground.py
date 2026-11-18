"""Ground station: liveness tracking + the deterministic allocator. Sends assignments only.

    python -m swarm.ground --n 3 --port-base 0 --log run_dir/ground.jsonl [--mission-s 180]

Talks only through RadioLink(0). Never sends velocities.

Phases
  wait   Collect STATE. When all n agents (ids 1..n) report status "ready", set
         t0 = now + 1.0 and start. Restart case: if any agent already reports status "cover"
         (or an epoch > 0), the ground was restarted mid-mission: it gathers STATE for
         RESUME_GATHER_S, reuses t0 from the last "start" record in its own log file if there
         is one (else t0 = now), and continues with epoch = 1 + the highest epoch seen in
         agent STATEs or in its log.
  run    START {t0} broadcast at 1 Hz for the whole mission. ASSIGN to each live agent at
         ASSIGN_HZ (and immediately after every re-plan). Exits at t0 + mission_s.

Liveness per agent, from the wall time of its last STATE:
  alive -> suspect after SUSPECT_S silent -> lost after LOST_S silent; suspect -> alive when
  heard. lost -> alive when heard = "rejoin". Agents never heard since a restart are "unknown".
  The allocator's live set = agents that are alive or suspect.

Re-plan triggers (each bumps the global epoch and logs a full "assign" record):
  "lost"       an agent went suspect -> lost
  "rejoin"     a lost/unknown agent was heard again
  "infeasible" INFEASIBLE for the CURRENT epoch. Reports for any other epoch are ignored, so
               an agent repeating its report until the new epoch reaches it cannot trigger
               more re-plans. The (agent, slot_key) pair goes on the blacklist until
               now + BLACKLIST_COOLDOWN_S. Infeasible-triggered re-plans are at least
               --replan-min-s apart; reports arriving inside that window are blacklisted
               at once and folded into one deferred re-plan.
  Positions used for allocation are the last STATE positions (stale by design).

ground.jsonl: one JSON object per line. Every line has "event", "t_wall" (wall time) and
"t_mission" (t_wall - t0, null before t0 is known). Events:
  "config"     {n, port_base, mission_s, suspect_s, lost_s, cooldown_s, replan_min_s, params}
  "start"      {t0, epoch_start, resumed: bool, alive: [ids]}
  "liveness"   {agent, from, to}        states: unknown|alive|suspect|lost
  "infeasible" {agent, epoch, slot, slot_key: [x, y], reason, pos, action}
               action: "replan" (re-plan now) | "deferred" (re-plan after the min interval)
               (duplicates for an already-blacklisted pair and old-epoch reports are not logged
               individually; they are counted in the "end" record)
  "blacklist"  {entries: [{agent, slot_key: [x, y], expires_t_wall}]}   on every change
  "assign"     {epoch, trigger, trigger_detail, alive: [ids],
                assignments: {"<id>": {share, slots, loop_from, note}},
                changed: [ids]}     ids whose share or slot list differs from the previous
                                    epoch (also ids that appeared or disappeared)
  "end"        {epoch, ignored_old_epoch, ignored_duplicate, reason}
"""
import argparse
import json
import os
import signal
import sys
import time

from swarm import config
from swarm.allocator import AllocParams, allocate, slot_key
from swarm.net import RadioLink

START_PERIOD_S = 1.0
START_LEAD_S = 1.0          # t0 = now + this when everyone is ready
RESUME_GATHER_S = 3.0       # restart: listen this long before re-planning
LOOP_PERIOD_S = 0.02
DEFAULT_REPLAN_MIN_S = 2.0


class JsonlLog:
    def __init__(self, path):
        self.f = open(path, "a") if path else None
        self.t0 = None

    def write(self, event, **rec):
        now = time.time()
        line = {"event": event, "t_wall": now,
                "t_mission": (now - self.t0) if self.t0 is not None else None}
        line.update(rec)
        if self.f:
            self.f.write(json.dumps(line) + "\n")
            self.f.flush()

    def close(self):
        if self.f:
            self.f.close()


def read_previous_log(path):
    """(t0 of the last 'start' record or None, highest epoch logged or 0)."""
    t0, epoch = None, 0
    if not path or not os.path.exists(path):
        return t0, epoch
    with open(path) as f:
        for line in f:
            try:
                rec = json.loads(line)
            except ValueError:
                continue
            if rec.get("event") == "start":
                t0 = rec.get("t0", t0)
            if rec.get("event") == "assign":
                epoch = max(epoch, int(rec.get("epoch", 0)))
    return t0, epoch


class AgentRec:
    def __init__(self):
        self.last_heard = None
        self.pos = None
        self.status = None
        self.epoch = 0
        self.liveness = "unknown"


class Ground:
    def __init__(self, n, link, log, params, mission_s, suspect_s, lost_s, cooldown_s,
                 replan_min_s, log_path=None):
        self.n = n
        self.link = link
        self.log = log
        self.params = params
        self.mission_s = mission_s
        self.suspect_s = suspect_s
        self.lost_s = lost_s
        self.cooldown_s = cooldown_s
        self.replan_min_s = replan_min_s
        self.log_path = log_path
        self.agents = {}                 # id -> AgentRec
        self.phase = "wait"
        self.resume_since = None
        self.t0 = None
        self.epoch = 0
        self.assignments = {}            # id -> Assignment (current epoch)
        self.blacklist = {}              # (agent, slot_key) -> expiry wall time
        self.pending_infeasible = False
        self.last_infeasible_replan = float("-inf")
        self.next_start_send = 0.0
        self.next_assign_send = 0.0
        self.ignored_old_epoch = 0
        self.ignored_duplicate = 0

    # ------------------------------------------------------------ message handling
    def on_message(self, msg, now):
        src = msg.get("src")
        if not isinstance(src, int) or not 1 <= src <= self.n:
            return
        if msg.get("type") == "STATE":
            self.on_state(src, msg, now)
        elif msg.get("type") == "INFEASIBLE" and self.phase == "run":
            self.on_infeasible(src, msg, now)

    def on_state(self, src, msg, now):
        rec = self.agents.setdefault(src, AgentRec())
        rec.last_heard = now
        rec.pos = msg.get("pos", rec.pos)
        rec.status = msg.get("status", rec.status)
        rec.epoch = int(msg.get("epoch", 0) or 0)
        if self.phase == "run" and rec.liveness in ("lost", "unknown"):
            self.set_liveness(src, "alive")
            self.replan("rejoin", {"agent": src}, now)
        elif self.phase == "run" and rec.liveness == "suspect":
            self.set_liveness(src, "alive")

    def on_infeasible(self, src, msg, now):
        epoch = msg.get("epoch")
        asg = self.assignments.get(src)
        if epoch != self.epoch or asg is None:
            self.ignored_old_epoch += 1
            return
        idx = msg.get("slot")
        if not isinstance(idx, int) or not 0 <= idx < len(asg.slots):
            return
        key = slot_key(asg.slots[idx])
        if (src, key) in self.blacklist:
            self.ignored_duplicate += 1
            return
        self.blacklist[(src, key)] = now + self.cooldown_s
        due = now - self.last_infeasible_replan >= self.replan_min_s
        self.log.write("infeasible", agent=src, epoch=epoch, slot=idx, slot_key=list(key),
                       reason=msg.get("reason"), pos=msg.get("pos"),
                       action="replan" if due else "deferred")
        self.log_blacklist()
        self.pending_infeasible = True

    # ------------------------------------------------------------ liveness
    def set_liveness(self, agent, new):
        rec = self.agents[agent]
        if rec.liveness != new:
            self.log.write("liveness", agent=agent, **{"from": rec.liveness, "to": new})
            rec.liveness = new

    def update_liveness(self, now):
        for a in sorted(self.agents):
            rec = self.agents[a]
            if rec.liveness in ("unknown", "lost") or rec.last_heard is None:
                continue
            silent = now - rec.last_heard
            if silent >= self.lost_s:
                self.set_liveness(a, "lost")
                self.replan("lost", {"agent": a}, now)
            elif silent >= self.suspect_s and rec.liveness == "alive":
                self.set_liveness(a, "suspect")

    def live_ids(self):
        return sorted(a for a, r in self.agents.items() if r.liveness in ("alive", "suspect"))

    # ------------------------------------------------------------ blacklist
    def log_blacklist(self):
        entries = [{"agent": a, "slot_key": list(k), "expires_t_wall": exp}
                   for (a, k), exp in sorted(self.blacklist.items())]
        self.log.write("blacklist", entries=entries)

    def expire_blacklist(self, now):
        old = [pair for pair, exp in self.blacklist.items() if exp <= now]
        for pair in old:
            del self.blacklist[pair]
        if old:
            self.log_blacklist()

    # ------------------------------------------------------------ planning
    def replan(self, trigger, detail, now):
        alive = self.live_ids()
        positions = {a: self.agents[a].pos for a in alive if self.agents[a].pos}
        new = allocate(alive, positions, set(self.blacklist), self.params) if alive else {}
        self.epoch += 1
        old = self.assignments
        changed = sorted(a for a in set(old) | set(new)
                         if a not in old or a not in new
                         or old[a].share != new[a].share or old[a].slots != new[a].slots)
        self.assignments = new
        self.log.write("assign", epoch=self.epoch, trigger=trigger, trigger_detail=detail,
                       alive=alive, changed=changed,
                       assignments={str(a): dict(new[a].to_dict(), note=new[a].note)
                                    for a in sorted(new)})
        self.pending_infeasible = False
        if trigger == "infeasible":
            self.last_infeasible_replan = now
        self.send_assignments()
        self.next_assign_send = now + 1.0 / config.ASSIGN_HZ

    def maybe_deferred_replan(self, now):
        if self.pending_infeasible and now - self.last_infeasible_replan >= self.replan_min_s:
            self.replan("infeasible", {"blacklist_size": len(self.blacklist)}, now)

    def send_assignments(self):
        alive = sorted(self.assignments)
        for a in alive:
            d = self.assignments[a].to_dict()
            self.link.send(a, "ASSIGN", epoch=self.epoch, agent=a, alive=alive, **d)

    # ------------------------------------------------------------ start / restart
    def maybe_start(self, now):
        ids = range(1, self.n + 1)
        if any(r.status == "cover" or r.epoch > 0 for r in self.agents.values()):
            if self.resume_since is None:
                self.resume_since = now
            if now - self.resume_since >= RESUME_GATHER_S:
                self.start(now, resumed=True)
        elif all(a in self.agents and self.agents[a].status == "ready" for a in ids):
            self.start(now, resumed=False)

    def start(self, now, resumed):
        prev_t0, prev_epoch = read_previous_log(self.log_path) if resumed else (None, 0)
        if resumed:
            self.t0 = prev_t0 if prev_t0 is not None else now
            self.epoch = max([prev_epoch] + [r.epoch for r in self.agents.values()])
        else:
            self.t0 = now + START_LEAD_S
        self.log.t0 = self.t0
        self.phase = "run"
        for a in sorted(self.agents):
            if now - self.agents[a].last_heard < self.lost_s:
                self.set_liveness(a, "alive")
        self.log.write("start", t0=self.t0, epoch_start=self.epoch + 1, resumed=resumed,
                       alive=self.live_ids())
        self.link.send("all", "START", t0=self.t0)
        self.next_start_send = now + START_PERIOD_S
        self.replan("restart" if resumed else "start", {}, now)

    # ------------------------------------------------------------ main loop
    def step(self, now):
        for msg in self.link.recv_all():
            self.on_message(msg, now)
        if self.phase == "wait":
            self.maybe_start(now)
            return True
        self.update_liveness(now)
        self.expire_blacklist(now)
        self.maybe_deferred_replan(now)
        if now >= self.next_start_send:
            self.link.send("all", "START", t0=self.t0)
            self.next_start_send = now + START_PERIOD_S
        if now >= self.next_assign_send:
            self.send_assignments()
            self.next_assign_send = now + 1.0 / config.ASSIGN_HZ
        return now < self.t0 + self.mission_s

    def run(self):
        reason = "mission_end"
        try:
            while self.step(time.time()):
                time.sleep(LOOP_PERIOD_S)
        except SystemExit:
            reason = "signal"
        finally:
            self.log.write("end", epoch=self.epoch, ignored_old_epoch=self.ignored_old_epoch,
                           ignored_duplicate=self.ignored_duplicate, reason=reason)


def _exit_on_signal(signum, frame):
    raise SystemExit(0)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Ground station: liveness + allocator")
    ap.add_argument("--n", type=int, required=True)
    ap.add_argument("--port-base", type=int, default=0)
    ap.add_argument("--log", default=None)
    ap.add_argument("--mission-s", type=float, default=config.MISSION_S)
    # timing overrides (tests); defaults come from config
    ap.add_argument("--suspect-s", type=float, default=config.SUSPECT_S)
    ap.add_argument("--lost-s", type=float, default=config.LOST_S)
    ap.add_argument("--cooldown-s", type=float, default=config.BLACKLIST_COOLDOWN_S)
    ap.add_argument("--replan-min-s", type=float, default=DEFAULT_REPLAN_MIN_S)
    # sensor footprint for lane spacing (owned by the sensor model)
    ap.add_argument("--swath-across", type=float, default=AllocParams.swath_across)
    ap.add_argument("--footprint-along", type=float, default=AllocParams.footprint_along)
    args = ap.parse_args(argv)

    params = AllocParams(swath_across=args.swath_across, footprint_along=args.footprint_along)
    log = JsonlLog(args.log)
    log.write("config", n=args.n, port_base=args.port_base, mission_s=args.mission_s,
              suspect_s=args.suspect_s, lost_s=args.lost_s, cooldown_s=args.cooldown_s,
              replan_min_s=args.replan_min_s,
              params={"swath_across": params.swath_across,
                      "footprint_along": params.footprint_along,
                      "max_slot_spacing": params.max_slot_spacing})
    signal.signal(signal.SIGTERM, _exit_on_signal)
    signal.signal(signal.SIGINT, _exit_on_signal)
    link = RadioLink(config.GROUND_ID, args.port_base)
    ground = Ground(args.n, link, log, params, args.mission_s, args.suspect_s, args.lost_s,
                    args.cooldown_s, args.replan_min_s, log_path=args.log)
    try:
        ground.run()
    finally:
        log.close()
        link.close()


if __name__ == "__main__":
    sys.exit(main())
