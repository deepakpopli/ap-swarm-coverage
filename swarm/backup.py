"""Onboard backup allocator (SWARM_BACKUP=1, config.BACKUP): an agent re-plans when the ground
is gone.

Election, each agent on its own radio view:
  - armed once it has heard the ground (an ASSIGN or START from GROUND_ID): before the first
    plan there is nothing to keep going;
  - the "boss" is the ground or a lower-id agent running a backup; the boss is gone when none
    of its ASSIGN / START has been heard for GROUND_LOST_S;
  - the agent runs a backup if the boss is gone, it is flying, and no lower id is alive
    (a FLYING SWARM_STATE received within LOST_S, the ground's own "not lost" rule).
The backup is a swarm.ground.Ground fed with this agent's received STATE, INFEASIBLE and
SIGHTING plus its own STATE, sending through this agent's radio. It starts like a restarted
ground: listen RESTART_LISTEN_S, jump EPOCH_STRIDE above every epoch heard, keep t0.
It stops when a boss is heard again (the ground came back, or a lower-id backup), when a lower
id is heard alive, or when this agent stops flying. Like every Ground with SWARM_BACKUP=1 it
overtakes: an agent flying a higher epoch makes it jump EPOCH_STRIDE above (Ground.overtake;
BACKUP.md has the convergence argument).

Logs: "backup_start" / "backup_stop" in agent_<id>.events.jsonl; the backup's ground events in
agent_<id>.ground.jsonl, the format of ground.jsonl (analysis/logs.py merges them).
"""
import os

from swarm import config, protocol
from swarm.ground import FLYING, Ground, JsonLines

FED = ("SWARM_STATE", "SWARM_INFEASIBLE", "SWARM_SIGHTING")   # what a Ground reads
TRIGGER_FIRST = "backup"         # the backup's first plan (a restarted ground logs "restart")


class LocalLink:
    """The backup Ground's link: an inbox the agent fills, sends go out on the agent's radio."""

    def __init__(self, radio):
        self.radio = radio
        self.inbox = []

    def feed(self, msg) -> None:
        self.inbox.append(msg)

    def recv_all(self) -> list:
        messages, self.inbox = self.inbox, []
        return messages

    def send(self, msg) -> None:
        self.radio.send(msg)


class Backup:
    def __init__(self, agent_id, n, radio, log_dir, events, mission_s=config.MISSION_S,
                 priority_zone=None):
        self.id = agent_id
        self.n = n
        self.radio = radio
        self.log_path = os.path.join(log_dir, f"agent_{agent_id}.ground.jsonl")
        self.events = events             # the agent's events.jsonl
        self.mission_s = mission_s
        self.priority_zone = priority_zone
        self.heard = {}                  # agent id -> (receive time of last STATE, status)
        self.boss_heard_t = None         # last ASSIGN / START of the ground or a lower id
        self.ground = None               # the running backup Ground, or None
        self.link = None
        self.log = None
        self.reported_plan = False       # first-plan liveness records written
        self.done = False                # the mission is over: never start again

    # ---- what the agent hands us
    def outranks(self, sender) -> bool:
        """The ground outranks every agent; a lower id outranks a higher one."""
        return sender == config.GROUND_ID or sender < self.id

    def on_message(self, msg, now) -> None:
        kind, sender = msg.get_type(), msg.get_srcSystem()
        if kind == "SWARM_STATE":
            self.heard[sender] = (now, protocol.STATUSES[msg.status])
        if kind in ("SWARM_ASSIGN", "SWARM_START") and self.outranks(sender):
            self.boss_heard_t = now
            if self.ground is not None:
                self.stop(now, "ground_back" if sender == config.GROUND_ID else "lower_backup")
        if self.ground is not None and kind in FED:
            self.link.feed(msg)

    def on_own(self, msg) -> None:
        """Our own STATE (and SIGHTING): the radio never returns them to the sender."""
        if self.ground is not None and msg.get_type() in FED:
            self.link.feed(msg)

    # ---- election
    def alive_ids(self, now, own_status) -> list:
        ids = [a for a, (t, status) in self.heard.items()
               if now - t < config.LOST_S and status in FLYING]
        return sorted(ids + ([self.id] if own_status in FLYING else []))

    def should_run(self, now, own_status) -> bool:
        if self.done or self.boss_heard_t is None:
            return False
        if now - self.boss_heard_t < config.GROUND_LOST_S:
            return False
        alive = self.alive_ids(now, own_status)
        return bool(alive) and alive[0] == self.id

    # ---- one tick
    def step(self, now, own_status, t0) -> None:
        if self.ground is None:
            if self.should_run(now, own_status):
                self.start(now, own_status, t0)
            return
        if not self.should_run(now, own_status):
            self.stop(now, "outranked" if own_status in FLYING else "not_flying")
            return
        if not self.ground.step(now):
            self.done = True
            self.stop(now, "mission_end")
            return
        if self.ground.planned_at is not None and not self.reported_plan:
            self.reported_plan = True
            self.log_left_out(now)

    def log_left_out(self, now) -> None:
        """Agents this agent heard before but not in the first plan: the backup declares them
        lost here (a restarted ground has never heard them, so it never would)."""
        for a in sorted(set(self.heard) - set(self.ground.assignments)):
            self.log.write(now, "liveness", agent=a, status=self.heard[a][1],
                           reason="backup_plan", **{"from": "alive", "to": "lost"})

    # ---- start and stop
    def start(self, now, own_status, t0) -> None:
        self.link = LocalLink(self.radio)
        self.log = JsonLines(self.log_path)
        self.ground = Ground(self.n, self.link, self.log, self.mission_s, self.priority_zone,
                             previous_t0=t0)
        self.ground.restart_trigger = TRIGGER_FIRST
        self.reported_plan = False
        silent_s = round(now - self.boss_heard_t, 3)
        self.events.write(now, "backup_start", boss_silent_s=silent_s,
                          alive=self.alive_ids(now, own_status), t0=t0)

    def stop(self, now, reason) -> None:
        epoch = self.ground.epoch
        self.log.write(now, "end", epoch=epoch)
        self.log.close()
        self.ground = self.link = self.log = None
        self.events.write(now, "backup_stop", reason=reason, epoch=epoch)

    def close(self, now) -> None:
        if self.ground is not None:
            self.stop(now, "agent_exit")

    @property
    def running(self) -> bool:
        return self.ground is not None
