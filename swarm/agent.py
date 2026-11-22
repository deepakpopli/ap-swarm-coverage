"""Onboard agent process, one per drone.

    python -m swarm.agent --id K --n N --backend mav|fake --port-base P --log-dir DIR
                          [--obstacle x,y,r[,t]]

Every tick at CONTROL_HZ: coverage -> safety filter -> one setpoint to the own autopilot.
Status, broadcast in STATE:
  takeoff   listen LISTEN_S, wait for clear_for_takeoff(), climb straight up to the hold altitude
  ready     at the hold altitude, no assignment yet
  cover     flying the assignment, from the first ASSIGN on (START only sets t0 for the logs;
            t_mission is logged from then on, negative during the warm-up)
  hold      the filter keeps us at the hold altitude (deaf, or nothing to fly)
  failsafe  the autopilot left GUIDED for > GUIDED_LOST_S: no more setpoints, STATE goes on
If the own autopilot link dies the agent sends nothing more and exits after LINK_DEAD_S, so
neighbours never get a stale position with a fresh time stamp.
Logs: agent_<id>.csv (protocol.AGENT_CSV_COLUMNS, a row per tick) and agent_<id>.events.jsonl
(its "takeoff_done" event holds the autopilot parameters as read back).
--obstacle: test hook, a static keep-out disc only this agent knows about (SafetyFilter),
from mission time t on if given (else from the start).
"""
import argparse
import csv
import json
import os
import signal
import sys
import threading
import time
from typing import Optional

from swarm import config, protocol
from swarm.coverage import CoverageController
from swarm.safety import SafetyFilter, clear_for_takeoff

LISTEN_S = 1.0        # hear the neighbours before deciding that take-off is clear
GUIDED_LOST_S = 1.0   # not GUIDED for longer than this: failsafe
LINK_DEAD_S = 2.0     # own autopilot silent for longer than this: exit
STATE_EVERY = round(config.CONTROL_HZ / config.STATE_HZ)   # ticks between STATE messages


class JsonlLog:
    """One JSON object per line, flushed at once, so a killed process loses nothing."""

    def __init__(self, path: str):
        self.file = open(path, "w")

    def write(self, t_wall: float, event: str, **fields) -> None:
        record = {"t_wall": round(t_wall, 3), "event": event, **fields}
        self.file.write(json.dumps(record) + "\n")
        self.file.flush()

    def close(self) -> None:
        self.file.close()


def _num(x: Optional[float], digits: int = 2) -> str:
    return "" if x is None else f"{x:.{digits}f}"


class Agent:
    """tick(now) holds all logic; run() calls it in real time, tests in simulated time."""

    def __init__(self, agent_id: int, vehicle, link, log_dir: str):
        self.id = agent_id
        self.vehicle = vehicle
        self.link = link
        self.coverage = CoverageController()
        self.safety = SafetyFilter(agent_id)
        self.status = "takeoff"
        self.t0: Optional[float] = None              # mission t = 0, from START
        self.t_listen: Optional[float] = None        # first tick: start of listening
        self.takeoff_thread: Optional[threading.Thread] = None
        self.takeoff_error: Optional[Exception] = None
        self.params: Optional[dict] = None           # read back from the autopilot
        self.not_guided_since: Optional[float] = None
        self.link_down_since: Optional[float] = None
        self.ticks = 0
        os.makedirs(log_dir, exist_ok=True)
        self.csv_file = open(os.path.join(log_dir, f"agent_{agent_id}.csv"), "w", newline="")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(protocol.AGENT_CSV_COLUMNS)
        self.events = JsonlLog(os.path.join(log_dir, f"agent_{agent_id}.events.jsonl"))
        self.obstacle = None                         # (x, y, radius, t_mission), test hook

    def close(self) -> None:
        self.csv_file.close()
        self.events.close()

    # ---- one tick -----------------------------------------------------------------------
    def tick(self, now: float) -> bool:
        """One control period at wall time `now`. Returns False when the agent must exit."""
        if not self._taking_off():          # MavVehicle reads its link itself while climbing
            self.vehicle.poll()
        st = self.vehicle.state()
        if st is None:                      # no position from the autopilot yet
            return True
        if not st.link_ok:
            return self._link_down(now)
        if self.link_down_since is not None:
            self.events.write(now, "autopilot_link_back")
            self.link_down_since = None
        for msg in self.link.recv_all():
            self._on_message(msg, now, st)
        if self.ticks % STATE_EVERY == 0:
            self._send_state(st)
        self.ticks += 1
        if self.obstacle is not None and self.t0 is not None and now >= self.t0 + self.obstacle[3]:
            self.safety.add_obstacle(*self.obstacle[:3])
            self.events.write(now, "obstacle", xy=list(self.obstacle[:2]))
            self.obstacle = None
        nominal = cmd = None
        if self.status == "takeoff":
            if not self._takeoff(now, st):
                return False
        elif not self._failsafe(now, st):
            nominal, cmd = self._control(now, st)
        self._log_row(now, st, nominal, cmd)
        return True

    def _on_message(self, msg, now: float, st) -> None:
        self.safety.heard(now)
        kind = msg.get_type()
        if kind == "SWARM_STATE":
            self.safety.update_neighbour(msg.get_srcSystem(), (msg.x, msg.y, msg.z),
                                         (msg.vx, msg.vy, msg.vz), msg.time_usec / 1e6,
                                         protocol.STATUSES[msg.status])
        elif kind == "SWARM_START" and msg.t0_usec / 1e6 != self.t0:
            self.t0 = msg.t0_usec / 1e6
            self.events.write(now, "start", t0=self.t0)
        elif kind == "SWARM_ASSIGN":
            event = self.coverage.on_assign(msg.epoch, protocol.assign_slots(msg),
                                            msg.loop_from, st.pos, st.vel)
            if event is not None:
                self.events.write(now, **event)

    def _send_state(self, st) -> None:
        self.link.send(protocol.state(st.t, st.pos, st.vel,
                                      -1 if self.coverage.epoch is None else self.coverage.epoch,
                                      self.coverage.idx, self.status))

    # ---- phases ---------------------------------------------------------------------------
    def _taking_off(self) -> bool:
        return self.takeoff_thread is not None and self.takeoff_thread.is_alive()

    def _takeoff(self, now: float, st) -> bool:
        """Listen, wait until nobody is close, climb in a thread (it blocks). False on failure."""
        if self.takeoff_thread is None:
            if self.t_listen is None:
                self.t_listen = now
            if now - self.t_listen >= LISTEN_S and clear_for_takeoff(st.pos, self.safety.nbrs):
                self.takeoff_thread = threading.Thread(target=self._climb, daemon=True)
                self.takeoff_thread.start()
                self.events.write(now, "takeoff_start", alt=self.safety.hold_z)
            return True
        if self.takeoff_thread.is_alive():
            return True
        if self.takeoff_error is not None:
            self.events.write(now, "takeoff_failed", error=repr(self.takeoff_error))
            return False
        self.status = "ready"
        self.events.write(now, "takeoff_done", z=round(st.pos[2], 2), params=self.params)
        return True

    def _climb(self) -> None:
        try:
            self.params = self.vehicle.arm_and_takeoff(self.safety.hold_z)
        except Exception as e:               # reported by _takeoff() on the main thread
            self.takeoff_error = e

    def _failsafe(self, now: float, st) -> bool:
        """True once the autopilot has been out of GUIDED for GUIDED_LOST_S (permanent)."""
        if self.status == "failsafe":
            return True
        if st.guided:
            self.not_guided_since = None
            return False
        if self.not_guided_since is None:
            self.not_guided_since = now
        if now - self.not_guided_since <= GUIDED_LOST_S:
            return False
        self.status = "failsafe"
        self.events.write(now, "failsafe", z=round(st.pos[2], 2))
        return True

    def _link_down(self, now: float) -> bool:
        """Own autopilot gone: send nothing, exit after LINK_DEAD_S."""
        if self.link_down_since is None:
            self.link_down_since = now
            self.events.write(now, "autopilot_link_down")
        if now - self.link_down_since > LINK_DEAD_S:
            self.events.write(now, "autopilot_link_lost", exit=True)
            return False
        return True

    def _control(self, now: float, st):
        """Coverage -> safety filter -> setpoint. Returns (nominal, safe command) for the log."""
        nominal = self.coverage.step(now, st.pos, self.safety.is_deaf(now))
        cmd = self.safety.step(now, st.pos, st.vel, (nominal.vx, nominal.vy), nominal.idle)
        self.vehicle.send_setpoint(cmd.vx, cmd.vy, cmd.z)
        for event in nominal.events + cmd.events:
            self.events.write(now, **event)
        body = self.coverage.infeasible_due(now)
        if body is not None:
            self.link.send(protocol.infeasible(body["epoch"], body["slot"], body["slot_xy"]))
        if not cmd.holding:
            self.status = "cover"
        elif self.coverage.epoch is None and "deaf" not in cmd.flags:
            self.status = "ready"
        else:
            self.status = "hold"
        return nominal, cmd

    def _log_row(self, now: float, st, nominal, cmd) -> None:
        t_mission = None if self.t0 is None else now - self.t0
        row = [_num(now, 3), _num(t_mission, 3)]
        row += [_num(v) for v in st.pos + st.vel] + [_num(st.yaw, 3)]
        row += [self.status, "" if self.coverage.epoch is None else self.coverage.epoch,
                self.coverage.idx]
        if cmd is None:                           # no setpoint this tick
            row += [""] * 8
        else:
            row += [_num(nominal.vx), _num(nominal.vy), _num(cmd.vx), _num(cmd.vy), _num(cmd.z),
                    _num(cmd.nbr_dist), _num(cmd.nbr_age, 3), "|".join(cmd.flags)]
        self.csv.writerow(row)
        self.csv_file.flush()

    # ---- real time ----------------------------------------------------------------------
    def run(self) -> int:
        """Tick at CONTROL_HZ on the monotonic clock until told to stop. Returns the exit code."""
        period_s = 1.0 / config.CONTROL_HZ
        next_tick = time.monotonic()
        try:
            self.vehicle.connect()
            while self.tick(time.time()):
                next_tick += period_s
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:                         # fell behind: do not try to catch up
                    next_tick = time.monotonic()
            return 2
        except KeyboardInterrupt:
            return 0
        finally:
            self.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="One swarm agent.")
    ap.add_argument("--id", type=int, required=True)
    ap.add_argument("--n", type=int, required=True)
    ap.add_argument("--backend", choices=("mav", "fake"), default="mav")
    ap.add_argument("--port-base", type=int, default=0)
    ap.add_argument("--log-dir", default="run")
    ap.add_argument("--obstacle", default=None,
                    help="test hook: x,y,radius in metres[,from mission time in s]")
    args = ap.parse_args(argv)
    if args.backend == "mav":
        from swarm.mav_vehicle import MavVehicle
        vehicle = MavVehicle(args.id)
    else:
        from sim.fake_vehicle import FakeVehicle
        x, y = config.start_xy(args.id - 1, args.n)
        vehicle = FakeVehicle(args.id, start=(x, y, 0.0))
    from swarm.net import RadioLink
    link = RadioLink(args.id, port_base=args.port_base)

    def stop(signum, frame):                 # SIGTERM ends the loop like Ctrl-C
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    agent = Agent(args.id, vehicle, link, args.log_dir)
    if args.obstacle:
        x, y, radius_m, *t = (float(v) for v in args.obstacle.split(","))
        if t:
            agent.obstacle = (x, y, radius_m, t[0])
        else:
            agent.safety.add_obstacle(x, y, radius_m)
    return agent.run()


if __name__ == "__main__":
    sys.exit(main())
