"""Onboard agent process, one per drone.

    python -m swarm.agent --id K --n N --backend mav|fake --port-base P --base-instance I --log-dir DIR

Layering per control tick (CONTROL_HZ = 10 Hz exactly, on a fixed schedule):
    coverage controller -> nominal (vx, vy, z) -> safety filter -> safe (vx, vy, z) -> vehicle.send_setpoint
Every setpoint after takeoff goes through the safety filter, including pre-START hold.

Sequence: connect -> arm_and_takeoff(ALT_NOMINAL) in a background thread (the main loop keeps
broadcasting STATE "takeoff" and only reads vehicle.state() meanwhile; it does not call poll()) ->
"ready": hold through the filter (which puts each agent on its own altitude layer) -> once START has
been received, wall time >= its t0, and an ASSIGN is held: cover.
Autopilot link dead (state().link_ok False) for > 2 s: exit at once, sending nothing.
Ground silent: keep flying the last assignment (only the deaf rule stops us, and neighbours' STATE
messages keep us from being deaf).

Logs (onboard, the only input for offline metrics), CSV flushed every control tick (so a SIGKILLed
agent loses at most the row being written); SIGTERM and SIGINT both end the loop cleanly:
    DIR/agent_<id>.csv           one row per control tick, header CSV_HEADER
    DIR/agent_<id>.events.jsonl  discrete events, each with t_wall and t_mission
"""
import argparse
import json
import os
import signal
import sys
import threading
import time
from typing import Optional

from swarm import config as C
from swarm.coverage import CoverageController
from swarm.safety import DEAF, LAYERED, SafetyFilter, SafetyParams

CSV_HEADER = ("t_wall,t_mission,x,y,z,vx,vy,vz,yaw,status,epoch,slot,cmd_vx,cmd_vy,cmd_z,"
              "nom_vx,nom_vy,min_nbr_dist,min_nbr_age,flags")
LINK_DEAD_S = 2.0
MAIN_HZ = 50.0
STATS_EVERY_S = 10.0


def _f(x, nd=3) -> str:
    return "" if x is None else f"{x:.{nd}f}"


class AgentLog:
    """CSV + JSONL writer. Both flushed on every write (10 rows/s: cheap, and a SIGKILL loses nothing)."""

    def __init__(self, log_dir: str, agent_id: int):
        os.makedirs(log_dir, exist_ok=True)
        self.csv = open(os.path.join(log_dir, f"agent_{agent_id}.csv"), "w", buffering=1 << 16)
        self.csv.write(CSV_HEADER + "\n")
        self.csv.flush()
        self.ev = open(os.path.join(log_dir, f"agent_{agent_id}.events.jsonl"), "w")

    def row(self, fields, now: float) -> None:
        self.csv.write(",".join(fields) + "\n")
        self.csv.flush()

    def event(self, rec: dict) -> None:
        self.ev.write(json.dumps(rec, separators=(",", ":")) + "\n")
        self.ev.flush()

    def close(self) -> None:
        for fh in (self.csv, self.ev):
            try:
                fh.flush()
                fh.close()
            except Exception:
                pass


class Agent:
    """All logic is in step(now); run() is the real-time loop. Tests drive step() in simulated time."""

    def __init__(self, agent_id: int, n: int, vehicle, link, log_dir: str,
                 params: SafetyParams = SafetyParams()):
        self.id, self.n = agent_id, n
        self.vehicle, self.link = vehicle, link
        self.log = AgentLog(log_dir, agent_id)
        self.coverage = CoverageController(agent_id)
        self.safety = SafetyFilter(agent_id, params)
        self.phase = "ground"            # ground -> takeoff -> ready -> mission
        self.t0: Optional[float] = None  # START wall time (mission t = 0)
        self.status = "ground"
        self._takeoff: Optional[threading.Thread] = None
        self._takeoff_err: Optional[BaseException] = None
        self._link_bad_since: Optional[float] = None
        self._next_ctrl: Optional[float] = None
        self._next_state: Optional[float] = None
        self._last_tick: Optional[float] = None
        self._periods = []
        self._overruns = 0
        self._next_stats: Optional[float] = None
        self._last_slot = -1

    # ---- helpers ----------------------------------------------------------------------------
    def t_mission(self, now: float) -> Optional[float]:
        return None if self.t0 is None else now - self.t0

    def event(self, now: float, rec: dict) -> None:
        tm = self.t_mission(now)
        out = {"t_wall": round(now, 3), "t_mission": None if tm is None else round(tm, 3)}
        out.update(rec)
        self.log.event(out)

    # ---- lifecycle --------------------------------------------------------------------------
    def start(self, now: float) -> None:
        """Connect and start the (blocking) takeoff in a background thread."""
        self.vehicle.connect()
        self.event(now, {"event": "connected", "id": self.id, "n": self.n, "layer": self.safety.layer})
        self.phase = "takeoff"
        self._next_state = now
        self._link_bad_since = None

        def _go():
            try:
                self.vehicle.arm_and_takeoff(C.ALT_NOMINAL)
            except BaseException as e:   # reported by step()
                self._takeoff_err = e
        self._takeoff = threading.Thread(target=_go, name=f"takeoff{self.id}", daemon=True)
        self._takeoff.start()

    def step(self, now: float) -> bool:
        """One main-loop iteration. Returns False when the agent must exit."""
        if self.phase in ("ready", "mission"):
            self.vehicle.poll()
        st = self.vehicle.state()
        # None = no position fix yet (not a dead link; a dead link during takeoff makes
        # arm_and_takeoff raise, which exits below). Afterwards link_ok False for > 2 s = gone.
        if st is not None and not st.link_ok:
            if self._link_bad_since is None:
                self._link_bad_since = now
            elif now - self._link_bad_since > LINK_DEAD_S:
                self.event(now, {"event": "autopilot_link_lost", "exit": True})
                return False
        else:
            self._link_bad_since = None

        for msg in self.link.recv_all():
            self._handle(msg, now)

        if self.phase == "takeoff" and self._takeoff is not None and not self._takeoff.is_alive():
            if self._takeoff_err is not None:
                self.event(now, {"event": "takeoff_failed", "error": repr(self._takeoff_err)})
                return False
            self.phase = "ready"
            self.event(now, {"event": "takeoff_done", "z": None if st is None else round(st.pos[2], 2)})
            self._next_ctrl = now
            self._next_stats = now + STATS_EVERY_S

        if self.phase == "ready" and self.t0 is not None and now >= self.t0 and self.coverage.epoch is not None:
            self.phase = "mission"
            self.event(now, {"event": "mission_start", "epoch": self.coverage.epoch})

        if self._next_state is not None and now >= self._next_state and st is not None:
            self._broadcast_state(st)
            self._next_state += 1.0 / C.STATE_HZ
            if self._next_state < now:
                self._next_state = now + 1.0 / C.STATE_HZ

        if self.phase in ("ready", "mission") and now >= self._next_ctrl and st is not None:
            self._control_tick(now, st)
            self._next_ctrl += 1.0 / C.CONTROL_HZ
            if self._next_ctrl < now:          # we fell more than one period behind: resync
                self._overruns += 1
                self._next_ctrl = now + 1.0 / C.CONTROL_HZ

        if self._next_stats is not None and now >= self._next_stats:
            self._log_stats(now)
            self._next_stats += STATS_EVERY_S
        return True

    # ---- messages ---------------------------------------------------------------------------
    def _handle(self, msg: dict, now: float) -> None:
        src = msg.get("src")
        if src == self.id:
            return
        self.safety.heard(now)
        mtype = msg.get("type")
        if mtype == "STATE":
            self.safety.update_neighbour(src, msg["pos"], msg["vel"], msg.get("status", ""),
                                         t_sent=float(msg.get("t", now)), t_recv=now)
        elif mtype == "START":
            if self.t0 is None:
                self.t0 = float(msg["t0"])
                self.event(now, {"event": "start", "t0": self.t0})
        elif mtype == "ASSIGN":
            if msg.get("agent") == self.id:
                ev = self.coverage.on_assign(msg, now)
                if ev:
                    self.event(now, ev)

    def _broadcast_state(self, st) -> None:
        if self.phase == "takeoff":
            status = "takeoff" if st.armed else "ground"
        else:
            status = self.status
        self.link.send("all", "STATE",
                       pos=[round(v, 2) for v in st.pos], vel=[round(v, 2) for v in st.vel],
                       epoch=-1 if self.coverage.epoch is None else self.coverage.epoch,
                       slot=self.coverage.idx if self.coverage.slots else -1, status=status)

    # ---- control ----------------------------------------------------------------------------
    def _control_tick(self, now: float, st) -> None:
        if self._last_tick is not None:
            self._periods.append(now - self._last_tick)
        self._last_tick = now

        holding = True
        slot = -1
        if self.phase == "mission":
            cov = self.coverage.step(now, st.pos)
            for ev in cov.events:
                self.event(now, ev)
            nominal = (cov.vx, cov.vy, cov.z)
            holding, slot = cov.holding, cov.slot
        else:
            nominal = (0.0, 0.0, C.ALT_NOMINAL)

        cmd = self.safety.step(now, st.pos, st.vel, nominal)
        self.vehicle.send_setpoint(cmd.vx, cmd.vy, cmd.z)

        for ev in cmd.events:
            self.event(now, ev)
        if cmd.infeasible and self.phase == "mission":
            ev = self.coverage.report_safety(now, st.pos)
            if ev:
                self.event(now, ev)
        body = self.coverage.infeasible_message(now)
        if body is not None:
            self.link.send(C.GROUND_ID, "INFEASIBLE", **body)
            self.event(now, dict(body, event="infeasible_sent"))

        if self.phase != "mission":
            self.status = "hold" if cmd.mode == DEAF else "ready"
        elif cmd.mode == DEAF:
            self.status = "hold"
        elif cmd.mode == LAYERED and self.safety.layer_reason == "fallback":
            self.status = "fallback"
        elif holding:
            self.status = "hold"
        else:
            self.status = "cover"

        tm = self.t_mission(now)
        x, y, z = st.pos
        vx, vy, vz = st.vel
        self.log.row([
            _f(now), _f(tm), _f(x, 2), _f(y, 2), _f(z, 2), _f(vx, 2), _f(vy, 2), _f(vz, 2), _f(st.yaw, 3),
            self.status, "" if self.coverage.epoch is None else str(self.coverage.epoch), str(slot),
            _f(cmd.vx, 2), _f(cmd.vy, 2), _f(cmd.z, 2), _f(nominal[0], 2), _f(nominal[1], 2),
            _f(cmd.min_nbr_dist, 2), _f(cmd.min_nbr_age, 3), "|".join(cmd.flags)], now)

    def _log_stats(self, now: float) -> None:
        p = sorted(self._periods)
        self._periods = []
        if not p:
            return
        p99 = p[min(len(p) - 1, int(0.99 * len(p)))]
        self.event(now, {"event": "loop_stats", "n": len(p), "mean_ms": round(1000 * sum(p) / len(p), 2),
                         "p99_ms": round(1000 * p99, 2), "max_ms": round(1000 * p[-1], 2),
                         "overruns": self._overruns})

    # ---- real-time loop ---------------------------------------------------------------------
    def run(self, clock=time.time, sleep=time.sleep) -> int:
        """Real-time main loop at MAIN_HZ. Returns the process exit code."""
        code = 0
        try:
            self.start(clock())          # inside try: a SIGTERM while connecting still closes the logs
            nxt = clock()
            while True:
                now = clock()
                if not self.step(now):
                    code = 2
                    break
                t = clock()
                while nxt <= t:                     # next main step on the 50 Hz grid
                    nxt += 1.0 / MAIN_HZ
                # also wake exactly for the next control tick so setpoints go out at 10 Hz sharp
                wake = nxt if self._next_ctrl is None else min(nxt, self._next_ctrl)
                if wake > t:
                    sleep(wake - t)
        except KeyboardInterrupt:
            code = 0
        finally:
            self.log.close()
        return code


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--id", type=int, required=True)
    ap.add_argument("--n", type=int, required=True)
    ap.add_argument("--backend", choices=("mav", "fake"), default="mav")
    ap.add_argument("--port-base", type=int, default=0)
    ap.add_argument("--base-instance", type=int, default=0)
    ap.add_argument("--log-dir", default="run")
    a = ap.parse_args(argv)

    if a.backend == "mav":
        from swarm.mav_vehicle import MavVehicle
        instance = a.base_instance + a.id - 1            # same mapping as sim/launch.py
        vehicle = MavVehicle(a.id, port=5760 + 10 * instance)
    else:
        from swarm.fake_vehicle import FakeVehicle
        x, y = C.start_xy(a.id - 1, a.n)
        vehicle = FakeVehicle(a.id, start=(x, y, 0.0))
    from swarm.net import RadioLink
    link = RadioLink(a.id, port_base=a.port_base)

    # SIGTERM ends the loop the same way as Ctrl-C (run() closes the logs in its finally).
    def _on_term(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_term)
    return Agent(a.id, a.n, vehicle, link, a.log_dir).run()


if __name__ == "__main__":
    sys.exit(main())
