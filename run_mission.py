#!/usr/bin/env python3
"""Run ONE coverage mission end to end and leave a complete run directory.

    python run_mission.py --mission A --backend gazebo --n 3 --seed 1 --out runs/eval/seed_1
        [--mission-s 180] [--port-base 0] [--base-instance 0] [--no-kill] [--kill-window 60 90]

Missions
  A  At a seeded mission time in --kill-window (default [60, 90] s) a seeded victim (1..N)
     loses its whole drone at once: SIGKILL of its arducopter (Fleet.kill_vehicle) and SIGKILL
     of its agent process, back to back. With --backend fake only the agent process is killed.
     --no-kill runs the same mission without the kill (baseline).
  B  Needs a weights/priority-zone option in the ground station. Not implemented: exits 2.
  C  No kill. The radio blacks out a seeded agent for 10 s starting at a seeded time in
     --kill-window (chosen before launch because the radio needs it on its command line).
  Seeding: rng = random.Random(seed); t = rng.uniform(*window); agent = rng.randint(1, n).

Order of start: tools/wait_cool.sh (not for fake), Fleet (not for fake), radio, ground, agents.
Every child runs under `nice -n 19`, logs to <out>/<name>.log and its PID is listed in
run/pids/<out name>.pid so tools/thermal_guard.sh can pause it; the file is removed at the end.

The runner is test equipment: it reads ground.jsonl to learn mission t0 (the "start" event) and
never sends anything to the swarm.

Exit codes: 0 ok, 1 aborted (watchdog / child died / error), 2 bad usage or mission B,
3 tainted (thermal guard paused the run: run/TAINTED appeared or changed), 130 interrupted.
meta.json is written at launch and rewritten at the end, whatever happens.
"""
import argparse
import datetime
import json
import os
import random
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)

from swarm import config  # noqa: E402

PID_DIR = os.path.join(ROOT, "run", "pids")
TAINT_FILE = os.path.join(ROOT, "run", "TAINTED")
WAIT_COOL = os.path.join(ROOT, "tools", "wait_cool.sh")
START_TIMEOUT_S = 300.0     # the ground's "start" must appear this soon after the agents launch
BLACKOUT_S = 10.0
END_GRACE_S = 1.0           # keep running this long past mission_s so the last samples land
STOP_WAIT_S = 10.0          # SIGTERM -> SIGKILL grace per child
LOOP_S = 0.05


class Abort(Exception):
    """The run cannot continue; the message says why."""


class Interrupted(Exception):
    """SIGINT / SIGTERM received by the runner."""


# ------------------------------------------------------------------ small helpers
def now_iso():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")


def log(msg):
    print(f"[run_mission {time.strftime('%H:%M:%S')}] {msg}", flush=True)


def git_commit():
    """HEAD commit hash, with '-dirty' if the working tree has changes; None outside git."""
    try:
        head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True,
                              text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain"], cwd=ROOT, capture_output=True,
                               text=True, check=True).stdout.strip()
        return head + ("-dirty" if dirty else "")
    except (OSError, subprocess.CalledProcessError):
        return None


def taint_stamp():
    """mtime of run/TAINTED, or None if it does not exist."""
    try:
        return os.stat(TAINT_FILE).st_mtime
    except FileNotFoundError:
        return None


def tail(path, n=20):
    try:
        with open(path, errors="replace") as f:
            return "".join(f.readlines()[-n:])
    except OSError as e:
        return f"(cannot read {path}: {e})"


def write_json(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=1)
        f.write("\n")
    os.replace(tmp, path)


def plan_events(mission, seed, n, window, no_kill):
    """Seeded choice of the mission event. Returns (kill_agent, kill_t, blackout or None)."""
    rng = random.Random(seed)
    t = round(rng.uniform(window[0], window[1]), 3)   # ms resolution, same as on the radio CLI
    agent = rng.randint(1, n)
    if mission == "A" and not no_kill:
        return agent, t, None
    if mission == "C":
        return None, None, [agent, t, round(t + BLACKOUT_S, 3)]
    return None, None, None


# ------------------------------------------------------------------ child processes
class Children:
    """The processes this runner started itself (not the Fleet's; Fleet tracks those)."""

    def __init__(self, run_dir, pid_file):
        self.run_dir = run_dir
        self.pid_file = pid_file
        self.procs = {}          # name -> Popen, in start order
        self.logs = {}           # name -> log path

    def start(self, name, argv):
        path = os.path.join(self.run_dir, name + ".log")
        out = open(path, "w")
        env = dict(os.environ, PYTHONPATH=ROOT, PYTHONUNBUFFERED="1")
        # `nice` execs the command, so Popen.pid is the python process itself
        p = subprocess.Popen(["nice", "-n", "19"] + argv, cwd=ROOT, env=env,
                             stdout=out, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        out.close()
        self.procs[name] = p
        self.logs[name] = path
        self.write_pid_file()
        log(f"started {name} pid {p.pid}")
        return p

    def write_pid_file(self):
        os.makedirs(PID_DIR, exist_ok=True)
        pids = [str(p.pid) for p in self.procs.values() if p.poll() is None]
        with open(self.pid_file, "w") as f:
            f.write(" ".join(pids) + "\n")

    def remove_pid_file(self):
        if os.path.exists(self.pid_file):
            os.remove(self.pid_file)

    def sigkill(self, name):
        """SIGKILL one child now (no wait). Returns the wall time of the signal."""
        p = self.procs[name]
        t = time.time()
        if p.poll() is None:
            os.kill(p.pid, signal.SIGKILL)
        return t

    def first_dead(self, ignore):
        """Name of the first child (not in `ignore`) that has exited, or None."""
        for name, p in self.procs.items():
            if name not in ignore and p.poll() is not None:
                return name
        return None

    def stop(self, names, sig_stop=signal.SIGTERM):
        """SIGCONT (in case the thermal guard paused it) + sig_stop, wait, then SIGKILL."""
        live = [self.procs[n] for n in names if n in self.procs and self.procs[n].poll() is None]
        for p in live:
            for sig in (signal.SIGCONT, sig_stop):
                try:
                    os.kill(p.pid, sig)
                except ProcessLookupError:
                    pass
        deadline = time.time() + STOP_WAIT_S
        for p in live:
            try:
                p.wait(timeout=max(0.1, deadline - time.time()))
            except subprocess.TimeoutExpired:
                log(f"pid {p.pid} ignored SIGTERM, SIGKILL")
                p.kill()
                p.wait(timeout=5)


# ------------------------------------------------------------------ ground.jsonl watcher
class StartWatcher:
    """Reads ground.jsonl as it grows and returns t0 from the first "start" event."""

    def __init__(self, path):
        self.path = path
        self.offset = 0
        self.t0 = None

    def poll(self):
        if self.t0 is not None or not os.path.exists(self.path):
            return self.t0
        with open(self.path) as f:
            f.seek(self.offset)
            chunk = f.read()
        end = chunk.rfind("\n")
        if end < 0:
            return None               # no complete line yet
        self.offset += len(chunk[:end + 1].encode())
        for line in chunk[:end + 1].splitlines():
            try:
                rec = json.loads(line)
            except ValueError:
                continue
            if rec.get("event") == "start":
                self.t0 = float(rec["t0"])
                break
        return self.t0


# ------------------------------------------------------------------ the mission
class Mission:
    def __init__(self, args):
        self.a = args
        self.run_dir = os.path.abspath(args.out)
        self.name = os.path.basename(os.path.normpath(self.run_dir))
        self.fake = args.backend == "fake"
        self.children = Children(self.run_dir, os.path.join(PID_DIR, f"mission_{self.name}.pid"))
        self.fleet = None
        self.taint0 = None
        self.killed = set()       # agent ids this runner killed on purpose
        self.kill_agent, self.kill_t, self.blackout = plan_events(
            args.mission, args.seed, args.n, args.kill_window, args.no_kill)
        self.meta = {
            "seed": args.seed, "mission": args.mission, "n": args.n, "backend": args.backend,
            "t0_wall": None, "kill_agent": self.kill_agent, "kill_t_mission": self.kill_t,
            "kill_wall": None, "blackout": self.blackout, "mission_s": args.mission_s,
            "kill_window": list(args.kill_window),
            "radio": {"loss": args.loss, "delay": args.delay, "seed": args.seed},
            "port_base": args.port_base, "base_instance": args.base_instance,
            "tainted": False, "outcome": "running", "git_commit": git_commit(),
            "started": now_iso(), "finished": None,
        }

    def save_meta(self):
        write_json(os.path.join(self.run_dir, "meta.json"), self.meta)

    # ---- checks run every loop
    def check_taint(self):
        if taint_stamp() != self.taint0:
            self.meta["tainted"] = True
            raise Abort("run/TAINTED appeared or changed: the thermal guard paused the run")

    def check_children(self, t_mission):
        ignore = {f"agent_{a}" for a in self.killed}
        if t_mission is not None and t_mission >= self.a.mission_s - 2.0:
            ignore.add("ground")      # the ground ends the mission by itself at t0 + mission_s
        dead = self.children.first_dead(ignore)
        if dead:
            path = self.children.logs[dead]
            rc = self.children.procs[dead].returncode
            raise Abort(f"{dead} exited early (code {rc}); last lines of {path}:\n{tail(path)}")
        if self.fleet is not None:
            for a in range(1, self.a.n + 1):
                if a not in self.killed and not self.fleet.alive(a):
                    raise Abort(f"arducopter of agent {a} died early; see "
                                f"{self.fleet.run_dir}/s{self.fleet.instance(a)}/sitl.log:\n"
                                + tail(os.path.join(self.fleet.run_dir,
                                                    f"s{self.fleet.instance(a)}", "sitl.log")))

    # ---- launch
    def start_fleet(self):
        from sim.launch import Fleet
        self.fleet = Fleet(self.a.backend, self.a.n, os.path.join(self.run_dir, "sim"),
                           base_instance=self.a.base_instance)
        log(f"starting Fleet backend={self.a.backend} n={self.a.n}")
        self.fleet.start()
        log(f"Fleet up: {self.fleet.pids}")

    def start_radio(self):
        argv = [sys.executable, "-m", "swarm.radio", "--seed", str(self.a.seed),
                "--loss", str(self.a.loss), "--delay", str(self.a.delay),
                "--port-base", str(self.a.port_base),
                "--log", os.path.join(self.run_dir, "radio.jsonl")]
        if self.blackout:
            agent, t_start, t_end = self.blackout
            argv += ["--blackout", f"{agent}:{t_start:.3f}:{t_end:.3f}"]
        self.children.start("radio", argv)

    def start_ground(self):
        self.children.start("ground", [
            sys.executable, "-m", "swarm.ground", "--n", str(self.a.n),
            "--port-base", str(self.a.port_base), "--mission-s", str(self.a.mission_s),
            "--log", os.path.join(self.run_dir, "ground.jsonl")])

    def start_agents(self):
        for a in range(1, self.a.n + 1):
            self.children.start(f"agent_{a}", [
                sys.executable, "-m", "swarm.agent", "--id", str(a), "--n", str(self.a.n),
                "--backend", "fake" if self.fake else "mav",
                "--port-base", str(self.a.port_base),
                "--base-instance", str(self.a.base_instance),
                "--log-dir", self.run_dir])

    # ---- phases
    def wait_for_start(self):
        watcher = StartWatcher(os.path.join(self.run_dir, "ground.jsonl"))
        deadline = time.time() + START_TIMEOUT_S
        log("waiting for the ground's start event")
        while watcher.poll() is None:
            self.check_taint()
            self.check_children(None)
            if time.time() > deadline:
                raise Abort(f"no 'start' event in ground.jsonl within {START_TIMEOUT_S:.0f} s; "
                            f"last lines of ground.log:\n{tail(self.children.logs['ground'])}")
            time.sleep(LOOP_S * 4)
        self.meta["t0_wall"] = watcher.t0
        self.save_meta()
        log(f"mission t0 = {watcher.t0:.3f} (in {watcher.t0 - time.time():.1f} s)")
        return watcher.t0

    def kill_drone(self, t0):
        a = self.kill_agent
        self.killed.add(a)
        t_kill = self.children.sigkill(f"agent_{a}")
        if self.fleet is not None:
            self.fleet.kill_vehicle(a)       # SIGKILL of its arducopter, right after
        log(f"KILLED agent {a} at t_mission {t_kill - t0:.2f} (planned {self.kill_t:.2f})")
        self.children.procs[f"agent_{a}"].wait(timeout=5)
        self.children.write_pid_file()
        self.meta["kill_wall"] = t_kill
        self.save_meta()

    def fly(self, t0):
        t_end = t0 + self.a.mission_s + END_GRACE_S
        next_report = t0 + 30.0
        while True:
            now = time.time()
            if now >= t_end:
                return
            t_mission = now - t0
            self.check_taint()
            self.check_children(t_mission)
            if self.kill_agent and self.kill_agent not in self.killed and t_mission >= self.kill_t:
                self.kill_drone(t0)
            if now >= next_report:
                log(f"t_mission {t_mission:.0f} / {self.a.mission_s:.0f} s")
                next_report += 30.0
            time.sleep(LOOP_S)

    def teardown(self):
        self.children.remove_pid_file()          # so the guard cannot re-pause them meanwhile
        names = list(self.children.procs)
        agents = [n for n in names if n.startswith("agent_")]
        # agents first so they close their logs (they handle SIGINT and SIGTERM the same way)
        self.children.stop(agents, signal.SIGTERM)
        self.children.stop(["ground", "radio"])
        if self.fleet is not None:
            for pid in self.fleet.pids.values():  # undo a thermal pause before stop()
                try:
                    os.kill(pid, signal.SIGCONT)
                except ProcessLookupError:
                    pass
            self.fleet.stop()

    def run(self):
        os.makedirs(self.run_dir, exist_ok=True)
        self.save_meta()
        log(f"mission {self.a.mission} seed {self.a.seed} backend {self.a.backend} -> {self.run_dir}"
            f"; kill {self.kill_agent}@{self.kill_t} blackout {self.blackout}")
        code = 1
        launched = False
        try:
            if not self.fake:
                log("waiting for the machine to be cool (tools/wait_cool.sh)")
                subprocess.run(["bash", WAIT_COOL], check=True)
            self.taint0 = taint_stamp()      # baseline: only changes from now on taint this run
            launched = True
            if not self.fake:
                self.start_fleet()
            self.start_radio()
            self.start_ground()
            self.start_agents()
            t0 = self.wait_for_start()
            self.fly(t0)
            self.check_taint()
            self.meta["outcome"] = "ok"
            code = 0
        except Abort as e:
            first_line = str(e).splitlines()[0]      # the log tail is in the console output
            self.meta["outcome"] = "tainted" if self.meta["tainted"] else f"aborted: {first_line}"
            log(f"ABORT: {e}")
            code = 3 if self.meta["tainted"] else 1
        except Interrupted:
            self.meta["outcome"] = "interrupted"
            log("interrupted, tearing down")
            code = 130
        except Exception as e:
            self.meta["outcome"] = f"error: {e!r}"
            log(f"ERROR: {e!r}")
            raise
        finally:
            signal.signal(signal.SIGINT, signal.SIG_IGN)   # do not interrupt the teardown
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            self.teardown()
            if launched and taint_stamp() != self.taint0:
                self.meta["tainted"] = True
                self.meta["outcome"] = "tainted"
                code = 3
            self.meta["finished"] = now_iso()
            self.save_meta()
            log(f"done: {self.meta['outcome']} (exit {code})")
        return code


# ------------------------------------------------------------------ CLI
def ground_has_priority_option():
    """True if swarm.ground advertises a weights/priority-zone option (needed for mission B)."""
    r = subprocess.run([sys.executable, "-m", "swarm.ground", "--help"], cwd=ROOT,
                       capture_output=True, text=True)
    return any(w in r.stdout for w in ("--weights", "--priority"))


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="Run one coverage mission end to end.")
    ap.add_argument("--mission", choices=["A", "B", "C"], required=True)
    ap.add_argument("--backend", choices=["sitl", "gazebo", "fake"], required=True)
    ap.add_argument("--n", type=int, default=3)
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--out", required=True, help="run directory (must not exist or be empty)")
    ap.add_argument("--mission-s", type=float, default=config.MISSION_S)
    ap.add_argument("--port-base", type=int, default=0)
    ap.add_argument("--base-instance", type=int, default=0)
    ap.add_argument("--no-kill", action="store_true", help="mission A without the kill")
    ap.add_argument("--kill-window", type=float, nargs=2, default=[60.0, 90.0],
                    metavar=("T_MIN", "T_MAX"),
                    help="mission-time window for the A kill / C blackout start (default 60 90)")
    ap.add_argument("--loss", type=float, default=config.RADIO_LOSS)
    ap.add_argument("--delay", type=float, default=config.RADIO_DELAY_S)
    a = ap.parse_args(argv)
    lo, hi = a.kill_window
    if not 0 <= lo <= hi:
        ap.error("--kill-window needs 0 <= T_MIN <= T_MAX")
    if a.mission != "B" and hi + (BLACKOUT_S if a.mission == "C" else 0) > a.mission_s:
        ap.error("--kill-window must end inside the mission (and leave room for C's blackout)")
    if a.n < 1:
        ap.error("--n must be >= 1")
    return a


def main(argv=None):
    a = parse_args(argv)
    if a.mission == "B":
        if ground_has_priority_option():
            print("mission B: the ground has a priority option, but run_mission.py does not "
                  "drive it yet: not implemented", file=sys.stderr)
        else:
            print("mission B: not implemented (swarm/ground.py has no weights/priority-zone "
                  "option)", file=sys.stderr)
        return 2
    if os.path.isdir(a.out) and os.listdir(a.out):
        print(f"{a.out} exists and is not empty; refusing to mix runs "
              "(the ground would read an old ground.jsonl)", file=sys.stderr)
        return 2

    def on_signal(signum, frame):
        raise Interrupted()
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    return Mission(a).run()


if __name__ == "__main__":
    sys.exit(main())
