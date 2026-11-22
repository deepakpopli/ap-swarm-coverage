#!/usr/bin/env python3
"""Run ONE mission: a fleet, a radio, a ground station, N agents and a schedule of timed faults.

    python run_mission.py --mission A|B|C|ref --backend gazebo|sitl|fake --n 3 --seed S --out DIR
                          [--scenario NAME] [--mission-s 180] [--zone xmin,ymin,xmax,ymax]
Environment: SWARM_ALT (cruise altitude, m; default 50).

The runner is test equipment: it starts processes, injects faults at mission times and stops
everything. It never sends anything to the swarm; it learns t0 by re-reading ground.jsonl.

A fault is (t_mission, action, target):
  kill sitl:<id>    SIGKILL the arducopter only; the agent must notice and go silent by itself
  kill agent:<id>   SIGKILL the onboard process only; the autopilot's failsafe lands the vehicle
  kill ground, start ground (same log file), kill radio, start radio
The fake backend has no arducopter, so there "kill sitl" kills the agent (its vehicle lives in it).

Missions: A = seeded kill of a seeded victim at t in [60, 90]; ref = no fault; B = priority zone
(default: the 60 x 60 m square in the middle) plus the A kill; C = seeded 10 s radio blackout.
Scenarios: see SCENARIOS. Exit code 0 = valid run, 1 = invalid (meta.json "outcome" says why).
"""
import argparse
import json
import os
import random
import signal
import subprocess
import sys
import time

from sim import launch
from swarm import config
from swarm.allocator import allocate

ROOT = os.path.dirname(os.path.abspath(__file__))
TAINT_FILE = os.path.join(ROOT, "run", "TAINTED")
START_TIMEOUT_S = 400.0     # agents launched -> START: take-off, assignment, first loop slot
END_GRACE_S = 1.0           # run on past the mission end so the last log rows land
STOP_WAIT_S = 10.0          # SIGTERM -> SIGKILL
LOOP_S = 0.2
OBSTACLE_RADIUS_M = 10.0    # > 2 x SLOT_REACH_M: the agent cannot "fly past" the slot either
MIDDLE_ZONE = (120.0, 120.0, 180.0, 180.0)

# scenario -> (the mission it modifies, what it tests)
SCENARIOS = {
    "swap": ("A", "re-plan with the lane groups reversed at 30 s: agents cross at cruise"),
    "obstacle": ("A", "an unknown obstacle on a loop slot: infeasible, blacklist, re-plan"),
    "blackout13": ("C", "a 13 s blackout, longer than LOST_S: declared lost, then rejoins"),
    "mute": ("A", "one agent's transmitter fails from 60 s to the end (it still hears)"),
    "agentdeath": ("A", "kill the victim's agent process instead of its autopilot"),
    "groundrestart": ("A", "ground down from 40 s to 55 s, then the Mission A kill"),
    "radiooutage": ("A", "the radio is down from 60 s to 70 s"),
    "groundloss": ("A", "ground down from 40 s for good, then the Mission A kill"),
}


class Invalid(Exception):
    """The run cannot count; the message says why."""


def log(text):
    print(f"[run_mission {time.strftime('%H:%M:%S')}] {text}", flush=True)


def plan(args):
    """The faults and the extra process options of this mission. Returns a dict."""
    rng = random.Random(args.seed)      # the same two draws as always, so seeds keep meaning
    t_event = round(rng.uniform(60.0, 90.0), 3)
    agent = rng.randint(1, args.n)
    out = {"faults": [], "ground": [], "radio": [], "agents": {}, "kill_agent": None,
           "blackout": None, "mute": None, "priority_zone": None}
    kill = (t_event, "kill", f"sitl:{agent}")
    if args.mission == "A" and args.scenario in (None, "groundrestart", "groundloss"):
        out["faults"].append(kill)
    if args.mission == "B":
        out["priority_zone"] = list(args.zone or MIDDLE_ZONE)
        out["ground"] += ["--priority-zone", ",".join(str(v) for v in out["priority_zone"])]
        out["faults"].append(kill)
    if args.mission == "C":
        length_s = 13.0 if args.scenario == "blackout13" else 10.0
        out["blackout"] = {"agent": agent, "t0": t_event, "t1": t_event + length_s}
        out["radio"] += ["--blackout", f"{agent}:{t_event}:{t_event + length_s}"]
    if args.scenario == "swap":
        out["ground"] += ["--swap-at", "30"]
    if args.scenario == "obstacle":
        x, y = obstacle_slot(agent, args.n)
        out["agents"][agent] = ["--obstacle", f"{x},{y},{OBSTACLE_RADIUS_M}"]
        out["obstacle"] = {"agent": agent, "xy": [x, y], "radius_m": OBSTACLE_RADIUS_M}
    if args.scenario == "mute":
        out["mute"] = {"agent": agent, "t0": 60.0, "t1": args.mission_s}
        out["radio"] += ["--mute", f"{agent}:60:{args.mission_s}"]
    if args.scenario == "agentdeath":
        out["faults"].append((t_event, "kill", f"agent:{agent}"))
    if args.scenario in ("groundrestart", "groundloss"):
        out["faults"].insert(0, (40.0, "kill", "ground"))
    if args.scenario == "groundrestart":
        out["faults"].insert(1, (55.0, "start", "ground"))
    if args.scenario == "radiooutage":
        out["faults"] += [(60.0, "kill", "radio"), (70.0, "start", "radio")]
    if args.backend == "fake":     # no arducopter: the fake vehicle dies with its agent
        out["faults"] = [(t, a, target.replace("sitl:", "agent:"))
                         for t, a, target in out["faults"]]
    for _, action, target in out["faults"]:
        if action == "kill" and ":" in target:
            out["kill_agent"] = int(target.split(":")[1])
    return out


def obstacle_slot(agent, n):
    """The last loop slot of the agent's first plan (the ground plans from the start line)."""
    starts = {a: config.start_xy(a - 1, n) for a in range(1, n + 1)}
    return allocate(list(starts), starts, set())[agent].slots[-1]


def read_t0(ground_log):
    """t0 of the first "start" record in ground.jsonl, or None."""
    if not os.path.exists(ground_log):
        return None
    with open(ground_log) as f:
        for line in f:
            if line.endswith("\n") and json.loads(line)["event"] == "start":
                return json.loads(line)["t0"]
    return None


def read_params(run_dir, n):
    """The autopilot parameters each agent read back after setting them (None for fake)."""
    params = {}
    for a in range(1, n + 1):
        with open(os.path.join(run_dir, f"agent_{a}.events.jsonl")) as f:
            for line in f:
                record = json.loads(line)
                if record["event"] == "takeoff_done":
                    params[a] = record["params"]
    return params


def git_commit():
    head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True,
                          text=True).stdout.strip()
    dirty = subprocess.run(["git", "status", "--porcelain", "--untracked-files=no"], cwd=ROOT,
                           capture_output=True, text=True).stdout.strip()
    return head + ("-dirty" if dirty else "")


def taint_mtime():
    return os.stat(TAINT_FILE).st_mtime if os.path.exists(TAINT_FILE) else None


class Mission:
    def __init__(self, args, faults_plan):
        self.args = args
        self.plan = faults_plan
        self.out = os.path.abspath(args.out)
        self.fleet = None
        self.procs = {}             # name -> Popen of the processes we started ourselves
        self.starts = {}            # name -> how often it was started (for its log name)
        self.may_exit = set()       # names that may end without it being an error
        self.taint0 = taint_mtime()
        self.meta = {"mission": args.mission, "scenario": args.scenario, "seed": args.seed,
                     "n": args.n, "backend": args.backend, "alt_cruise_m": config.ALT_CRUISE_M,
                     "lost_after_s": config.LOST_S, "wp_spd_ms": config.WP_SPD_MS,
                     "mission_s": args.mission_s, "t0_wall": None,
                     "faults": [{"t_mission": t, "action": a, "target": target, "wall": None}
                                for t, a, target in faults_plan["faults"]],
                     "kill_agent": faults_plan["kill_agent"], "kill_wall": None,
                     "blackout": faults_plan["blackout"], "mute": faults_plan["mute"],
                     "priority_zone": faults_plan["priority_zone"],
                     "obstacle": faults_plan.get("obstacle"), "autopilot_params": None,
                     "rtf": None, "radio": {"loss": config.RADIO_LOSS,
                                            "delay": config.RADIO_DELAY_S, "seed": args.seed},
                     "outcome": "running", "tainted": False, "git_commit": git_commit(),
                     "wall_s": None}

    def save_meta(self):
        with open(os.path.join(self.out, "meta.json"), "w") as f:
            json.dump(self.meta, f, indent=1)

    # ---- processes
    def start(self, name, argv):
        self.starts[name] = self.starts.get(name, 0) + 1
        log_name = name if self.starts[name] == 1 else f"{name}.{self.starts[name]}"
        process = launch.start_process([sys.executable, "-m"] + argv,
                                       os.path.join(self.out, log_name + ".log"), cwd=ROOT)
        self.procs[name] = process
        self.may_exit.discard(name)
        launch.register_pid(f"run_{name}", process.pid)

    def start_radio(self):
        self.start("radio", ["sim.radio", "--n", str(self.args.n), "--seed", str(self.args.seed),
                             "--log", os.path.join(self.out, "radio.jsonl")] + self.plan["radio"])

    def start_ground(self):
        self.start("ground", ["swarm.ground", "--n", str(self.args.n),
                              "--mission-s", str(self.args.mission_s),
                              "--log", os.path.join(self.out, "ground.jsonl")]
                   + self.plan["ground"])

    def start_agent(self, a):
        backend = "fake" if self.args.backend == "fake" else "mav"
        self.start(f"agent_{a}", ["swarm.agent", "--id", str(a), "--n", str(self.args.n),
                                  "--backend", backend, "--log-dir", self.out]
                   + self.plan["agents"].get(a, []))

    def kill(self, name):
        self.may_exit.add(name)
        self.procs[name].kill()
        self.procs[name].wait()
        launch.unregister_pids(f"run_{name}")

    def apply(self, fault):
        """Inject one fault now and record when."""
        action, target = fault["action"], fault["target"]
        fault["wall"] = time.time()
        log(f"t = {fault['wall'] - self.meta['t0_wall']:.1f} s: {action} {target}")
        if target.startswith("sitl:"):
            agent = int(target.split(":")[1])
            self.may_exit.add(f"agent_{agent}")       # it exits by itself once its link dies
            self.fleet.kill_vehicle(agent)            # raises if the Gazebo model stays
        elif action == "kill":
            self.kill(target.replace("agent:", "agent_"))
        elif target == "ground":
            self.start_ground()
        else:
            self.start_radio()
        if ":" in target:
            self.meta["kill_wall"] = fault["wall"]

    # ---- watching
    def check(self, t_mission):
        """Raise Invalid if the thermal guard paused anything or a process ended unexpectedly."""
        if taint_mtime() != self.taint0:
            self.meta["tainted"] = True
            raise Invalid("tainted: the thermal guard paused the run")
        for name, process in self.procs.items():
            ended_on_time = name == "ground" and t_mission >= self.args.mission_s - 1.0
            if process.poll() is not None and name not in self.may_exit and not ended_on_time:
                raise Invalid(f"{name} died unexpectedly (exit {process.returncode})")
        if self.fleet is not None:
            for agent, process in self.fleet.autopilots.items():
                if process.poll() is not None:
                    raise Invalid(f"arducopter {agent} died unexpectedly")
            if self.fleet.gz is not None and self.fleet.gz.poll() is not None:
                raise Invalid("gz sim died unexpectedly")

    def wait_until(self, t_wall):
        while time.time() < t_wall:
            self.check(time.time() - self.meta["t0_wall"])
            time.sleep(LOOP_S)

    def wait_for_start(self):
        deadline = time.time() + START_TIMEOUT_S
        while self.meta["t0_wall"] is None:
            if time.time() > deadline:
                raise Invalid(f"the mission never started (no START in {START_TIMEOUT_S:.0f} s)")
            self.check(-config.COVERAGE_WINDOW_S)
            time.sleep(LOOP_S)
            self.meta["t0_wall"] = read_t0(os.path.join(self.out, "ground.jsonl"))
        self.save_meta()
        log(f"START received; t = 0 in {self.meta['t0_wall'] - time.time():.0f} s")

    def rtf(self):
        return self.fleet.real_time_factor() if self.args.backend == "gazebo" else None

    # ---- the whole run
    def run(self):
        started = time.time()
        if self.args.backend != "fake":
            log("waiting for the machine to be cool")
            subprocess.run(["bash", os.path.join(ROOT, "tools", "thermal_guard.sh"), "--wait"],
                           check=True)
            self.taint0 = taint_mtime()
            self.fleet = launch.Fleet(self.args.backend, self.args.n,
                                      os.path.join(self.out, "sim"))
            self.fleet.start()
        self.start_radio()
        self.start_ground()
        for a in range(1, self.args.n + 1):
            self.start_agent(a)
        self.wait_for_start()
        t0 = self.meta["t0_wall"]
        self.wait_until(t0)
        rtf_start = self.rtf()
        for fault in self.meta["faults"]:
            self.wait_until(t0 + fault["t_mission"])
            self.apply(fault)
            self.save_meta()
        self.wait_until(t0 + self.args.mission_s - 2.0)
        if self.fleet is not None:
            self.meta["rtf"] = {"start": rtf_start, "end": self.rtf()}
        self.wait_until(t0 + self.args.mission_s + END_GRACE_S)
        self.meta["wall_s"] = round(time.time() - started)

    def stop(self):
        """Stop everything we started: agents first (they close their logs), then the rest."""
        launch.unregister_pids("run_")              # the guard must not pause them again
        names = sorted(self.procs, key=lambda name: not name.startswith("agent_"))
        for name in names:
            if self.procs[name].poll() is None:
                self.procs[name].send_signal(signal.SIGCONT)
                self.procs[name].terminate()
        for name in names:
            try:
                self.procs[name].wait(timeout=STOP_WAIT_S)
            except subprocess.TimeoutExpired:
                log(f"{name} ignored SIGTERM: SIGKILL")
                self.procs[name].kill()
                self.procs[name].wait()
        if self.fleet is not None:
            self.fleet.stop()


def interrupted(signum, frame):
    raise Invalid(f"interrupted by signal {signum}")


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="Run one mission end to end.")
    ap.add_argument("--mission", choices=("A", "B", "C", "ref"), required=True)
    ap.add_argument("--backend", choices=("gazebo", "sitl", "fake"), required=True)
    ap.add_argument("--n", type=int, default=3)
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--out", required=True, help="run directory; must not exist or be empty")
    ap.add_argument("--scenario", choices=sorted(SCENARIOS))
    ap.add_argument("--mission-s", type=float,
                    help="default 180 s; 240 s for the obstacle scenario (blacklist expiry)")
    ap.add_argument("--zone", type=lambda text: [float(v) for v in text.split(",")],
                    help="Mission B priority zone xmin,ymin,xmax,ymax (default: the middle)")
    args = ap.parse_args(argv)
    if args.scenario and SCENARIOS[args.scenario][0] != args.mission:
        ap.error(f"--scenario {args.scenario} needs --mission {SCENARIOS[args.scenario][0]}")
    if args.mission_s is None:
        args.mission_s = 240.0 if args.scenario == "obstacle" else config.MISSION_S
    out = os.path.abspath(args.out)
    if args.backend == "fake" and any(out.startswith(os.path.join(ROOT, d) + os.sep)
                                      for d in ("runs", "results")):
        ap.error("the fake backend is not a measurement: use run/, not runs/ or results/")
    if os.path.isdir(out) and os.listdir(out):
        ap.error(f"{args.out} is not empty (the ground would read an old ground.jsonl)")
    return args


def main(argv=None):
    args = parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    launch.register_pid("run_mission", os.getpid())  # the guard pauses us with the rest
    mission = Mission(args, plan(args))
    mission.save_meta()
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupted)
    try:
        mission.run()
        mission.meta["outcome"] = "ok"
    except (Invalid, RuntimeError, subprocess.CalledProcessError) as error:
        mission.meta["outcome"] = f"invalid: {error}"
    finally:
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(signum, signal.SIG_IGN)       # a second signal must not stop the stop
        mission.stop()
    if taint_mtime() != mission.taint0:
        mission.meta["tainted"] = True
        mission.meta["outcome"] = "invalid: tainted: the thermal guard paused the run"
    if args.backend != "fake" and mission.meta["t0_wall"] is not None:
        mission.meta["autopilot_params"] = read_params(mission.out, args.n)
    mission.save_meta()
    log(f"{mission.meta['outcome']} -> {args.out}")
    return 0 if mission.meta["outcome"] == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
