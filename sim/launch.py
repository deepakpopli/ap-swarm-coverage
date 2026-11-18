#!/usr/bin/env python3
"""Fleet launcher: N ArduCopter SITL processes, optionally under one Gazebo Harmonic world.

    from sim.launch import Fleet
    with Fleet("sitl", 3, "run/myexp") as fleet:   # or Fleet("gazebo", ...)
        fleet.start()
        ...
        fleet.kill_vehicle(2)       # SIGKILL of agent 2's arducopter
    # leaving the block (or fleet.stop()) tears everything down

CLI:  python3 sim/launch.py --backend sitl -n 3 --run-dir run/fleet   (Ctrl-C stops)

Ids and ports
  agent id 1..N == MAV_SYSID; SITL instance = base_instance + id - 1
  autopilot MAVLink: tcp 127.0.0.1:5760 + 10*instance (see mavlink_port())
  Gazebo JSON FDM port: 9002 + 10*instance

Frames
  Agent index i = id - 1 starts at config.start_xy(i, n) in the area ENU frame
  (x east, y north, metres) whose origin is config.ORIGIN_LLA.
  sitl:   each arducopter gets --home = config.enu_to_lla(start), alt = origin alt,
          so the EKF origin / home of every vehicle is its own start point.
  gazebo: the world's spherical_coordinates is the area origin, model i is placed at
          its start (x east, y north) and every arducopter gets --home = area origin.
          The JSON backend reports the model's world position relative to --home, so
          GLOBAL_POSITION_INT again lands on the start point. The plugin runs with
          lock_step=0 so a killed autopilot does not freeze the world.

CPU / thermal rules: every process runs under `nice -n 19` and its PID is written to
run/pids/<tag>_*.pid so tools/thermal_guard.sh can SIGSTOP it; the files are removed in stop().
"""
import argparse
import atexit
import os
import signal
import socket
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from swarm import config                       # noqa: E402
from sim.gazebo import gen_world               # noqa: E402

# same env vars and defaults as setup.sh
ARDUPILOT = os.path.expanduser(os.environ.get("ARDUPILOT_HOME", "~/ardupilot"))
ARDUCOPTER = os.path.join(ARDUPILOT, "build/sitl/bin/arducopter")
DEFAULTS_DIR = os.path.join(ARDUPILOT, "Tools/autotest/default_params")
AP_GAZEBO = os.path.expanduser(os.environ.get("ARDUPILOT_GAZEBO", "~/ardupilot_gazebo"))
PID_DIR = os.path.join(ROOT, "run", "pids")


def mavlink_port(instance):
    """TCP port of SITL instance `instance`'s first serial port (SERIAL0)."""
    return 5760 + 10 * instance


def _port_open(port, host="127.0.0.1"):
    """True if something accepts TCP connections on host:port."""
    try:
        with socket.create_connection((host, port), timeout=0.3):
            return True
    except OSError:
        return False


def _session_pids(sid):
    """All live PIDs whose session id is `sid` (the gz sim launcher plus its children)."""
    pids = []
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            fields = open(f"/proc/{d}/stat").read().rsplit(")", 1)[1].split()
            if int(fields[3]) == sid:      # field 6 of /proc/pid/stat = session id
                pids.append(int(d))
        except (OSError, IndexError, ValueError):
            pass
    return pids


class Fleet:
    """N ArduCopter SITL autopilots (+ one Gazebo server for backend "gazebo").

    backend:       "sitl" (ArduPilot's built-in quad physics) or "gazebo" (Gazebo Harmonic iris).
    n:             number of vehicles; agent ids are 1..n.
    run_dir:       working directory for logs/eeprom (created).
    base_instance: SITL instance of agent 1. For gazebo, base_instance != 0 shifts the FDM
                   ports and model names accordingly, but only ONE Gazebo world may run at a time.
    """

    def __init__(self, backend="sitl", n=3, run_dir=None, base_instance=0):
        if backend not in ("sitl", "gazebo"):
            raise ValueError("backend must be 'sitl' or 'gazebo'")
        self.backend = backend
        self.n = n
        self.base_instance = base_instance
        self.run_dir = os.path.abspath(run_dir or os.path.join(ROOT, "run", f"fleet_{backend}"))
        self.tag = f"fleet{os.getpid()}_{backend}{base_instance}"
        self.procs = {}          # agent id -> Popen of its arducopter
        self.gz = None           # Popen of `gz sim` (session leader) or None
        self._pid_files = []
        self.t_start = None      # wall time start() began
        atexit.register(self.stop)

    # ---- ids / positions -------------------------------------------------------------
    def instance(self, agent_id):
        """SITL instance number of agent `agent_id` (1..n)."""
        return self.base_instance + agent_id - 1

    def port(self, agent_id):
        """MAVLink TCP port of agent `agent_id`'s autopilot."""
        return mavlink_port(self.instance(agent_id))

    def start_xy(self, agent_id):
        """Start position (x east, y north) in metres, area ENU frame."""
        return config.start_xy(agent_id - 1, self.n)

    @property
    def pids(self):
        """{name: pid} of every process this fleet started and that is still registered."""
        out = {f"sitl{a}": p.pid for a, p in self.procs.items()}
        if self.gz is not None:
            out["gz"] = self.gz.pid
        return out

    # ---- pid registry for the thermal guard ------------------------------------------
    def _register(self, name, pids):
        os.makedirs(PID_DIR, exist_ok=True)
        path = os.path.join(PID_DIR, f"{self.tag}_{name}.pid")
        with open(path, "w") as f:
            f.write(" ".join(str(p) for p in pids))
        if path not in self._pid_files:
            self._pid_files.append(path)

    def _unregister(self, name):
        path = os.path.join(PID_DIR, f"{self.tag}_{name}.pid")
        if os.path.exists(path):
            os.remove(path)
        if path in self._pid_files:
            self._pid_files.remove(path)

    # ---- start -----------------------------------------------------------------------
    def start(self, timeout=60.0):
        """Start the simulator(s); return once every autopilot accepts TCP on its MAVLink port.

        Does not arm or take off (that is MavVehicle's job). Raises RuntimeError on failure
        after cleaning up whatever was started.
        """
        self.t_start = time.time()
        os.makedirs(self.run_dir, exist_ok=True)
        try:
            if self.backend == "gazebo":
                self._start_gazebo()
            for agent_id in range(1, self.n + 1):
                self._start_sitl(agent_id)
            deadline = time.time() + timeout
            for agent_id in range(1, self.n + 1):
                while not _port_open(self.port(agent_id)):
                    if self.procs[agent_id].poll() is not None:
                        raise RuntimeError(f"arducopter for agent {agent_id} exited, see "
                                           f"{self.run_dir}/s{self.instance(agent_id)}/sitl.log")
                    if time.time() > deadline:
                        raise RuntimeError(f"agent {agent_id}: port {self.port(agent_id)} not open")
                    time.sleep(0.2)
        except BaseException:
            self.stop()
            raise
        return self

    def _start_gazebo(self):
        gz_dir = os.path.join(self.run_dir, "gz")
        positions = [self.start_xy(a) for a in range(1, self.n + 1)]
        instances = [self.instance(a) for a in range(1, self.n + 1)]
        sdf = gen_world.generate(gz_dir, positions, config.ORIGIN_LLA, instances, lock_step=False)
        env = dict(os.environ)
        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.path.join(AP_GAZEBO, "build")
        env["GZ_SIM_RESOURCE_PATH"] = os.path.join(gz_dir, "models") + ":" + os.path.join(AP_GAZEBO, "models")
        env["GZ_PARTITION"] = self.tag     # keep gz-transport traffic private to this fleet
        log = open(os.path.join(self.run_dir, "gz.log"), "w")
        # new session so stop() can signal the launcher and all its children as one group
        self.gz = subprocess.Popen(["nice", "-n", "19", "gz", "sim", "-s", "-r", "-v", "2", sdf],
                                   env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self._register("gz", [self.gz.pid])
        time.sleep(5.0)                    # let the server load the world and plugins
        if self.gz.poll() is not None:
            raise RuntimeError(f"gz sim exited, see {self.run_dir}/gz.log")
        self._register("gz", _session_pids(self.gz.pid) or [self.gz.pid])

    def _start_sitl(self, agent_id):
        inst = self.instance(agent_id)
        wd = os.path.join(self.run_dir, f"s{inst}")
        os.makedirs(wd, exist_ok=True)
        defaults = os.path.join(DEFAULTS_DIR, "copter.parm")
        if self.backend == "gazebo":
            model = "JSON"
            defaults += "," + os.path.join(DEFAULTS_DIR, "gazebo-iris.parm")
            lat, lon = config.ORIGIN_LLA[0], config.ORIGIN_LLA[1]
        else:
            model = "+"
            lat, lon = config.enu_to_lla(*self.start_xy(agent_id))
        home = f"{lat:.8f},{lon:.8f},{config.ORIGIN_LLA[2]},0"
        cmd = ["nice", "-n", "19", ARDUCOPTER, "-w", "--model", model, "-I", str(inst),
               "--sysid", str(agent_id), "--home", home, "--defaults", defaults]
        log = open(os.path.join(wd, "sitl.log"), "w")
        # `nice` execs arducopter, so Popen.pid is the arducopter pid itself
        p = subprocess.Popen(cmd, cwd=wd, stdout=log, stderr=subprocess.STDOUT)
        self.procs[agent_id] = p
        self._register(f"sitl{agent_id}", [p.pid])

    # ---- kill / stop -----------------------------------------------------------------
    def kill_vehicle(self, agent_id):
        """SIGKILL agent `agent_id`'s arducopter (simulated sudden autopilot loss). Returns kill wall time."""
        p = self.procs.get(agent_id)
        if p is None:
            raise KeyError(agent_id)
        t = time.time()
        if p.poll() is None:
            os.kill(p.pid, signal.SIGKILL)
        p.wait(timeout=5)
        self._unregister(f"sitl{agent_id}")
        return t

    def alive(self, agent_id):
        """True while agent `agent_id`'s arducopter process is running."""
        p = self.procs.get(agent_id)
        return p is not None and p.poll() is None

    def stop(self):
        """Tear down: SIGTERM every arducopter, then the gz session; SIGKILL whatever lingers."""
        for p in self.procs.values():
            if p.poll() is None:
                p.terminate()
        for p in self.procs.values():
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait(timeout=3)
        if self.gz is not None and self.gz.poll() is None:
            try:
                os.killpg(self.gz.pid, signal.SIGTERM)   # only the session we created
                self.gz.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(self.gz.pid, signal.SIGKILL)
                self.gz.wait(timeout=3)
            except ProcessLookupError:
                pass
        if self.gz is not None:
            # children of the launcher may outlive it; they are still in our session
            for pid in _session_pids(self.gz.pid):
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        for path in list(self._pid_files):
            if os.path.exists(path):
                os.remove(path)
        self._pid_files.clear()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.stop()
        return False


def main():
    ap = argparse.ArgumentParser(description="Start a SITL fleet and keep it running until Ctrl-C.")
    ap.add_argument("--backend", choices=["sitl", "gazebo"], default="sitl")
    ap.add_argument("-n", type=int, default=3)
    ap.add_argument("--run-dir", default=None)
    ap.add_argument("--base-instance", type=int, default=0)
    a = ap.parse_args()
    fleet = Fleet(a.backend, a.n, a.run_dir, a.base_instance)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    with fleet:
        fleet.start()
        for agent_id in range(1, a.n + 1):
            print(f"agent {agent_id}: tcp:127.0.0.1:{fleet.port(agent_id)} start_xy={fleet.start_xy(agent_id)}")
        print("pids", fleet.pids, flush=True)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
