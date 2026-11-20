"""Fleet: N ArduCopter SITL autopilots, either with ArduPilot's built-in physics ("sitl") or
inside one Gazebo Harmonic world ("gazebo"). Only one fleet may run at a time.

Agent id 1..n = MAV_SYSID; SITL instance = id - 1; MAVLink on TCP config.sitl_tcp_port(id).
Both backends report the same area ENU positions:
  sitl:   the home of each arducopter is its start point config.start_xy(id - 1, n).
  gazebo: the home of every arducopter is the area origin, the world puts model id - 1 at its
          start point, and the JSON backend reports positions relative to home.

Every child runs under `nice -n 19`, gets SIGKILL when this process dies (PR_SET_PDEATHSIG,
so call start() from the main thread) and has its PID in run/pids/ so tools/thermal_guard.sh
can pause it.
"""
import ctypes
import glob
import os
import re
import signal
import subprocess
import time
from typing import Optional

from sim import gazebo_world
from swarm import config

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PID_DIR = os.path.join(ROOT, "run", "pids")
ARDUPILOT = os.path.expanduser(os.environ.get("ARDUPILOT_HOME", "~/ardupilot"))
ARDUCOPTER = os.path.join(ARDUPILOT, "build/sitl/bin/arducopter")
DEFAULT_PARAMS = os.path.join(ARDUPILOT, "Tools/autotest/default_params")
AP_GAZEBO = gazebo_world.AP_GAZEBO
PR_SET_PDEATHSIG = 1


def register_pid(name: str, pid: int) -> None:
    """Write run/pids/<name>.pid so the thermal guard can pause this process."""
    os.makedirs(PID_DIR, exist_ok=True)
    with open(os.path.join(PID_DIR, f"{name}.pid"), "w") as f:
        f.write(str(pid))


def unregister_pids(prefix: str) -> None:
    """Remove every run/pids/<prefix>*.pid."""
    for path in glob.glob(os.path.join(PID_DIR, f"{prefix}*.pid")):
        os.remove(path)


def _die_with_parent() -> None:
    """Runs in the child before exec: the kernel sends it SIGKILL when the parent dies."""
    ctypes.CDLL("libc.so.6", use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGKILL)


def start_process(cmd: list, log_path: str, **popen_args) -> subprocess.Popen:
    """Start cmd under nice -n 19, output to log_path, killed when we die (call it from the
    main thread). nice execs cmd, so the Popen pid is the pid of cmd itself."""
    log = open(log_path, "w")
    return subprocess.Popen(["nice", "-n", "19"] + cmd, stdout=log, stderr=subprocess.STDOUT,
                            preexec_fn=_die_with_parent, **popen_args)


def _port_holder(port: int) -> str:
    """The `ss` line of whoever listens on TCP port, '' if it is free."""
    result = subprocess.run(["ss", "-Htlnp", f"sport = :{port}"], capture_output=True, text=True)
    return result.stdout.strip()


class Fleet:
    def __init__(self, backend: str, n: int, run_dir: str):
        self.backend = backend        # "sitl" or "gazebo"
        self.n = n
        self.run_dir = os.path.abspath(run_dir)
        self.autopilots = {}          # agent id -> Popen of its arducopter
        self.gz = None                # Popen of `gz sim -s` (one process: the server)
        self.env = dict(os.environ, GZ_PARTITION=f"swarm{os.getpid()}",
                        GZ_SIM_SYSTEM_PLUGIN_PATH=os.path.join(AP_GAZEBO, "build"),
                        GZ_SIM_RESOURCE_PATH=os.path.join(self.run_dir, "gz", "models") + ":"
                        + os.path.join(AP_GAZEBO, "models"))

    def port(self, agent_id: int) -> int:
        return config.sitl_tcp_port(agent_id)

    def start(self, timeout_s: float = 60.0) -> None:
        """Start everything; return when every autopilot accepts TCP. Does not arm."""
        for agent_id in range(1, self.n + 1):
            holder = _port_holder(self.port(agent_id))
            if holder:
                raise RuntimeError(f"port {self.port(agent_id)} is busy: {holder}")
        unregister_pids("fleet_")     # left over if an earlier runner was killed
        os.makedirs(self.run_dir, exist_ok=True)
        if self.backend == "gazebo":
            self._start_gazebo()
        for agent_id in range(1, self.n + 1):
            self._start_autopilot(agent_id)
        deadline = time.time() + timeout_s
        for agent_id in range(1, self.n + 1):
            while not _port_holder(self.port(agent_id)):
                if self.autopilots[agent_id].poll() is not None or time.time() > deadline:
                    self.stop()
                    raise RuntimeError(f"autopilot {agent_id} did not start, see {self.run_dir}")
                time.sleep(0.2)

    def _start_gazebo(self) -> None:
        gz_dir = os.path.join(self.run_dir, "gz")
        positions = [config.start_xy(i, self.n) for i in range(self.n)]
        sdf = gazebo_world.generate(gz_dir, positions, config.ORIGIN_LLA)
        self.gz = start_process(["gz", "sim", "-s", "-r", "-v", "2", sdf],
                         os.path.join(self.run_dir, "gz.log"), env=self.env)
        register_pid("fleet_gz", self.gz.pid)

    def _start_autopilot(self, agent_id: int) -> None:
        instance = agent_id - 1
        work_dir = os.path.join(self.run_dir, f"s{instance}")
        os.makedirs(work_dir, exist_ok=True)
        params = os.path.join(DEFAULT_PARAMS, "copter.parm")
        if self.backend == "gazebo":
            model = "JSON"
            params += "," + os.path.join(DEFAULT_PARAMS, "gazebo-iris.parm")
            lat, lon = config.ORIGIN_LLA[0], config.ORIGIN_LLA[1]
        else:
            model = "+"
            lat, lon = config.enu_to_lla(*config.start_xy(instance, self.n))
        home = f"{lat:.8f},{lon:.8f},{config.ORIGIN_LLA[2]},0"
        cmd = [ARDUCOPTER, "-w", "--model", model, "-I", str(instance), "--sysid", str(agent_id),
               "--home", home, "--defaults", params]
        process = start_process(cmd, os.path.join(work_dir, "sitl.log"), cwd=work_dir)
        self.autopilots[agent_id] = process
        register_pid(f"fleet_sitl{agent_id}", process.pid)

    def kill_vehicle(self, agent_id: int) -> bool:
        """SIGKILL the agent's arducopter (sudden autopilot loss). True if a model was removed.

        gazebo: the dead model is then removed. The ardupilot_gazebo plugin waits 1 ms per
        physics step for an autopilot that does not answer, which dropped the whole world to
        ~0.6x real time. A run with a slowed world is invalid, so a failed removal raises.
        """
        process = self.autopilots.pop(agent_id)
        process.kill()
        process.wait()
        unregister_pids(f"fleet_sitl{agent_id}")
        if self.backend == "sitl":
            return False
        reply = self._gz(["service", "-s", "/world/swarm/remove", "--reqtype", "gz.msgs.Entity",
                          "--reptype", "gz.msgs.Boolean", "--timeout", "3000",
                          "--req", f'name: "iris_{agent_id - 1}" type: MODEL'])
        if "data: true" not in reply:
            raise RuntimeError(f"could not remove iris_{agent_id - 1}: {reply!r}")
        return True

    def real_time_factor(self) -> Optional[float]:
        """Gazebo's current real-time factor (takes ~1 s); None for sitl or if gz did not
        answer (it timed out twice in 34 runs; a probe must not end the run)."""
        if self.backend == "sitl":
            return None
        found = re.search(r"real_time_factor: ([\d.e-]+)",
                          self._gz(["topic", "-e", "-n", "1", "-t", "/world/swarm/stats"]))
        return float(found.group(1)) if found else None

    def _gz(self, args: list) -> str:
        """stdout of a gz command, '' if it does not answer within 10 s."""
        try:
            result = subprocess.run(["gz"] + args, env=self.env, capture_output=True,
                                    text=True, timeout=10)
        except subprocess.TimeoutExpired:
            return ""
        return result.stdout

    def stop(self) -> None:
        """SIGTERM every child, SIGKILL what is left after a few seconds, unregister PIDs."""
        children = list(self.autopilots.values()) + ([self.gz] if self.gz else [])
        for process in children:
            process.terminate()
        for process in children:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        self.autopilots = {}
        self.gz = None
        unregister_pids("fleet_")
