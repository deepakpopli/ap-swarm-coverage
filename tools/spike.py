#!/usr/bin/env python3
"""Bring-up / stack spike: N ArduCopter SITL, GUIDED velocity at 10 Hz, speed-cap test, SIGKILL test.

  spike.py --backend sitl    # SITL built-in physics
  spike.py --backend gazebo  # Gazebo Harmonic (server only) + ardupilot_gazebo JSON

Measures: time to airborne, real-time factor, control-loop period stats, max ground speed
when 15 m/s is commanded with WP_SPD=12, CPU per process, CPU temperature, and that
the other vehicles keep flying after one arducopter gets SIGKILL.
Prints one JSON line at the end (also written to --out).
"""
import argparse, glob, json, math, os, signal, subprocess, sys, threading, time
from pymavlink import mavutil

HOME = (-35.363262, 149.165237, 584.0)
AP = os.path.expanduser("~/ardupilot")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLK = os.sysconf("SC_CLK_TCK")


def cpu_temp():
    for h in glob.glob("/sys/class/hwmon/hwmon*"):
        try:
            if open(h + "/name").read().strip() == "k10temp":
                return int(open(h + "/temp1_input").read()) / 1000
        except OSError:
            pass
    return float("nan")


def cpu_ticks(pid):
    """utime+stime of a pid plus all its descendants (seconds)."""
    tot, todo = 0.0, [pid]
    while todo:
        p = todo.pop()
        try:
            f = open(f"/proc/{p}/stat").read().rsplit(")", 1)[1].split()
            tot += (int(f[11]) + int(f[12])) / CLK
            for t in os.listdir(f"/proc/{p}/task"):
                todo += [int(c) for c in open(f"/proc/{p}/task/{t}/children").read().split()]
        except (OSError, IndexError):
            pass
    return tot


def register(name, pid):
    d = os.path.join(ROOT, "run", "pids"); os.makedirs(d, exist_ok=True)
    open(os.path.join(d, name + ".pid"), "w").write(str(pid))


class Vehicle(threading.Thread):
    def __init__(self, idx, alt, speed_cmd, fly_s):
        super().__init__(daemon=True)
        self.i, self.alt, self.speed_cmd, self.fly_s = idx, alt, speed_cmd, fly_s
        self.res = {"id": idx}; self.phase = "init"; self.alive_after_kill = None
        self.last_pos_wall = 0.0; self.stop = False

    def log(self, *a):
        print(f"[v{self.i} {time.time()-T0:6.1f}s]", *a, flush=True)

    def run(self):
        try:
            self._run()
        except Exception as e:  # noqa
            self.res["error"] = repr(e); self.phase = "error"; self.log("ERROR", e)

    def _run(self):
        m = mavutil.mavlink_connection(f"tcp:127.0.0.1:{5760+10*self.i}", source_system=250, retries=30)
        self.m = m
        m.wait_heartbeat(timeout=60)
        self.log("heartbeat sysid", m.target_system)
        m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50000, 0, 0, 0, 0, 0)
        # speed limit through ArduPilot parameters (this build: WP_SPD in m/s; older builds: WPNAV_SPEED in cm/s)
        for name, val in [("WP_SPD", WP_SPD)] + EXTRA:
            m.mav.param_set_send(m.target_system, m.target_component, name.encode(), val, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            r = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
            got = None
            t = time.time()
            while time.time() - t < 3:
                if r and r.param_id.strip("\x00") == name:
                    got = r.param_value; break
                r = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            self.res["param_" + name] = got
        self.log("params", {k: v for k, v in self.res.items() if k.startswith("param")})
        # GUIDED, arm (retry until pre-arm passes), takeoff
        self.phase = "arming"
        t_arm0 = time.time()
        while not m.motors_armed():
            m.set_mode("GUIDED")
            m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
            t = time.time()
            while time.time() - t < 2:
                m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
                if m.motors_armed():
                    break
            if time.time() - t_arm0 > 180:
                raise RuntimeError("arm timeout")
        self.res["armed_after_s"] = round(time.time() - T0, 1)
        self.log("armed")
        m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, self.alt)
        self.phase = "takeoff"
        while True:
            p = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=5)
            if p and -p.z > self.alt - 1.0:
                break
        self.res["airborne_after_s"] = round(time.time() - T0, 1)
        self.log("at altitude")
        self.phase = "ready"
        while not GO.is_set():
            self.hold(m)
        # velocity commands at 10 Hz: north at speed_cmd
        self.phase = "fly"
        periods, vmax, vsteady = [], 0.0, []
        t_start = time.time(); nxt = t_start; last = None
        boot0 = None; wall0 = None
        while not self.stop:
            now = time.time()
            if now >= nxt:
                el = now - t_start
                if el < self.fly_s:
                    vn = self.speed_cmd
                else:
                    vn = 0.0; self.phase = "hover"
                m.mav.set_position_target_local_ned_send(0, m.target_system, m.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0x0DC7, 0, 0, 0, vn, 0, 0, 0, 0, 0, 0, 0)
                if last is not None:
                    periods.append(now - last)
                last = now; nxt += 0.1
                if nxt < now:
                    nxt = now + 0.1
            p = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=max(0.0, min(0.02, nxt - time.time())))
            if p:
                self.last_pos_wall = time.time()
                sp = math.hypot(p.vx, p.vy)
                if self.phase == "fly":
                    vmax = max(vmax, sp)
                    if time.time() - t_start > self.fly_s * 0.6:
                        vsteady.append(sp)
                if boot0 is None:
                    boot0, wall0 = p.time_boot_ms, time.time()
                self.res["rtf"] = round((p.time_boot_ms - boot0) / 1000 / max(1e-6, time.time() - wall0), 3) if time.time() - wall0 > 5 else None
                self.res["vmax"] = round(vmax, 2)
                if vsteady:
                    self.res["v_steady_mean"] = round(sum(vsteady) / len(vsteady), 2)
                if len(periods) > 10:
                    ps = sorted(periods)
                    self.res["loop_ms_mean"] = round(1000 * sum(ps) / len(ps), 2)
                    self.res["loop_ms_p99"] = round(1000 * ps[int(0.99 * (len(ps) - 1))], 2)
                    self.res["loop_ms_max"] = round(1000 * ps[-1], 2)

    def hold(self, m):
        m.mav.set_position_target_local_ned_send(0, m.target_system, m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0x0DC7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.1)


def main():
    global T0, GO
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", choices=["sitl", "gazebo"], default="sitl")
    ap.add_argument("-n", type=int, default=3)
    ap.add_argument("--base-instance", type=int, default=0)
    ap.add_argument("--fly-s", type=float, default=25)
    ap.add_argument("--speed-cmd", type=float, default=15.0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--no-lock-step", action="store_true")
    ap.add_argument("--wp-spd", type=float, default=12.0)
    ap.add_argument("--param", action="append", default=[], help="NAME=VALUE extra parameter (repeatable)")
    a = ap.parse_args()
    global WP_SPD, EXTRA
    WP_SPD = a.wp_spd; EXTRA = [(x.split("=")[0], float(x.split("=")[1])) for x in a.param]
    if a.base_instance != 0 and a.backend == "gazebo":
        sys.exit("gazebo world uses instances 0..n-1")
    work = os.path.join(ROOT, "run", f"spike_{a.backend}"); os.makedirs(work, exist_ok=True)
    procs = {}
    T0 = time.time(); GO = threading.Event()
    gz = None
    try:
        if a.backend == "gazebo":
            subprocess.check_call([sys.executable, os.path.join(ROOT, "sim/gazebo/gen_world.py"), str(a.n), os.path.join(work, "gz")] + (["--no-lock-step"] if a.no_lock_step else []))
            env = dict(os.environ)
            env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.path.expanduser("~/ardupilot_gazebo/build")
            env["GZ_SIM_RESOURCE_PATH"] = os.path.join(work, "gz", "models") + ":" + os.path.expanduser("~/ardupilot_gazebo/models")
            gz = subprocess.Popen(["nice", "-n", "19", "gz", "sim", "-s", "-r", "-v", "2", os.path.join(work, "gz", "swarm.sdf")],
                                  env=env, stdout=open(os.path.join(work, "gz.log"), "w"), stderr=subprocess.STDOUT, start_new_session=True)
            register("spike_gz", gz.pid)
            time.sleep(6)
            if gz.poll() is not None:
                sys.exit("gz sim exited, see " + work + "/gz.log")
        dlon = 10.0 / (111320.0 * math.cos(math.radians(HOME[0])))
        for k in range(a.n):
            i = a.base_instance + k
            d = os.path.join(work, f"s{i}"); os.makedirs(d, exist_ok=True)
            defaults = AP + "/Tools/autotest/default_params/copter.parm"
            if a.backend == "gazebo":
                model = "JSON"; defaults += "," + AP + "/Tools/autotest/default_params/gazebo-iris.parm"
            else:
                model = "+"
            cmd = ["nice", "-n", "19", AP + "/build/sitl/bin/arducopter", "-w", "--model", model, "-I", str(i), "--sysid", str(i + 1),
                   "--home", f"{HOME[0]},{HOME[1] + k*dlon:.7f},{HOME[2]},0", "--defaults", defaults]
            p = subprocess.Popen(cmd, cwd=d, stdout=open(os.path.join(d, "sitl.log"), "w"), stderr=subprocess.STDOUT)
            procs[i] = p; register(f"spike_sitl{i}", p.pid)
        time.sleep(2)
        vs = [Vehicle(a.base_instance + k, 30.0, a.speed_cmd, a.fly_s) for k in range(a.n)]
        for v in vs:
            v.start()
        t = time.time()
        while not all(v.phase in ("ready", "error") for v in vs):
            time.sleep(0.5)
            if time.time() - t > 300:
                raise RuntimeError("bring-up timeout: " + str([v.phase for v in vs]))
        if any(v.phase == "error" for v in vs):
            raise RuntimeError("vehicle error")
        bringup_s = round(time.time() - T0, 1)
        print("all airborne after", bringup_s, "s; starting velocity test", flush=True)
        cpu0 = {i: cpu_ticks(p.pid) for i, p in procs.items()}
        gz0 = cpu_ticks(gz.pid) if gz else 0
        w0 = time.time(); temps = []
        GO.set()
        while time.time() - w0 < a.fly_s + 3:
            temps.append(cpu_temp()); time.sleep(1)
        w1 = time.time()
        cpu = {f"sitl{i}_cpu_pct": round(100 * (cpu_ticks(p.pid) - cpu0[i]) / (w1 - w0), 1) for i, p in procs.items()}
        if gz:
            cpu["gz_cpu_pct"] = round(100 * (cpu_ticks(gz.pid) - gz0) / (w1 - w0), 1)
        load1 = float(open("/proc/loadavg").read().split()[0])
        # SIGKILL the middle vehicle's arducopter
        victim = a.base_instance + a.n // 2
        procs[victim].kill()  # SIGKILL
        t_kill = time.time()
        print("SIGKILL sent to instance", victim, flush=True)
        time.sleep(6)
        surv = {v.i: (v.last_pos_wall > t_kill + 4) for v in vs if v.i != victim}
        dead_silent = [v for v in vs if v.i == victim][0].last_pos_wall < t_kill + 1
        for v in vs:
            v.stop = True
        time.sleep(0.5)
        out = {"backend": a.backend, "n": a.n, "wp_spd": a.wp_spd, "speed_cmd": a.speed_cmd, "extra_params": a.param, "bringup_s": bringup_s, "vehicles": [v.res for v in vs], **cpu,
               "cpu_total_pct": round(sum(cpu.values()), 1), "load1": load1,
               "temp_c_max": max(temps), "temp_c_mean": round(sum(temps) / len(temps), 1),
               "lock_step": not a.no_lock_step, "kill_victim": victim, "survivors_still_reporting": surv, "victim_silent_within_1s": dead_silent}
        print(json.dumps(out, indent=1))
        if a.out:
            json.dump(out, open(a.out, "w"), indent=1)
    finally:
        for v in list(procs.values()):
            if v.poll() is None:
                v.terminate()
        if gz and gz.poll() is None:
            os.killpg(gz.pid, signal.SIGTERM)  # only the process group we created
            try:
                gz.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(gz.pid, signal.SIGKILL)
        for f in glob.glob(os.path.join(ROOT, "run", "pids", "spike_*.pid")):
            os.remove(f)


if __name__ == "__main__":
    main()
