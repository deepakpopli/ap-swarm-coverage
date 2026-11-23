"""Observer web view: serves http://localhost:8800, starts one mission, streams its logs.

    python3 -m gui --port 8800 --runs runs

It only reads log files; it is not part of the control loop. Opened without ?run= it shows the
newest run, live while one is being written.
"""
import argparse
import json
import os
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)                      # so "python3 gui/server.py" works too

from swarm import config                     # noqa: E402
from gui.model import RunModel, footprint_geometry   # noqa: E402

STATIC_DIR = os.path.join(os.path.dirname(__file__), "static")
SIM_LOCK = os.path.join(ROOT, "run", "sim.lock")   # as tools/run_lib.sh
LIVE_PERIOD_S = 0.5
TAIL_LINES = 25
LIVE_FRESH_S = 15.0            # a run whose logs grew this recently is live
MISSIONS = ("A", "B", "C", "ref")
BACKENDS = ("fake", "gazebo", "sitl")


class Runner:
    """The one mission subprocess this dashboard started, if any."""

    def __init__(self):
        self.lock = threading.Lock()
        self.process = None
        self.run_dir = None
        self.out_path = None

    def running(self):
        return self.process is not None and self.process.poll() is None

    def start(self, mission, seed, n, backend):
        with self.lock:
            if self.running():
                raise ValueError("a mission is already running; stop it first")
            if backend == "fake" and radio_port_busy():
                raise ValueError(f"UDP {config.RADIO_PORT} is busy: another mission is running")
            name = f"{mission}_s{seed}_n{n}_{backend}_{time.strftime('%H%M%S')}"
            # run_mission refuses fake runs under runs/: they are not measurements
            parent = os.path.join(ROOT, "run" if backend == "fake" else "runs", "dash")
            os.makedirs(parent, exist_ok=True)
            self.run_dir = os.path.join(parent, name)
            self.out_path = self.run_dir + ".out"   # outside: the run dir must start empty
            argv = mission_argv(mission, seed, n, backend, self.run_dir)
            with open(self.out_path, "w") as out:
                out.write("$ " + (argv[-1] if backend != "fake" else shlex.join(argv)) + "\n")
                out.flush()
                self.process = subprocess.Popen(argv, cwd=ROOT, stdout=out,
                                                stderr=subprocess.STDOUT,
                                                env=dict(os.environ, PYTHONUNBUFFERED="1"))
            return self.run_dir

    def stop(self):
        """SIGTERM to run_mission.py, which stops its own children; else to the waiting wrapper."""
        with self.lock:
            if not self.running():
                return False
            pids = descendants(self.process.pid)
            missions = [p for p in pids if b"run_mission.py" in cmdline(p)]
            for pid in missions or [self.process.pid] + pids:
                try:
                    os.kill(pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
            return True

    def status(self):
        tail = ""
        if self.out_path and os.path.exists(self.out_path):
            with open(self.out_path, errors="replace") as f:
                tail = "".join(f.readlines()[-TAIL_LINES:])
        return {"running": self.running(), "dir": self.run_dir, "tail": tail,
                "pid": self.process.pid if self.process else None,
                "exit": None if self.process is None else self.process.poll()}


def mission_argv(mission, seed, n, backend, run_dir):
    args = ["run_mission.py", "--mission", mission, "--seed", str(seed), "--n", str(n),
            "--backend", backend, "--out", run_dir]
    if backend == "fake":
        return [sys.executable] + args
    # one simulation at a time on this machine; exec so SIGTERM reaches run_mission itself
    line = f"exec flock {shlex.quote(SIM_LOCK)} nice -n 19 python3 {shlex.join(args)}"
    return ["bash", "-c", line]


def radio_port_busy():
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(("127.0.0.1", config.RADIO_PORT))
        return False
    except OSError:
        return True
    finally:
        probe.close()


def descendants(pid):
    """Child PIDs, recursively, from /proc: only processes we started."""
    out = []
    try:
        with open(f"/proc/{pid}/task/{pid}/children") as f:
            children = [int(c) for c in f.read().split()]
    except OSError:
        return out
    for child in children:
        out += [child] + descendants(child)
    return out


def cmdline(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            return f.read()
    except OSError:
        return b""


def newest_mtime(run_dir):
    """Wall time the run's logs last grew (its top-level files)."""
    try:
        return max(e.stat().st_mtime for e in os.scandir(run_dir) if e.is_file())
    except (OSError, ValueError):
        return 0.0


def read_outcome(run_dir):
    try:
        with open(os.path.join(run_dir, "meta.json")) as f:
            return json.load(f).get("outcome")
    except (OSError, ValueError):
        return "running"             # being rewritten right now


def pick_live(runs, now, fresh_s=LIVE_FRESH_S):
    """runs: [(dir, outcome, newest mtime)], newest first. The live run: still "running" and
    its logs grew in the last fresh_s (a crashed run stays "running" but stops growing)."""
    for run_dir, outcome, mtime in runs:
        if outcome == "running" and now - mtime <= fresh_s:
            return run_dir
    return None


class Dashboard:
    def __init__(self, run_roots):
        self.run_roots = [os.path.abspath(r) for r in run_roots]
        self.runner = Runner()
        self.models = {}
        self.models_lock = threading.Lock()

    def run_dirs(self):
        """Every directory with a meta.json under the run roots, newest first."""
        found = []
        for root in self.run_roots:
            for path, dirs, files in os.walk(root):
                if "meta.json" in files:
                    found.append(path)
                    dirs.clear()     # a run never contains another run
        return sorted(found, key=lambda p: -os.path.getmtime(os.path.join(p, "meta.json")))

    def live_dir(self):
        """The run being written now: the one this dashboard started, else the newest live one
        under the run roots (tools/show_gazebo.sh, run_eval.sh ...)."""
        if self.runner.running():
            return self.runner.run_dir
        runs = [(d, read_outcome(d), newest_mtime(d)) for d in self.run_dirs()[:5]]
        return pick_live(runs, time.time())

    def checked_dir(self, run_dir):
        path = os.path.realpath(run_dir or "")
        allowed = self.run_roots + [os.path.dirname(self.runner.run_dir or "/nonexistent")]
        roots = [os.path.realpath(r) for r in allowed]
        if not any(path == r or path.startswith(r + os.sep) for r in roots):   # a root may be a run
            raise ValueError("not a run directory of this dashboard")
        return path

    def model(self, run_dir):
        """One model per directory, refreshed on use; a finished run is read once."""
        path = self.checked_dir(run_dir)
        with self.models_lock:
            model = self.models.get(path) or self.models.setdefault(path, RunModel(path))
        with model_lock(model):
            model.refresh()
        return model


def model_lock(model):
    if not hasattr(model, "lock"):
        model.lock = threading.Lock()
    return model.lock


class Handler(BaseHTTPRequestHandler):
    dash = None                      # set in main()

    def log_message(self, fmt, *args):
        pass                         # SSE and heat requests would flood the terminal

    def send_json(self, body, code=200):
        data = json.dumps(body, allow_nan=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        url = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(url.query).items()}
        routes = {"/api/runs": self.api_runs, "/api/run": self.api_run, "/api/heat": self.api_heat,
                  "/api/status": lambda q: self.send_json(self.dash.runner.status()),
                  "/api/live": self.api_live}
        try:
            if url.path in routes:
                routes[url.path](q)
            else:
                self.send_static(url.path)
        except ValueError as error:
            self.send_json({"error": str(error)}, 400)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        url = urlparse(self.path)
        length = int(self.headers.get("Content-Length") or 0)
        body = json.loads(self.rfile.read(length) or b"{}")
        try:
            if url.path == "/api/start":
                self.send_json({"dir": self.dash.runner.start(*start_args(body))})
            elif url.path == "/api/stop":
                self.send_json({"stopped": self.dash.runner.stop()})
            else:
                self.send_json({"error": "no such route"}, 404)
        except ValueError as error:
            self.send_json({"error": str(error)}, 409)

    def send_static(self, path):
        name = "index.html" if path in ("/", "") else os.path.basename(path)
        full = os.path.join(STATIC_DIR, name)
        if not os.path.isfile(full):
            self.send_error(404)
            return
        types = {".html": "text/html", ".js": "text/javascript", ".css": "text/css"}
        with open(full, "rb") as f:
            data = f.read()
        self.send_response(200)
        self.send_header("Content-Type", types.get(os.path.splitext(name)[1], "text/plain"))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(data)

    def api_runs(self, q):
        self.send_json({"runs": self.dash.run_dirs(), "live": self.dash.live_dir(),
                        "geometry": footprint_geometry(), "missions": MISSIONS,
                        "backends": BACKENDS})

    def api_run(self, q):
        model = self.dash.model(q.get("dir"))
        with model_lock(model):
            payload, _ = model.delta(None)
        self.send_json(payload)

    def api_heat(self, q):
        model = self.dash.model(q.get("dir"))
        with model_lock(model):
            self.send_json(model.heat(float(q.get("t", 0))))

    def api_live(self, q):
        """Server-Sent Events: a delta of the run every LIVE_PERIOD_S, plus the runner status."""
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        cursor, run_dir, checked = None, None, 0.0
        while True:
            status = self.dash.runner.status()
            if time.time() - checked > 2.0:          # the run roots are walked every 2 s only
                checked = time.time()
                newer = self.dash.live_dir()
                if newer and newer != run_dir:       # a new live run; a finished one stays shown
                    run_dir, cursor = newer, None
            status["live_dir"] = run_dir
            message = {"status": status}
            if run_dir and os.path.isdir(run_dir):
                model = self.dash.model(run_dir)
                with model_lock(model):
                    message["run"], cursor = model.delta(cursor)
                    if model.latest_wall:
                        # where the page draws: LIVE_LAG_S (gui/static/app.js) behind the newest
                        message["heat"] = model.heat(model.latest_wall - model.base_wall - 1.0)
            self.wfile.write(b"data: " + json.dumps(message).encode() + b"\n\n")
            self.wfile.flush()
            time.sleep(LIVE_PERIOD_S)


def start_args(body):
    mission, backend = body.get("mission"), body.get("backend")
    seed, n = int(body.get("seed", 1)), int(body.get("n", 3))
    if mission not in MISSIONS or backend not in BACKENDS or not 1 <= n <= 8 or seed < 0:
        raise ValueError("bad start parameters")
    return mission, seed, n, backend


def default_run_roots():
    roots = [os.path.join(ROOT, "runs"), os.path.join(ROOT, "run", "dash")]
    return [r for r in roots if os.path.isdir(r)] or [ROOT]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8800)
    ap.add_argument("--host", default="127.0.0.1", help="e.g. a VPN address to watch from elsewhere")
    ap.add_argument("--runs", nargs="*", help="directories searched for run directories")
    args = ap.parse_args(argv)
    os.makedirs(os.path.join(ROOT, "run", "dash"), exist_ok=True)
    Handler.dash = Dashboard(args.runs or default_run_roots())
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True     # an open SSE stream must not block Ctrl-C
    print(f"gui: http://{args.host}:{args.port}  (observer only: reads log files)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
