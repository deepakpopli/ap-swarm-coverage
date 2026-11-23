# Resilient multi-drone coverage on ArduPilot SITL

Three quadrotors, each running its own ArduCopter SITL inside one Gazebo Harmonic world, keep a
300 m x 300 m area covered with a downward camera (SIYI A8 mini footprint, 50 m altitude). A
ground station assigns each drone fixed lanes and nothing else. Each drone turns its lanes into
a velocity, passes it through an onboard safety filter that keeps 8 m from every other drone,
and sends one GUIDED setpoint to its own autopilot 20 times a second. Every message between
drones and the ground goes through a radio emulator I wrote (20 % loss, 200 ms delay). One
autopilot is SIGKILLed at a seeded time; the ground notices the silence and gives its lanes to
the survivors. The evaluation measures how much coverage drops and how fast it comes back.

The design and the numbers are in [REPORT.md](REPORT.md). Why I chose what I chose is in
[DECISIONS.md](DECISIONS.md).

## Architecture

```
 ground station                    swarm/ground.py  (liveness, re-plan)
   allocator                       swarm/allocator.py  (lanes and slots, never velocities)
      |  SWARM_ASSIGN   ^ SWARM_INFEASIBLE      MAVLink dialect: swarm/swarm.xml
      v                 |
 radio emulator                    sim/radio.py  (UDP relay: 20 % loss, 200 ms, blackouts)
      |  SWARM_STATE between all nodes, 5 Hz      swarm/net.py, swarm/protocol.py
      v
 per drone, onboard                swarm/agent.py  (one process per drone, 20 Hz)
   coverage controller             swarm/coverage.py  (slots -> nominal velocity, 15 s timer)
   safety filter                   swarm/safety.py  (8 m, stale states, brake, deaf, hold)
   MAVLink to own autopilot        swarm/mav_vehicle.py  (GUIDED position targets, parameters)
      |
 ArduCopter SITL (GUIDED)          one arducopter per drone, sim/launch.py
 Gazebo Harmonic, iris model       sim/gazebo_world.py
```

`run_mission.py` is test equipment: it starts everything, injects the fault and stops
everything. It never sends anything to the swarm. `analysis/` computes the metrics offline
from each drone's own log.

## Requirements

- Ubuntu 22.04 or 24.04
- Python >= 3.10 (`pymavlink`, `numpy`, `matplotlib`, `Pillow` from `requirements.txt`; `pytest`
  for the tests)
- Gazebo Harmonic (`gz sim` 8.x)
- ArduPilot at commit `9165d22419406556fc9815d9f4e3e299e13713a8` and ardupilot_gazebo at
  commit `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`. `setup.sh --build` checks out both. The
  code depends on parameter names of this ArduPilot commit; another commit may refuse to arm.
- `ffmpeg` (the animation), `flock` (util-linux, one simulator at a time)
- Optional, live demo only: MediaMTX for the drone video in the browser (Pillow, for the photo
  mosaic and the camera bridge, is in `requirements.txt`).

## Setup

```
sudo apt install python3-pip python3-venv ffmpeg
# Gazebo Harmonic: https://gazebosim.org/docs/harmonic/install_ubuntu
sudo apt install gz-harmonic libgz-sim8-dev rapidjson-dev libopencv-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt pytest
./setup.sh --build      # clones and builds ArduCopter SITL and ardupilot_gazebo at the pinned commits
./setup.sh              # checks everything and prints OK / MISSING with the fix for each
```

On a new machine ArduPilot needs its own prerequisites first
(`Tools/environment_install/install-prereqs-ubuntu.sh -y` in the ArduPilot checkout, then a new
shell). `ARDUPILOT_HOME` and `ARDUPILOT_GAZEBO` choose where they go (default `~/ardupilot`,
`~/ardupilot_gazebo`). `setup.sh` never runs sudo.

## The evaluation: one command

```
./run_eval.sh
```

Headless Mission A, N = 3, 180 s per run, one run at a time: 16 runs of at least 5 minutes each
with start-up and teardown [update after the reference runs: time the full command]. Seeds 1-12, then
16 and 17 instead of 13-15: seeds 13, 14 and 15 draw drones 3, 3 and 1 again, and 16 and 17 give
every drone four or five runs. It also flies one no-loss N = 3 baseline and one no-loss N = 2
reference. Then it writes the table, plots and one animation to `results/gazebo_50m/`. Raw runs
go to `runs/gazebo_50m/` (git-ignored).

- Re-running continues where it stopped: a finished run is skipped. `FORCE=1` flies them again.
- `SEEDS="1 2 3 4 5" ./run_eval.sh` flies fewer seeds.
- A run counts only if Gazebo's real-time factor is at least 0.95 at its start and end. Slower,
  the simulator does not keep up with the wall clock the agents run on. A failed or invalid
  attempt is kept in `runs/gazebo_50m/failed/` and retried up to two times;
  `results/gazebo_50m/attempts.txt` lists every attempt.

## Missions B and C, and the scenarios

```
python3 run_mission.py --mission B --backend gazebo --n 3 --seed 1 --out runs/b1
python3 run_mission.py --mission C --backend gazebo --n 3 --seed 1 --out runs/c1
python3 run_mission.py --help          # --victim, --kill-at, --zone, --scenario ...
./run_scenarios.sh                     # swap, obstacle, agentdeath, payloadfail, groundloss,
                                       # radiooutage and Mission C, one run each -> results/scenarios/
python3 -m analysis.metrics runs/b1    # metrics.json for one run
```

`SWARM_LOST_S=4` shortens the lost timeout from 12 s to 4 s; `results/lost4/` is that
comparison. `--backend fake` runs the whole stack without a simulator, for quick checks.

## Live demo (not part of the evaluation)

```
tools/show_gazebo.sh                              # Mission A seed 3 with the Gazebo window
SWARM_DEMO=1 SWARM_DEMO_GROUND=field tools/show_gazebo.sh --mission B --n 3 --seed 1
SWARM_DEMO=1 SWARM_DEMO_CAMERA=1 tools/show_gazebo.sh   # + a nadir camera per drone (MediaMTX)
python3 -m gui                                    # web view on http://localhost:8800
```

`SWARM_DEMO=1` adds coloured drones, the field border and lanes; `SWARM_DEMO_GROUND=field` puts
farmland, trees, cars and a parked aircraft around it (Gazebo Fuel models, downloaded once).
`SWARM_DEMO_CAMERA=1` gives each drone a camera with the A8 mini's field of view; with MediaMTX
on the PATH (or `MEDIAMTX=`) the web view plays the three streams over WebRTC and saves a frame a
second. In the web view, "shade" shows how long ago each cell was seen and "photos" lays the
saved frames on the ground as a mosaic. A drone removed in the Gazebo window (right click,
Remove) is a loss the swarm handles. `python3 -m gui` shows the newest run under `runs/`, live while
it is written, or a finished run as a replay; it can also start a mission from the page.

Recording of the live demo: [video: to be named].

## Tests

```
python3 -m pytest -q          # fast tests; the real-time ones: python3 -m pytest -q -m slow
```

`tests/test_golden_run.py` pins the logs of a fake-backend Mission A run.

## Results index

| Folder | What |
|---|---|
| `results/gazebo_50m/` | The evaluation from `./run_eval.sh`: 14 seeds, baseline, N-1 reference, `replay_seed_1.mp4` |
| `results/gazebo_grid_A/` | Mission A, each drone killed at 60, 70, 80 and 90 s (12 runs) |
| `results/gazebo_grid_B/` | Mission B on the same grid: the butterfly with the phase hold |
| `results/gazebo_grid_B_nohold/` | The same, butterfly without the phase hold |
| `results/gazebo_grid_B_old/` | The same, one zone owner and no butterfly (older code) |
| `results/gazebo_payload/` | Payload failure: each drone lands itself at 75 s |
| `results/lost4/` | Mission A seeds 1-5 and Mission C with a 4 s lost timeout |
| `results/scenarios/` | One run each of `run_scenarios.sh`, plus one Mission B run |
| `results/replay_representative.mp4` | Top-down animation of one run with the dropout (45 s) |
| [video: to be named] | Screen recording of the live demo |

`gazebo_50m` and the grids have `results.md` (the table, median [min, max]), `attempts.txt` and
the figures (`coverage_vs_time.png`, `before_after.png`, `recovery_steps.png`, `age_maps.png`,
`assignments.png`, `separation.png`; the grids also `by_drone.png`). The other folders have one
`metrics.json` per run and `attempts.txt`.
