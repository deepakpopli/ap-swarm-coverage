# Report: resilient multi-drone coverage on ArduPilot SITL

For each requirement I state what I built, why I built it that way, and where the evidence is. Every measured number comes from a file under `results/` and is given as median [min, max] over runs unless I say otherwise. The reasons for the larger design choices are in [DECISIONS.md](DECISIONS.md). `./setup.sh --build` builds the stack, and `./run_eval.sh` runs the whole evaluation headless.

Conventions: the area is 300 m x 300 m, there are N = 3 drones unless stated, and the drone ids are 1..N. L is the moment of the loss, and P is a run's own coverage in the 60 s before it. A seed fixes the victim, the kill time and the dropped packets, so the same seed always gives the same faults.

## Summary

- **Mission A** (17 seeded headless Gazebo runs, one drone killed in each): coverage is 97.6 % [97.0, 97.8] before the loss and 94.3 % [92.8, 95.7] after recovery. That is 0.99 [0.94, 1.01] of a no-loss two-drone run and 1.45 [1.36, 1.47] times the (N-1)/N share.
- The ground declares the dead drone lost 12.1 s [11.8, 12.2] after the kill and re-plans once. In 12 of 17 runs coverage fell below 90 % of its pre-loss value and came back, after 65.8 s [37.2, 77.1]; in the other 5 it never fell that far. No two drones came closer than 8 m.
- The loss is still visible. The dip is 83.3 % at the median and 66.9 % in the worst run, and some cells go unseen for up to 142.5 s, against about 68 s normally. At 50 m the field is also over-provisioned (a no-loss two-drone run holds 95.1 % against 97.0 % for three), which is part of why the loss looks mild.
- **Mission B**: once the survivors' new split runs, the priority zone is seen within 30 s in every run. It is not held during the change-over or when the zone's own drone is lost (up to 57.3 s), and coverage of the whole area stays at about 79 %.
- **Mission C**: a drone that is silent for 10 s is not declared lost and needs no re-plan.

So the system does degrade gracefully, at a cost I chose deliberately: a 12 s wait before re-planning, so that a drone that is only silent is not given up.

## 1. Simulation

**What.** Every drone is its own `arducopter` SITL process, and all of them fly in one Gazebo Harmonic world with the ardupilot_gazebo iris model (`sim/launch.py`). Lock-step is off, because with it on a single killed autopilot froze the whole world. The middleware is plain pymavlink, without ROS.

**GUIDED at 20 Hz.** Each agent sends one position target per tick at `CONTROL_HZ = 20` (`swarm/config.py`). Measured over every evaluation run, the slowest agent sent at 20.0 Hz and the worst 99th-percentile setpoint period was 0.051 s [0.050, 0.051].

**12 m/s through ArduPilot parameters.** `swarm/mav_vehicle.py` sets `WP_SPD = 11.5` m/s before arming. It reads every parameter back, refuses to arm on a mismatch and logs the values. I send position targets rather than velocity targets, because in GUIDED mode WP_SPD caps only position targets. The autopilot therefore enforces the cap; my safety filter also clips its own output to 11.5 m/s, but that is not what holds the limit.

I chose 11.5 rather than 12 because the vehicle overshoots WP_SPD after braking. At `PSC_NE_JERK` 12 it overshot after a brake, so I set it to 4 (comment at `swarm/mav_vehicle.py:41`). The maximum reported velocity in the evaluation was 11.50 [11.49, 11.50] m/s. Speeds computed from 1 s differences of reported positions reach 12.01 m/s in some grid runs (`results/gazebo_grid_A/results.md`); that is 0.01 over the limit on a 1 s difference of reported positions, while the autopilot's own reported velocity never exceeded 11.50 m/s. I report it rather than round it away.

Every failsafe that could choose RTL (return to launch) is set to Land or off, because RTL would fly a drone straight across the other drones' lanes.

## 2. Architecture, and what happens when each component dies

The design is hybrid. What needs the whole picture runs on the ground, and what keeps the drone safe runs onboard.

![Architecture](docs/architecture.png)

| Process | Where | Does |
|---|---|---|
| `arducopter` x N | drone (autopilot) | flight control, the speed cap |
| `swarm/agent.py` x N | drone (companion computer) | coverage controller, safety filter, STATE at 5 Hz |
| `swarm/ground.py` | ground station | liveness, the allocator, the blacklist, START |
| `sim/radio.py` | stands in for the air link | loss, delay, blackouts |

The safety filter and the coverage controller run onboard, so they keep working when the ground or the radio fails. The allocator runs on the ground because it needs every drone's state and is not time-critical. It sends only lanes and slots, so a late or lost ASSIGN costs coverage, not separation. ASSIGN is re-sent every second with an epoch (a plan version number), so each lost packet costs one more second.

| What dies | What happens | Measured |
|---|---|---|
| An autopilot (Mission A) | The agent stops sending and exits after 2 s; the ground marks it suspect after 2 s and lost after 12 s, then re-plans | every survivor on its new plan 12.4 s [12.2, 15.3] after the kill |
| An agent process | The autopilot loses the agent's heartbeat and lands after 3 s; the rest is as above | `agentdeath`: lost 12.1 s after the kill, coverage after 95.5 % |
| The payload | The drone lands itself and reports `failsafe`; the ground treats it as lost at once | lost 1.3 to 1.5 s after the failure (`gazebo_payload/`) |
| The ground | The drones keep flying their last plan and the safety filter keeps working, but nothing re-plans | `groundloss`: 75.1 % after a loss, not recovered in the run; still 1.15 x the (N-1)/N share |
| The radio | After 3 s without hearing anyone a drone is "deaf": it stops at its private hold altitude (60, 40, 70 m for ids 1, 2, 3; cruise is 50 m) | `radiooutage` (10 s): 7.2 s deaf per drone, nobody lost, no re-plan, 96.4 % |

A restarted ground listens for 3 s and jumps its epoch 1000 above any it hears, so the drones accept its plans (tested in `tests/test_ground.py`, not flown). The ground is therefore the single point of failure for re-planning, which I return to under Limitations.

## 3. Sensing

**Payload.** The modelled sensor is a SIYI A8 mini gimbal camera pointing straight down (`swarm/sensor.py`, which links the product page and the manual). It weighs 95 g and is sold for small multirotors. Its horizontal field of view is 81 deg, and every recording mode is 16:9, so the vertical field of view is 51.3 deg. At 50 m the footprint is 85.4 m across the track and 48.0 m along it, and the rectangle turns with the drone's logged yaw.

**Range and rate.** Anything inside the footprint counts as seen. I do not model resolution, so "seen" means "imaged", not "a person detected". At 50 m the 4K mode gives about 2.2 cm per pixel, so 50 m is the range I use. Update rate: I count a sighting at each logged autopilot position (20 Hz), not at the camera's frame rate. At 11.5 m/s the drone moves 0.6 m between rows against a 48 m footprint, so the difference is small. `analysis/metrics.py` computes coverage only from each drone's own log and footprint, on a 2 m grid.

## 4. Allocation

**What.** `swarm/allocator.py` is a pure function of (live ids, last positions, blacklist, previous plan), plus each drone's velocity and active slot, used to predict where it starts a new plan, and the priority zone in Mission B. The area is divided into fixed north-south lanes at most one footprint wide, which gives 4 lanes at x = 37.5, 112.5, 187.5 and 262.5 m. The lanes are cut into contiguous groups, one per drone, choosing the cut whose longest loop is shortest; with three drones that is 1 + 1 + 2 lanes. Each drone gets a closed back-and-forth loop over its lanes, with slots at most 79 m apart. Ties break by id, and the same inputs always give the same plan (`tests/test_allocator.py`).

![Allocator](docs/allocator.png)

**Why.** The lanes never move; only their owners change. A re-plan therefore does not disturb drones whose share stays the same: such a drone keeps its old plan and its active slot, unless one of its slots is blacklisted; only the epoch number changes. The reasons for fixed lanes and for three drones are in DECISIONS.md.

**Re-plan.** The ground re-plans on a lost drone, on a rejoin, or on an accepted INFEASIBLE report. Measured: the changes beyond the one forced by the loss were 0 [0, 0] in every run.

**Assignments only:** shares and slots, never velocities. ASSIGN has one more field, a speed limit, used only by the Mission B phase hold. It is a cap the coverage controller respects, not a velocity command, and in Mission A it is always 0 (= cruise).

## 5. Coverage and redistribution

**Coverage controller** (`swarm/coverage.py`, onboard). It flies straight at the active slot at cruise speed, slowing only before sharp turns, moves on when it is within 5 m of it, and loops after the last slot. On a new plan it starts at the slot ahead, so a re-plan does not turn it round unless the way ahead is too long. Its output is a horizontal velocity, and the safety filter decides what reaches the autopilot.

**Redistribution.** When a drone is lost, the two survivors split the four lanes 2 + 2. The first survivor is on its new loop 12.3 s [12.1, 12.4] after the kill. `assignments.png` in each results set with a `results.md` shows the shares over time: one step at the re-plan, and flat otherwise.

**Why 12 s** (DECISIONS.md, "Dead or just silent?"). I also flew seeds 1-5 with a 4 s timeout (`results/lost4/`): loss is declared about 4 s after the kill, but in Mission C the silent drone is declared dead and comes back, costing two re-plans.

## 6. Safety filter

**Where.** `swarm/safety.py` runs onboard, every tick, between the coverage controller and the autopilot. It uses only the drone's own state and the last SWARM_STATE heard from each neighbour.

**What it enforces.** Any two drones stay at least 8 m apart in 3D. For each neighbour, either their altitude bands are 8 m apart, or the drone stays out of a keep-out disc around the neighbour's predicted position and closes on it no faster than it can still stop. Both drones of a pair apply the same rule and each takes half. The disc grows with the age of the neighbour's last state (9.0 m when fresh, 28.1 m at 2 s, up to 88.5 m), and a neighbour silent for 12 s is dropped. If the drone could not stop before a disc at its actual velocity, it brakes to zero.

**When it cannot satisfy every constraint.** The filter first removes the excess closing speed toward each disc; head-on, where almost no sideways speed is left, it also steps to the right (`SIDESTEP_BELOW_MS` in `swarm/safety.py`). If that breaks another disc's limit, it scales the whole velocity down until none is broken, down to zero if needed. So when the constraints conflict the drone slows and stops; it does not escape. Coverage gives way, separation does not. A drone held still like this reports its slot infeasible after 15 s, goes to its hold altitude, and the ground re-plans around it (section 7).

Inside a disc it may never close further, but nothing pushes it out either. Take-off is outside the filter: a drone climbs straight up only after listening 1 s and seeing no neighbour within 9 m.

**Evidence.** Time below 8 m is 0 s in every Gazebo run, and the minimum 3D separation from t = 10 s was 60.4 m or more in the evaluation. In the `headon` scenario two drones are sent straight at each other at the same altitude, where without the filter they would pass at about 0 m. They passed at 23.9 m, the filter braked for 0.25 s and 0.40 s, and there was no deadlock. This is one run, so it shows the filter working, not a measured margin.

The braking in the evaluation (0.4 s [0.0, 13.9] per run, all drones together) I attribute to the dead drone's growing disc before it is dropped at 12 s; I did not measure this separately. It costs a little coverage and nothing in separation.

## 7. Feasibility feedback

**Detection.** A slot not reached 15 s after it became active is infeasible (`swarm/coverage.py`). Slots are at most 79 m apart, so an unblocked drone reaches any slot in under 10 s. The drone then holds and repeats SWARM_INFEASIBLE every second until a newer epoch arrives.

**No reissue, no oscillation.** The ground accepts the first report, blacklists the (drone, slot) pair for 60 s and re-plans; repeats are ignored. The allocator never issues a blacklisted pair. When the blacklist expires, nothing re-plans.

**Evidence.** In the `obstacle` scenario, an obstacle that only one drone knows about appears on one of its slots. The ground accepted one INFEASIBLE and re-planned once; churn was 0.08 changes per drone per minute, coverage 97.0 %, and there was no second report after the expiry (`results/scenarios/obstacle.metrics.json`).

## 8. Faults and comms

**The kill.** `run_mission.py` sends SIGKILL to the victim's `arducopter` at a seeded time in [60, 90] s. There is no landing and no goodbye. In Gazebo the dead model is then removed from the world, because otherwise the plugin waits for an autopilot that never answers and slows the whole world.

**The radio.** `sim/radio.py` is one UDP relay. It drops each copy per receiver with probability 0.2 and forwards the rest 200 ms later, with a seeded random stream per (sender, receiver) pair. Measured over the evaluation: loss 20.0 % [19.4, 20.5] and mean delay 0.200 s.

**No side channel.** Agents talk to their own autopilot directly and to everyone else only through the radio. Stale states are handled by the safety filter (section 6).

## Evaluation: Mission A

`./run_eval.sh` flies 17 seeds (1-17) of 180 s each, all of them reported, plus one no-loss N = 3 baseline and one no-loss N = 2 reference. "After recovery" runs from the recovery time to the end of the run. The definitions are in `analysis/metrics.py` and `results/gazebo_50m/results.md`.

| Metric | Before the loss | After recovery |
|---|---|---|
| Coverage, mean (%) | 97.6 [97.0, 97.8] | 94.3 [92.8, 95.7] |
| Coverage, lowest (%) | 95.7 [94.3, 95.8] | 88.4 [87.6, 91.5] |
| Worst revisit gap (s) | 67.7 [67.5, 69.0] | 107.8 [72.8, 142.5] (from the loss on) |
| Steady revisit gap (s) | 67.4 [58.2, 69.0] | 68.0 [67.1, 71.0] (from lost declared on) |
| Recovery time (s) | - | 65.8 [37.2, 77.1] over the 12 runs that fell below 0.9 x P; 5 runs never did |
| Assignment churn (changes / drone / min) | 0.00 [0.00, 0.00] | 0.31 [0.26, 0.59] (from the loss on) |

- **(N-1)/N check.** (N-1)/N x P is 65.1 % [64.7, 65.2], and coverage after recovery is 1.45 [1.36, 1.47] times that. This check is easy at 50 m, where the field is over-provisioned: even the worst dip, 66.9 %, is above it. The harder comparison is the no-loss two-drone run, and coverage after recovery is 0.99 [0.94, 1.01] of it.
- **Recovery** means coverage back within 10 % of its pre-loss value and staying there. The dip, 83.3 % [66.9, 91.5], comes late because coverage has a 60 s memory: the dead drone's last sightings fade out of the window while the survivors are already on their new loops.
- **By victim** (mean [min, max] over that victim's runs). Losing drone 3, which flew two lanes, hurts most: coverage from L + 60 s is 91.5 % [89.0, 93.6], against 94.9 % [94.1, 95.7] for drone 1 and 94.0 % [92.7, 95.0] for drone 2.
- **Per drone and fault time** (`results/gazebo_grid_A/`, flown on earlier commits c68ee22 and f459190): each drone was killed at 60, 70, 80 and 90 s, 12 runs in all. Coverage after recovery is 94.5 % [91.6, 95.8], which is 0.99 [0.96, 1.00] of the N-1 reference.

Loss timeline: `docs/loss_timeline.png`. Each results set with a `results.md` also contains `coverage_vs_time.png`, `before_after.png` and `recovery_steps.png`. The animation is `results/replay_representative.mp4` (47 s, includes the dropout), showing seed 14, the median run.

## Mission B: priority zone

- **What flies.** With three drones, one flies a short out-and-back lane through the 60 m x 60 m zone and the other two share the rest. Two survivors fly a "butterfly" instead: each flies a two-lane loop whose centre lane sees the whole zone, and the two are kept half a lap apart by a ground phase hold (a speed cap of 0.8 or 0.6 x cruise).
- **Result.** With the phase hold, once the new split runs (from L + 40 s), the zone's oldest cell is 23.9 s [23.3, 26.1], under 30 s in all 12 runs. Coverage of the whole area stays at 79.0 % [76.3, 81.1] (from L + 60 s), against 68.6 % with the old single-owner split. Outside the zone the loss is not recovered: 0 of 12 runs return to 0.9 x P (`results/gazebo_grid_B/`).
- **Not held:** during the change-over (up to 33.8 s); when the zone's own drone is lost (50.3 s on average, up to 57.3 s: the 12 s timeout plus travel); during a long blackout of the zone drone; and with one survivor.

## Mission C: comms blackout

Every message to and from one drone is dropped for 10 s. With the 12 s timeout the drone is not declared lost in this run: it goes deaf after 3 s, holds for 7.2 s at its hold altitude, and is covering again 1.7 s after the blackout. There is no re-plan, and coverage is 97.7 % (`results/scenarios/C_s1.metrics.json`). With a 4 s timeout the same run declares the drone lost and re-plans twice (`results/lost4/C_s1.metrics.json`). Telling a silent drone from a dead one over the radio alone needs a timeout longer than the longest silence (DECISIONS.md).

## Live demo (not part of the evaluation)

`./demo.sh` runs one mission with the Gazebo window, drone cameras and the web view. None of it runs in the evaluation, and none of it feeds the swarm.

## Limitations and next steps

- **Lane tracking.** The coverage controller flies at slots, not along the lane line. Tracking the lane would make the footprint overlap more predictable.
- **The ground is a single point of failure for re-planning.** While it is down, a lost drone's lanes stay unflown (75.1 % in `groundloss`). The next step, in progress and not in this repo, is a leader-follower fallback in which the lowest-id live drone runs the ground code onboard.
- **No EKF under Gazebo.** The JSON backend sets `AHRS_EKF_TYPE 10`, so positions are the simulator's truth, and GPS faults and estimate errors are not exercised.
- **The dead drone is removed from Gazebo**, so the others never meet a falling airframe.
- **N = 3 only.** The allocator handles any N (unit tests only), but I flew only three drones, plus the two-drone no-loss reference.
- **Small samples.** There are 17 + 12 runs for Mission A, 12 per Mission B set, and one run per scenario. I give medians with min and max, not confidence intervals.
- **One-way radio failures** (a drone that hears but is not heard) are not emulated; the blackouts are two-way.

## Reuse and citations

- ArduPilot (ArduCopter SITL), GPLv3, and ardupilot_gazebo (plugin, iris model), at pinned commits in `setup.sh`, not modified.
- Gazebo Harmonic; pymavlink and mavgen (`swarm/swarm_mav.py` is generated from my `swarm/swarm.xml`).
- Gazebo Fuel models by OpenRobotics (CC0 1.0, live view only); MediaMTX, ffmpeg, numpy, matplotlib, Pillow.

I did not copy a swarm framework. The allocator, coverage controller, safety filter, radio emulator, ground station and metrics are written for this project.

## How I used AI

At first I thought about the problem myself. I read the problem statement again and again, tried to gather as much information as I could, and came up with a few architectures and ideas of my own.

To validate those ideas, I used multiple AI agents to run them as experiments in simulation for me. I evaluated the results and then decided which idea to keep. So I made a lot of prototypes before sticking to one approach.

After that I used AI for extensive testing. I wanted the system to not fail in a lot of conditions, so I stress tested the whole system. I automated the simulation pipeline with AI and ran the simulations many times, so that it also holds up in future demos. I also used AI to build a lot of local tools for development, such as testing scripts and stress-test scripts.

While developing the project with multiple AI agents running, I had the issue of a lot of bloated and junk code and files produced by the agents. So I reviewed the whole codebase properly, identified the junk code and files, and cleaned them up. I also tested things myself multiple times. The aim was to make the codebase smaller, more maintainable and carefully engineered, so it is easier for humans to read as well.

The web GUI was not part of the problem statement; it was an idea I had during development. At first I meant it as a personal tool for the project, but then I found it was a much easier way to show the drones and the overall system. So I added a few more things, improved it a bit more and decided to ship it with the project. It helps a lot while a simulation is running, to see what is going on.
