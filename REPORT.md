# Report: resilient multi-drone coverage on ArduPilot SITL

This follows the task's requirements 1 to 8, then the evaluation, Missions B and C, limitations,
reuse and AI use. For each requirement: what I built, why, and where the evidence is. Every number
comes from a file under `results/`, quoted as median [min, max] over runs like the files do. The
reasoning behind the bigger choices is in [DECISIONS.md](DECISIONS.md); I link to it instead of
repeating it.

Conventions: the area is 300 m x 300 m in a local ENU frame, origin at the south-west corner
(`swarm/config.py`). Drone ids are 1..N and equal each autopilot's MAV_SYSID; the ground is 255. N =
3 unless stated. Mission time t = 0 is 60 s after every drone is on its coverage loop, so the 60 s
coverage window is already full at t = 0.

## Summary

Over 14 seeded Gazebo runs of Mission A, mean coverage is 97.7 % [97.2, 97.8] in the 60 s before the
loss and 94.2 % [92.8, 95.7] after recovery. That is [update after the reference runs] of what a
no-loss two-drone run holds, and 1.45 [1.36, 1.47] times the (N-1)/N share the task asks for. The
ground declares the dead drone lost 12.1 s [11.8, 12.2] after the kill and re-plans once. In 10 of
14 runs coverage fell below 90 % of its pre-loss value and came back, after 63.5 s [37.2, 76.4]; in
4 runs it never fell that far. No two drones came closer than 8 m (`results/gazebo_50m/results.md`).

So yes, it degrades gracefully, at a cost I chose: a 12 s wait before re-planning, so that a drone
that is only silent (Mission C) is not given up.

## 1. Simulation

**What.** Every drone is its own `arducopter` SITL process (instance id - 1, MAVLink on TCP 5760 +
10 x instance), all inside one Gazebo Harmonic world with the ardupilot_gazebo iris model and JSON
backend (`sim/launch.py`, `sim/gazebo_world.py`). Lock-step is off: with it on, one killed autopilot
froze the whole world. Middleware is plain pymavlink, no ROS (why: DECISIONS.md, "mavlink for
everything"). ArduPilot's built-in physics (`--backend sitl`) and a no-simulator fake (`--backend
fake`) run the same code, for checks and tests.

**GUIDED at 20 Hz.** Each agent sends one position target per tick at `CONTROL_HZ = 20`
(`swarm/config.py`). Measured in every evaluation run: slowest agent 20.0 Hz, worst p99 setpoint
period 0.051 s [0.050, 0.051], longest single period 0.089 s (`results/gazebo_50m/results.md`;
`results/gazebo_grid_A/results.md`: 20.0 Hz, p99 0.050 [0.050, 0.051] s).

**12 m/s through ArduPilot parameters.** `swarm/mav_vehicle.py` sets `WP_SPD = 11.5` m/s before
arming, reads every parameter back and refuses to arm on a mismatch; the read-back values are logged
in each run's `takeoff_done` event. The setpoints are position targets placed ahead of the drone,
not velocity targets, because WP_SPD caps position targets in GUIDED and not velocity targets
(measured; docstring of `swarm/mav_vehicle.py`). So the autopilot enforces the cap, not my code.
11.5 and not 12, because the vehicle overshoots WP_SPD by a small tracking error; a lower
`PSC_NE_JERK` keeps that small. Measured maximum ground speed in the evaluation: 11.50 [11.49,
11.50] m/s from the reported velocity and 11.54 [11.44, 11.99] m/s from position differences over 1
s (`results/gazebo_50m/results.md`). In the grid sets the position-difference maximum reaches 12.01
m/s in some runs (`results/gazebo_grid_A/results.md`): 0.01 over the limit on a 1 s difference of
reported positions. I report it rather than round it away.

Every failsafe that could choose RTL is set to Land or off (`PARAMS` in `swarm/mav_vehicle.py`): RTL
would fly a drone straight across the others' lanes.

## 2. Architecture, and what happens when each component dies

Hybrid. What needs the whole picture runs on the ground; what keeps the drone safe runs onboard.

| Process | Where | Does |
|---|---|---|
| `arducopter` x N | drone (autopilot) | flight control, the speed cap |
| `swarm/agent.py` x N | drone (companion computer) | coverage controller, safety filter, STATE at 5 Hz, its own log |
| `swarm/ground.py` | ground station | liveness, the allocator, the blacklist, START |
| `sim/radio.py` | stands in for the air link | loss, delay, blackouts |

The safety filter and the coverage controller are onboard so separation and coverage go on when the
ground or the radio does not. The allocator is on the ground because it needs every drone's state
and is not time-critical; it sends only lanes and slots, so a late or lost ASSIGN can never be a
dangerous command. ASSIGN is re-sent every second and is idempotent by epoch.

What happens when one component dies:

- **An autopilot (the Mission A fault).** The agent sees its own MAVLink link fail, sends nothing
  more (so nobody gets a stale position with a fresh time stamp) and exits after 2 s
  (`swarm/agent.py`). The others stop hearing it: suspect after 2 s, lost after 12 s, then the
  ground re-plans the survivors. Measured: lost 12.1 s [11.8, 12.2] after the kill, every survivor
  holding its new plan after 12.4 s [12.2, 15.3] (`results/gazebo_50m/results.md`).
- **An agent process.** Its autopilot loses the agent's heartbeat and lands after 3 s
  (`FS_GCS_ENABLE = 5`, `FS_GCS_TIMEOUT = 3`). The rest is as above. Scenario `agentdeath`: lost
  12.1 s after the kill, one re-plan, coverage after 95.5 %, never below 90 % of before
  (`results/scenarios/agentdeath.metrics.json`).
- **The payload.** The drone lands itself and keeps talking with status `failsafe`, which the ground
  treats as lost at once. Lost 1.4 to 1.5 s after the failure in all three runs
  (`results/gazebo_payload/`, `results/scenarios/payloadfail.metrics.json`).
- **The ground station.** The drones keep flying their last assignment and keep hearing each other,
  so the safety filter works as before. Nothing re-plans while the ground is down, so a drone lost
  in that time leaves its lanes unflown. Scenario `groundloss` (ground down for good from 40 s, then
  the Mission A kill at 64 s): coverage 97.6 % before, 75.1 % mean after, dip 73.1 %, never
  recovered; still 1.15 x the (N-1)/N share (`results/scenarios/groundloss.metrics.json`). A
  restarted ground listens 3 s, jumps its epoch 1000 above any it hears and keeps t0 from its log
  (`Ground.maybe_plan_first`; tested in `tests/test_ground.py`, not flown).
- **The radio.** After 3 s without hearing anyone a drone is "deaf": it stops and goes to its
  private hold altitude (cruise 50 m, then 60, 40, 70 m for ids 1, 2, 3), so deaf drones are
  vertically apart. Scenario `radiooutage` (radio down 60-70 s): 7.2 s deaf per drone, nobody
  declared lost, no re-plan, mission mean coverage 96.4 %, no time below 8 m
  (`results/scenarios/radiooutage.metrics.json`).

The ground is the single point of failure for re-planning. See Limitations.

## 3. Sensing

**Payload.** A SIYI A8 mini gimbal camera pointing straight down (`swarm/sensor.py`; product page
and user manual linked there). It is sold for small multirotors [owner: add its weight]. Horizontal
field of view 81 deg; every recording mode is 16:9, so the vertical field of view is 51.3 deg
(derived). At 50 m above flat ground the footprint is 85.4 m across the track x 48.0 m along it. The
rectangle turns with the drone's logged yaw.

**Range and rate: what I assumed.** The footprint is geometric. I assume anything inside it counts
as seen; I do not model resolution, so "seen" means "imaged", not "a person detected". At 50 m the
4K mode gives about 2.2 cm per pixel across (85.4 m over 3840 pixels), so 50 m is the range I use.
Coverage is evaluated on the positions the autopilot reports at 20 Hz (one log row per control
tick), not at the camera's frame rate; at 11.5 m/s a drone moves 0.6 m between rows against a 48 m
footprint, so the difference is small.

**Coverage only from footprint and logged position.** `analysis/metrics.py` reads each drone's own
log (`agent_<id>.csv`, positions as its autopilot reports them) and nothing else, on a 2 m grid,
once a second. A dead drone's rows stop at its loss.

## 4. Allocation

**What.** `swarm/allocator.py`, a pure function of (live ids, last positions, blacklist, previous
plan). The area gets fixed north-south lanes one footprint wide: ceil(300 / 85) = 4 lanes at x =
37.5, 112.5, 187.5 and 262.5 m. They are cut into contiguous groups, one per drone, choosing the cut
whose longest loop is shortest; with three drones that is 1 + 1 + 2 lanes. Each drone gets its share
(a rectangle) and an ordered list of slots: a closed boustrophedon loop over its lanes, with points
no more than 79 m apart. Ties break by id; sorted ids everywhere. The same inputs give the same plan
(`tests/test_allocator.py::test_determinism`).

**Re-plan.** On a lost drone, a rejoin, or an accepted INFEASIBLE report (`swarm/ground.py`). Lanes
never move; only owners change. A drone whose share did not change keeps its old plan object, so it
gets no new epoch (`test_previous_plan_kept_for_unaffected_agents`). Measured: "changes beyond the
one forced by the loss" 0 [0, 0] in every evaluation run (`results/gazebo_50m/results.md`).

**Assignments only.** The allocator outputs shares and slots. SWARM_ASSIGN has one more field, a
speed limit, used only by the Mission B phase hold (below) to slow one drone to 0.8 or 0.6 x cruise.
It is a cap the coverage controller respects, not a velocity command; in Mission A it is always 0 (=
cruise). The safety filter still sits between it and the autopilot.

Why fixed lanes and three drones: DECISIONS.md, "Why 3 drones and not 4 or 5".

## 5. Coverage and redistribution

**Coverage controller** (`swarm/coverage.py`, onboard). It flies straight at the active slot at
cruise, slowing only before sharp turns, and moves on when within 5 m of it or past it. After the
last slot it loops. On a new plan it starts at the slot it already heads to, or the nearest one
ahead, so a re-plan does not turn it round. The output is a nominal horizontal velocity; the safety
filter decides what reaches the autopilot.

**Redistribution.** When a drone is lost, the two survivors split the four lanes 2 + 2. Lost 12 s
after the kill, first survivor on its new loop 12.3 s [12.1, 12.4] after it
(`results/gazebo_50m/results.md`). `assignments.png` in each results set shows the shares over time:
one step at the re-plan, flat otherwise.

Why the 12 s: DECISIONS.md, "Dead or just silent?". `results/lost4/` flew seeds 1-5 with 4 s: lost
is then declared about 4 s after the kill, but in Mission C the silent drone is declared dead and
comes back, which costs two re-plans (see Mission C).

## 6. Safety filter

**Where.** `swarm/safety.py`, onboard, every tick, between the coverage controller and the
autopilot. It uses only the drone's own state from its autopilot and the last SWARM_STATE heard from
each neighbour. Pure logic, no I/O; `tests/test_safety.py` drives it directly.

**What it enforces.** Any two drones at least 8 m apart in 3D. Per neighbour, either their altitude
bands are 8 m apart (a drone only ever targets cruise or its own hold altitude, 10 m apart), or the
filter keeps out of a keep-out disc around the neighbour's predicted position and closes on it no
faster than it can still stop. Both drones of a pair apply the same rule and each takes half.

**Stale states.** The disc grows with the age of the neighbour's last state: 9.0 m when fresh, 14.5
m at 1 s, 28.1 m at 2 s, 64.5 m at 4.6 s (by then a silent neighbour has itself gone deaf and
braked), up to 88.5 m (computed from `SafetyParams.radius_m`). Late or out-of-order states are
ignored. A neighbour silent for 12 s is dropped. A neighbour in failsafe is landing, so only the air
above its last height is free.

**Brake and deaf.** If the drone's actual velocity, not just the command, could not stop before a
disc, the filter commands zero (`brake`). If it has heard nobody for 3 s it is deaf: zero horizontal
speed at its hold altitude until it hears someone and is clear again.

**When it cannot satisfy every constraint.** The constraint per disc is a maximum closing speed. The
filter first removes the excess closing speed toward each disc, with a step to the right in a
head-on case. If that breaks another disc's limit, it scales the whole velocity down until none is
broken, down to zero if needed. So when constraints conflict, the drone slows and stops; it does not
try to escape sideways or push itself out of a disc. Coverage gives way, separation does not. A
drone held still like this still has its slot timer running, so after 15 s it reports the slot
infeasible, goes to its hold altitude (vertically clear of the cruise layer) and the ground re-plans
around it. Inside a disc it is never allowed to close further, but nothing in the filter pushes it
out either.

**Take-off is outside the filter.** A drone climbs straight up to its hold altitude only after
listening 1 s and seeing no neighbour within 9 m horizontally (8 m plus position error;
`clear_for_takeoff`). The start line is 10 m apart, so take-off is clear by construction.

**Evidence.** In the Gazebo flights the drones never came close: minimum 3D separation from t = 10 s
was 60.4 m or more in the evaluation and 24.2 m or more across every Gazebo set in `results/` (the
lowest is in `gazebo_grid_B_old`); time below 8 m is 0 s everywhere. The 10.0 m "whole flight"
minimum is the start line on the ground. The cases where the filter has to work are in
`tests/test_safety.py`: crossings at every angle including head-on at cruise, n drones converging on
one point, a 10 s blackout while a neighbour approaches, a cruiser passing under a forgotten deaf
drone, braking on actual speed, vertical separation of hold and cruise. The `obstacle` scenario
exercises it in Gazebo (below).

**Why the logs show braking.** The summed `brake` time per evaluation run is 1.0 s [0.0, 13.9] over
all drones (`results/gazebo_50m/results.md`). I went through every row flagged `brake` in the 14
runs (`agent_<id>.csv`, against the kill time and victim in `meta.json`). In mission time all of it
falls between L + 3.6 s and L + 11.9 s, where L is the kill: after the kill and before the dead
drone's state is 12 s old and dropped. 9 of 14 runs brake at all; 5 do not.

The cause is the dead drone's keep-out disc. Its last state ages, so the filter moves the disc
centre along the dead drone's last velocity (for at most 4.6 s, up to 53 m) and grows the radius:
64.5 m at 4.6 s, 88.5 m at 12 s. The brake rule lets a drone close on a disc at only half the speed
that can still stop before it, because a live neighbour would take the other half. A dead one does
not, so this is conservative: at 11.5 m/s straight at the disc, the rule brakes once the disc edge
is less than about 100 m away (`v_stop_ms` in `swarm/safety.py`). So it is not braking near the
dead drone's last position. The braking drones were 76 to 203 m from it, and the nearest live drone
was 71 m or more away. Each stop lasts 0.3 to 7.5 s. Recomputed from the dead drone's last logged
state, the brake rule fires on 848 of the 862 braking rows; the other 14 are inside the same
window, where the state a survivor last heard can be older than the last one logged. The worst run,
seed 17 (13.9 s), has both survivors braking in bursts of up to 7.1 s. This is the price of not
knowing whether the silent drone is dead or still flying. It costs a little coverage before the
re-plan and nothing in separation.

There is also braking before t = 0, in the warm-up the metrics leave out (`analysis/metrics.py`
counts mission time only): in 13 of 14 runs drone 2 brakes for 0.6 to 2.05 s while it is still at 40
to 42 m, its hold altitude, between 2.5 and 19.3 m north of the start line, with a neighbour 25 to
29 m away. That is the three drones leaving the start line for their loops. The filter slows drone 2
until the others are clear.

## 7. Feasibility feedback

**Detection.** A slot not reached 15 s after it became active is infeasible (`swarm/coverage.py`).
Slots are at most 79 m apart, under 10 s from a standing start, so a drone that is not blocked
always makes it. The timer pauses while the drone is deaf (a blackout says nothing about the slot)
but not while the safety filter blocks it: a slot the drone is kept from is exactly what the ground
must hear about. The drone then holds and repeats SWARM_INFEASIBLE every second until a newer epoch
arrives.

**No reissue, no oscillation.** The ground accepts the first report, blacklists the (drone, slot)
pair for 60 s and re-plans; repeats are ignored, and a report survives an unrelated re-plan
(`Ground.on_infeasible`). The allocator never issues a blacklisted pair: it re-spaces the points of
that leg to step around it. When the blacklist expires nothing re-plans; the pair is only free for
later plans. Unchanged shares keep their plan, so other drones see no churn.

**Evidence.** Scenario `obstacle`: an obstacle only one drone knows about appears on one of its loop
slots at 30 s, in a 240 s run so the blacklist expires inside it. The filter held the drone at the
obstacle's edge (16.9 s braking), the slot timed out, the ground accepted one INFEASIBLE, ignored
none and re-planned once; churn 0.08 changes per drone per minute, coverage 97.0 % mean, no second
report after the expiry (`results/scenarios/obstacle.metrics.json`). Unit tests:
`tests/test_coverage.py` (exactly one report per stuck slot, then hold and resend; timer pauses only
while deaf), `tests/test_ground.py` (blacklist, expiry without re-plan),
`tests/test_allocator.py::test_blacklisted_pair_never_issued`.

## 8. Faults and comms

**The kill.** `run_mission.py` sends SIGKILL to the victim's `arducopter` at a seeded time in [60,
90] s; the victim is drawn from the seed too. No landing, no goodbye. In Gazebo the dead model is
then removed from the world: the ardupilot_gazebo plugin waits for an autopilot that never answers
and slowed the whole world to about 0.6 x real time (`Fleet.kill_vehicle` in `sim/launch.py`).

**The radio.** `sim/radio.py` is one UDP relay. Every node sends every MAVLink packet to it; it
drops each copy per receiver with probability 0.2 and forwards the rest 200 ms later. Each (sender,
receiver) pair has its own seeded random stream, so the k-th message from A to B is dropped in every
run of a seed or in none. Measured over the evaluation: loss 20.0 % [19.5, 20.5], mean delay 0.200
s, max 0.202 [0.201, 0.224] s (`results/gazebo_50m/results.md`, from each run's `radio.jsonl`).
Messages are a small MAVLink 2 dialect, 51 bytes for a state (DECISIONS.md).

**No side channel.** Agents talk to their own autopilot over TCP and to everyone else through the
radio only (`swarm/protocol.py`, `swarm/net.py`). The runner reads `ground.jsonl` only to learn t0
for the fault schedule and never sends to the swarm. Gazebo knows every position, but only the live
view's overlay and camera bridge read it, and the evaluation never starts them.

**Stale states.** The safety filter uses the last state received from each neighbour, aged from its
own time stamp (section 6).

## Evaluation: Mission A

`./run_eval.sh`: 14 seeds (1-12, 16, 17; why: README), 180 s each, one no-loss N = 3 baseline and
one no-loss N = 2 reference. Definitions are in `analysis/metrics.py` and at the top of
`results/gazebo_50m/results.md`. "Before" is the 60 s before the loss; "after recovery" is from the
recovery time to the end of the run.

| Metric | Before the loss | After recovery |
|---|---|---|
| Coverage, mean (%) | 97.7 [97.2, 97.8] | 94.2 [92.8, 95.7] |
| Coverage, lowest (%) | 95.7 [94.6, 95.8] | 88.5 [88.0, 91.5] |
| Worst revisit gap (s) | 67.7 [67.5, 69.0] | 107.8 [72.8, 140.4] (from the loss on) |
| Steady revisit gap (s) | 67.4 [58.2, 69.0] | 67.9 [67.1, 70.3] (from lost declared on) |
| Assignment churn (changes / drone / min) | 0.00 [0.00, 0.00] | 0.29 [0.26, 0.59] (from the loss on) |

Coverage from L + 60 s, when the dead drone's last sightings have left the window: 94.0 % [89.0,
95.7]. Mean over the whole mission: 94.9 % [89.2, 96.4].

- **(N-1)/N check.** (N-1)/N x P is 65.1 % [64.8, 65.2]. After recovery / that share: 1.45 [1.36,
  1.47]. This check is easy at 50 m: the lowest dip of any run, 66.9 %, is above it. The harder
  comparison is the no-loss two-drone run: [update after the reference runs] mean; after recovery /
  that: [update after the reference runs]. Recovery measured against the no-loss three-drone
  baseline instead of each run's own P: [update after the reference runs].
- **Recovery** (coverage back within 10 % of its pre-loss value and staying there): 4 of 14 runs
  never fell below; 10 did and all came back, in 63.5 s [37.2, 76.4]. The dip is 83.7 % [66.9, 91.5]
  and comes late, 44.2 s [26.3, 115.9] after the loss, because coverage has a 60 s memory: the dead
  drone's last sightings fade out of the window while the survivors are already on their new loops.
- **Worst revisit gap.** Before the loss it is about the 2-lane loop's period, 67.7 s. Over the
  loss, cells on the dead drone's lane wait up to 107.8 s [72.8, 140.4]. Once the new split runs,
  the steady gap is back to 67.9 s.
- **Churn.** One re-plan per run, forced by the loss; nothing else changes ("changes beyond the one
  forced by the loss" 0 [0, 0]).
- **By victim.** Losing drone 3, which flew two lanes, hurts most: coverage from L + 60 s 90.9 %
  [89.0, 92.8] and recovery 67 to 76 s, against 94.8 % (drone 1) and 94.0 % (drone 2).
- **Safety and control.** Minimum 3D separation from t = 10 s: 74.9 m [60.4, 75.0]; time below 8 m:
  0 s in every run. Setpoints: 20.0 Hz for the slowest agent in every run.

**Per drone and fault time** (`results/gazebo_grid_A/`, final): each drone killed at 60, 70, 80 and
90 s, 12 runs. Before 97.5 % [97.2, 97.8], after recovery 94.5 % [91.6, 95.8], after / N-1 reference
0.99 [0.96, 1.00], recovery 63.8 s [51.0, 74.8] over the 9 runs that fell below, churn after 0.32
[0.25, 0.67]. `by_drone.png` shows it per drone and fault time.

Figures in each set: `coverage_vs_time.png`, `before_after.png`, `recovery_steps.png`,
`age_maps.png`, `assignments.png`, `separation.png`. The animation:
`results/replay_representative.mp4` (45 s, top-down, includes the dropout); `run_eval.sh` also
writes one into `results/gazebo_50m/` (`replay_seed_1.mp4`). The screen recording of the live demo:
[video: to be named].

## Mission B: priority zone

**Limits first.** The 60 m x 60 m zone is seen at least every 30 s while all three drones fly and
once the new split has settled. It is not held in these cases:

- during the change-over after a loss (about 40 s), the zone can go a few seconds over 30 s unseen:
  [update after the grid rerun];
- when the drone that flies the zone is the one lost, the zone waits for the 12 s lost timeout plus
  the survivors' travel: [update after the grid rerun] (58.7 s in the no-hold set);
- during a blackout of 10 s or more of the zone drone: it holds deaf and nothing re-plans;
- with one survivor only the zone strip is covered.

**What.** With three drones one owns a short out-and-back lane through the zone, its ends close
enough to the zone centre (126.5 m) that it is back within the 30 s; the other two share the rest.
With two survivors that leaves too little for the rest, so they fly a "butterfly": each a two-lane
loop, north on a centre lane 8 m beside the zone centre, then south on a wing lane one footprint
further out. Both centre lanes see the whole zone; the two drones are kept half a lap apart, so the
zone is seen twice a lap (`_butterfly_plans` in `swarm/allocator.py`). The half lap is chosen once
at the re-plan from a predicted transit, and in Gazebo it came out up to 8 s off. So the ground runs
a phase hold: the drone too close behind the other gets a speed limit of 0.8 x cruise (0.6 x when
more than 3 s off) until they are half a lap apart again (`butterfly_pace`).

**Results** (each drone killed at 60-90 s, 12 runs per set):

| Set | Zone longest unseen (s) | Zone oldest from L + 40 s (s) | Coverage from L + 60 s (%) |
|---|---|---|---|
| `gazebo_grid_B` (butterfly, phase hold) | [update after the grid rerun] | [update after the grid rerun] | [update after the grid rerun] |
| `gazebo_grid_B_nohold` (butterfly, no hold) | 30.6 [26.4, 58.7] | 26.0 [22.9, 31.9] | 79.1 [77.1, 81.9] |
| `gazebo_grid_B_old` (one zone owner) | 27.3 [26.5, 65.4] | 26.2 [26.2, 27.2] | 68.6 [64.5, 73.4] |

The old split held the zone but left the rest at 68.6 %, close to the (N-1)/N line. The butterfly
lifts that to about 79 % without the hold; whether the phase hold keeps the zone under 30 s once the
new split runs on the final code: [update after the grid rerun]. The 58.7 s maximum in the no-hold
set is the zone drone's own loss (victim 2). `compare.png` in `gazebo_grid_B/` puts the sets side by
side. `gazebo_grid_B` is being flown again on the final code [update after the grid rerun].

## Mission C: comms blackout

Every message to and from one drone dropped for 10 s. With the 12 s lost timeout the silent drone is
never declared lost: it goes deaf after 3 s, holds 7.2 s at its hold altitude, and is back on its
loop 1.7 s after the blackout; no re-plan, mission mean coverage 97.7 %
(`results/scenarios/C_s1.metrics.json`). With a 4 s timeout the same run declares it lost, re-plans,
then re-plans again when it rejoins; back covering 4.8 s after the blackout
(`results/lost4/C_s1.metrics.json`, an earlier commit with the same code). Telling silent from dead
over the radio alone needs a timeout longer than the longest silence; DECISIONS.md explains the
numbers and a camera-based check I tried and dropped.

## Live demo (not part of the evaluation)

`tools/show_gazebo.sh` runs one mission with the Gazebo window and a marker overlay. With
`SWARM_DEMO=1` the drones are coloured and the field drawn; `SWARM_DEMO_GROUND=field` adds farmland
and scenery. `SWARM_DEMO_CAMERA=1` puts the A8 mini's field of view on each drone as a Gazebo
camera, streamed through MediaMTX and shown in the web view (`python3 -m gui`) over WebRTC; one
frame a second is saved with its pose and laid on the ground as a photo mosaic. None of this runs in
the evaluation, and none of it feeds the swarm.

## Limitations and next steps

- **Lane tracking.** The coverage controller flies at slots; it does not hold the lane line. A
  controller that tracks the lane would make the footprint overlap more predictable.
- **The ground is a single point of failure for re-planning.** While it is down, a lost drone's
  lanes stay unflown (75.1 % in `groundloss`). Next: run the deterministic allocator on every drone
  from the shared states.
- **No EKF under Gazebo.** The JSON backend sets `AHRS_EKF_TYPE 10`: positions are the simulator's
  truth, so GPS faults and estimate errors are not exercised.
- **The dead drone is removed from Gazebo**, so a falling airframe is never in the air for the
  others.
- **N = 3 only.** More drones would degrade more gently (DECISIONS.md); the allocator handles any N
  but I flew only three.
- **Small samples.** 14 + 12 runs for Mission A, 12 per Mission B set, one run per scenario. Medians
  with min and max, no confidence intervals.
- **One-way radio failures** (a drone that hears but is not heard) are not emulated; the blackouts
  are two-way.

## Reuse and citations

- ArduPilot (ArduCopter SITL), GPLv3, pinned commit in `setup.sh`. Run as a binary, not modified.
- ardupilot_gazebo (plugin and the iris model), pinned commit in `setup.sh`. The live view's drones
  are copies of `iris_with_standoffs` with colours and an optional camera.
- Gazebo Harmonic.
- pymavlink and mavgen: `swarm/swarm_mav.py` is generated by mavgen from my `swarm/swarm.xml`.
- Gazebo Fuel models, live view only, fetched at run time: all by OpenRobotics and all CC0 1.0
  (Creative Commons Zero) on their Fuel pages: Cessna C-172, Base Station, Water tower, Pine Tree,
  Oak tree, SUV, Pickup, Hatchback and Hatchback blue. Licences read from the Fuel API
  (`https://fuel.gazebosim.org/1.0/OpenRobotics/models/<name>`, field `license_name`) on 24 Nov
  2026.
- MediaMTX (live video only), ffmpeg, numpy, matplotlib, Pillow (photo mosaic and camera bridge).

I did not copy a swarm framework. The allocator, coverage controller, safety filter, radio emulator,
ground station and metrics are written for this project.

## Where I used AI coding assistants

**TODO(owner): fill this in. Suggested skeleton:**

- Areas where assistants helped: [e.g. drafting code for X, tests for Y, plots, these documents]
- What I decided and wrote myself: [design, the safety filter rules, the evaluation definitions ...]
- What I checked by hand: [each number against the results files, flights watched, code read]
