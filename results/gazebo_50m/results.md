# gazebo_50m

17 runs, backend gazebo, N = 3. Cells: median [min, max] over runs; a reference of one run is a single value. Definitions: "How to read this" below and analysis/metrics.py.

Code: commit 1dbc16b35052, 1dbc16b35052-dirty (from each run's meta.json).

## How to read this

- Mission t = 0 is 60 s after every drone is on its coverage loop, so the 60 s coverage window is already full at t = 0 (no start-up ramp).
- The agent is killed at time L in [60, 90] s mission time (seeded, or chosen with --kill-at); the mission ends at 180 s.
- coverage(t) = % of 2 m cells seen in (t - 60 s, t], every 1 s. Windows: before = [L - 60 s, L); dip = lowest coverage(t) on [L, end]; from L + 60 s = [L + 60 s, end] (the victim's last sightings have left the window); after recovery = [L + recovery, end] (a run that never fell below 0.9 P counts as recovered at L); end of run = the last 30 s; mission mean and the no-loss references = [0, end].
- Longest unseen: before = [0, L), from L on = [L, end] (a gap counts where it ends). Steady revisit gap: before = [0, L), from L on = [R, end], R = declared lost. Churn: [0, L) and [L, end].
- Recovery = first time from which coverage stays >= 0.9 x P to the end. 0.0 s means it never fell below 0.9 x P, not an instant recovery.
- The task's (N-1)/N x P check (at most 65.2 %) is passed trivially at this altitude: the lowest dip of any run is 66.9 %. The stronger comparison is against the no-loss N-1 reference ("attainable with N-1").
- Reference runs used: baseline_n3, ref_n2.

| Metric | Before [L - 60 s, L) | From L on [L, end] | From L + 60 s | After recovery [L + rec, end] | End of run (last 30 s) | Attainable with N-1 (no-loss ref) | (N-1)/N x P | n |
|---|---|---|---|---|---|---|---|---|
| Coverage, mean (%) | 97.6 [97.0, 97.8] | - | 93.9 [89.0, 95.7] | 94.3 [92.8, 95.7] | 95.3 [93.6, 95.7] | 95.1 (1 run) | 65.1 [64.7, 65.2] | 17 |
| Coverage, min (%); from L on: the dip | 95.7 [94.3, 95.8] | 83.3 [66.9, 91.5] | 88.9 [72.1, 94.0] | 88.4 [87.6, 91.5] | 91.6 [88.3, 94.0] | - | - | 17 |
| Dip: when, after L (s) | - | 46.1 [26.3, 115.9] | - | - | - | - | - | 17 |
| Worst revisit gap (longest unseen) (s) | 67.7 [67.5, 69.0] | 107.8 [72.8, 142.5] | - | - | - | - | - | 17 |
| Steady revisit gap (s); from R | 67.4 [58.2, 69.0] | 68.0 [67.1, 71.0] | - | - | - | - | - | 17 |
| Assignment churn (/agent/min) | 0.00 [0.00, 0.00] | 0.31 [0.26, 0.59] | - | - | - | - | - | 17 |

- No-loss N baseline (B), mean coverage: 97.0 % (1 run).
- No-loss N-1 reference (attainable with N-1), mean coverage: 95.1 % (1 run).
- Recovery to 0.9 x P: 5 of 17 runs never fell below it (recovery 0.0 s); 12 fell below, 12 of them back within the run, over those 12: 65.8 [37.2, 77.1] s.
- Recovery to 0.9 x B: 5 of 17 runs never fell below it (recovery 0.0 s); 12 fell below, 12 of them back within the run, over those 12: 65.3 [36.2, 76.4] s.
- Recovery to 0.9 x (N-1)/N x P (weaker reading): 17 of 17 runs recovered; over the recovered runs: 0.0 [0.0, 0.0] s.

## By victim

Cells: mean [min, max] over that victim's runs. Lanes: the victim's loop lanes in the first plan.

| Victim | Lanes before the loss | n runs | Coverage from L + 60 s, mean (%) | Recovery to 0.9 x P | Dip (%) | Longest unseen, whole (s) |
|---|---|---|---|---|---|---|
| 1 | 1 (x = 37.5 m) | 6 | 94.9 [94.1, 95.7] | 4 never below; 2 below, 37.2 s, 60.2 s | 88.2 [80.5, 91.5] | 107.0 [103.6, 108.5] |
| 2 | 1 (x = 112.5 m) | 5 | 94.0 [92.7, 95.0] | 1 never below; 4 below, 50.1 s, 66.8 s, 51.0 s, 48.2 s | 84.7 [81.8, 88.4] | 83.0 [72.8, 113.2] |
| 3 | 2 (x = 187.5, 262.5 m) | 6 | 91.5 [89.0, 93.6] | 0 never below; 6 below, 70.8 s, 67.3 s, 76.4 s, 75.6 s, 77.1 s, 64.8 s | 72.5 [66.9, 80.5] | 119.6 [97.9, 142.5] |

## All metrics

| Metric | Unit | Median [min, max] | n |
|---|---|---|---|
| Worst revisit gap (longest unseen), whole mission [0, end] | s | 107.8 [72.8, 142.5] | 17 |
| Cells unseen > 60 s at some time, whole mission | % | 75.0 [62.5, 85.1] | 17 |
| Cells unseen > 90 s at some time, whole mission | % | 5.2 [0.0, 42.4] | 17 |
| Cells unseen > 60 s, gap ending from the loss on | % | 74.9 [62.3, 84.2] | 17 |
| Cells unseen > 90 s, gap ending from the loss on | % | 5.2 [0.0, 42.4] | 17 |
| Coverage over the mission, mean | % | 94.6 [89.2, 96.4] | 17 |
| After / ((N-1)/N x P) | ratio | 1.45 [1.36, 1.47] | 17 |
| After / attainable with N-1 | ratio | 0.99 [0.94, 1.01] | 17 |
| Coverage deficit after the loss, against P | %.s | 478 [238, 1550] | 17 |
| Coverage deficit after the loss, against the no-loss N baseline | %.s | 454 [189, 1468] | 17 |
| Loss -> declared lost | s | 12.1 [11.8, 12.2] | 17 |
| Loss -> new assignment held by every survivor | s | 12.4 [12.2, 15.3] | 17 |
| Loss -> first survivor on its new loop | s | 12.3 [12.1, 12.4] | 17 |
| Largest open gap at L | s | 64.8 [61.0, 67.1] | 17 |
| Largest open gap at the end | s | 65.2 [64.2, 66.7] | 17 |
| Cells with no completed gap, before | % | 0.0 [0.0, 0.2] | 17 |
| Cells with no completed gap, after | % | 0.0 [0.0, 0.0] | 17 |
| Changes beyond the one forced by the loss | count | 0 [0, 0] | 17 |
| Min 3D separation, whole flight | m | 10.0 [10.0, 10.0] | 17 |
| Min 3D separation, from t = 10 s | m | 74.9 [60.4, 75.0] | 17 |
| Time below 8 m | s | 0.0 [0.0, 0.0] | 17 |
| Time braking, all agents | s | 0.4 [0.0, 13.9] | 17 |
| Time deaf, all agents | s | 0.0 [0.0, 0.0] | 17 |
| Infeasible reports accepted | count | 0 [0, 0] | 17 |
| Infeasible reports ignored | count | 0 [0, 0] | 17 |
| Setpoint rate, slowest agent | Hz | 20.0 [20.0, 20.0] | 17 |
| Setpoint period p99, worst agent | s | 0.051 [0.050, 0.051] | 17 |
| Setpoint period max, worst agent | s | 0.053 [0.051, 0.089] | 17 |
| Max ground speed, logged velocity (vx, vy; as the autopilot reports it) | m/s | 11.50 [11.49, 11.50] | 17 |
| Max ground speed, position differences over 1 s (reported positions) | m/s | 11.53 [11.44, 11.99] | 17 |
| Radio: loss rate | % | 20.0 [19.4, 20.5] | 17 |
| Radio: mean delay | s | 0.200 [0.200, 0.200] | 17 |
| Radio: max delay | s | 0.203 [0.201, 0.224] | 17 |

Both speeds are what the autopilots report (logged velocity and position). Under Gazebo that is the simulator's own state: its JSON backend sets AHRS_EKF_TYPE 10 (no EKF), so a GPS fault there changes nothing.

Re-plans by trigger, all runs: lost 17; in the warm-up: none.
Agents' epoch columns vs ground re-plans: no mismatch.

## Per run

| Run | Seed | Victim | Kill t (s) | Lost declared t (s) | Recovery (s) | Dip (%) | Dip at, after L (s) | From L + 60 s, mean (%) | After recovery, mean (%) | Longest unseen, whole (s) | Min separation from 10 s (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| seed_1 | 1 | 1 | 64.1 | 76.2 | 0.0 (never below) | 91.5 | 115.9 | 95.7 | 95.1 | 108.5 | 75.0 |
| seed_2 | 2 | 1 | 88.7 | 100.8 | 0.0 (never below) | 90.0 | 26.3 | 94.1 | 94.1 | 107.7 | 74.9 |
| seed_3 | 3 | 3 | 67.2 | 79.4 | 70.8 | 72.2 | 47.8 | 92.6 | 93.8 | 140.4 | 74.9 |
| seed_4 | 4 | 1 | 67.2 | 79.2 | 0.0 (never below) | 90.8 | 112.8 | 95.1 | 95.7 | 106.4 | 74.9 |
| seed_5 | 5 | 3 | 78.7 | 90.8 | 67.3 | 75.6 | 33.3 | 92.8 | 94.4 | 97.9 | 75.0 |
| seed_6 | 6 | 1 | 83.8 | 96.0 | 37.2 | 86.8 | 34.2 | 94.7 | 92.8 | 108.1 | 74.9 |
| seed_7 | 7 | 1 | 69.8 | 81.8 | 60.2 | 80.5 | 53.2 | 94.6 | 94.6 | 107.8 | 74.9 |
| seed_8 | 8 | 2 | 66.9 | 79.0 | 0.0 (never below) | 88.4 | 49.1 | 95.0 | 95.1 | 72.8 | 74.9 |
| seed_9 | 9 | 2 | 73.9 | 86.0 | 50.1 | 84.0 | 46.1 | 93.4 | 93.1 | 76.0 | 74.9 |
| seed_10 | 10 | 2 | 77.2 | 89.0 | 66.8 | 83.3 | 35.8 | 92.7 | 93.8 | 113.2 | 73.1 |
| seed_11 | 11 | 3 | 73.6 | 85.8 | 76.4 | 66.9 | 38.4 | 89.0 | 94.2 | 108.1 | 70.4 |
| seed_12 | 12 | 3 | 74.4 | 86.6 | 75.6 | 68.8 | 38.6 | 89.3 | 94.0 | 108.3 | 60.4 |
| seed_13 | 13 | 3 | 67.9 | 80.0 | 77.1 | 70.8 | 48.1 | 91.8 | 94.4 | 142.5 | 74.9 |
| seed_14 | 14 | 3 | 63.2 | 75.2 | 64.8 | 80.5 | 56.8 | 93.6 | 94.3 | 120.5 | 75.0 |
| seed_15 | 15 | 1 | 89.1 | 101.2 | 0.0 (never below) | 89.3 | 27.9 | 95.4 | 94.5 | 103.6 | 74.9 |
| seed_16 | 16 | 2 | 71.0 | 83.2 | 51.0 | 85.9 | 48.0 | 94.9 | 94.5 | 77.3 | 74.9 |
| seed_17 | 17 | 2 | 75.8 | 87.6 | 48.2 | 81.8 | 42.2 | 93.9 | 93.3 | 75.5 | 74.9 |
