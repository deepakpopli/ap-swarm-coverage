# gazebo_50m

14 runs, backend gazebo, N = 3. Cells: median [min, max] over runs; a reference of one run is a single value. Definitions: "How to read this" below and analysis/metrics.py.

Code: commit f459190fd7c1 (from each run's meta.json).

## How to read this

- Mission t = 0 is 60 s after every drone is on its coverage loop, so the 60 s coverage window is already full at t = 0 (no start-up ramp).
- The agent is killed at a seeded time L in [60, 90] s mission time; the mission ends at 180 s.
- coverage(t) = % of 2 m cells seen in (t - 60 s, t], every 1 s. Windows: before = [L - 60 s, L); dip = lowest coverage(t) on [L, end]; from L + 60 s = [L + 60 s, end] (the victim's last sightings have left the window); after recovery = [L + recovery, end] (a run that never fell below 0.9 P counts as recovered at L); end of run = the last 30 s; mission mean and the no-loss references = [0, end].
- Longest unseen: before = [0, L), from L on = [L, end] (a gap counts where it ends). Steady revisit gap: before = [0, L), from L on = [R, end], R = declared lost. Churn: [0, L) and [L, end].
- Recovery = first time from which coverage stays >= 0.9 x P to the end. 0.0 s means it never fell below 0.9 x P, not an instant recovery.
- The task's (N-1)/N x P check (about 65 %) is passed trivially at this altitude: the lowest dip of any run is 66.9 %. The stronger comparison is against the no-loss N-1 reference ("attainable with N-1").
- Reference runs used: baseline_n3, ref_n2.

| Metric | Before [L - 60 s, L) | From L on [L, end] | From L + 60 s | After recovery [L + rec, end] | End of run (last 30 s) | Attainable with N-1 (no-loss ref) | (N-1)/N x P | n |
|---|---|---|---|---|---|---|---|---|
| Coverage, mean (%) | 97.8 [97.4, 97.9] | - | 94.3 [89.2, 95.7] | 94.5 [92.9, 95.9] | 95.5 [94.4, 95.7] | 95.2 (1 run) | 65.2 [64.9, 65.3] | 14 |
| Coverage, min (%); from L on: the dip | 95.8 [95.7, 95.9] | 83.6 [66.9, 91.8] | 89.2 [72.4, 94.0] | 88.7 [88.1, 91.8] | 91.9 [88.8, 94.0] | - | - | 14 |
| Dip: when, after L (s) | - | 41.7 [25.1, 80.1] | - | - | - | - | - | 14 |
| Worst revisit gap (longest unseen) (s) | 67.5 [67.5, 67.7] | 106.6 [68.8, 112.0] | - | - | - | - | - | 14 |
| Steady revisit gap (s); from R | 67.2 [58.1, 67.4] | 67.6 [67.2, 69.0] | - | - | - | - | - | 14 |
| Assignment churn (/agent/min) | 0.00 [0.00, 0.00] | 0.29 [0.26, 0.59] | - | - | - | - | - | 14 |

- No-loss N baseline (B), mean coverage: 97.7 % (1 run).
- No-loss N-1 reference (attainable with N-1), mean coverage: 95.2 % (1 run).
- Recovery to 0.9 x P: 3 of 14 runs never fell below it (recovery 0.0 s); 11 fell below, 11 of them back within the run, over those 11: 62.9 [37.2, 77.8] s.
- Recovery to 0.9 x B: 3 of 14 runs never fell below it (recovery 0.0 s); 11 fell below, 11 of them back within the run, over those 11: 62.9 [37.2, 77.8] s.
- Recovery to 0.9 x (N-1)/N x P (weaker reading): 14 of 14 runs recovered; over the recovered runs: 0.0 [0.0, 0.0] s.

## By victim

Cells: mean [min, max] over that victim's runs. Lanes: the victim's loop lanes in the first plan.

| Victim | Lanes before the loss | n runs | Coverage from L + 60 s, mean (%) | Recovery to 0.9 x P | Dip (%) | Longest unseen, whole (s) |
|---|---|---|---|---|---|---|
| 1 | 1 (x = 37.5 m) | 5 | 95.0 [94.4, 95.7] | 2 never below; 3 below: 62.9, 37.2, 59.2 s | 86.2 [80.8, 91.6] | 107.8 [106.0, 109.5] |
| 2 | 1 (x = 112.5 m) | 5 | 94.5 [93.3, 95.5] | 1 never below; 4 below: 48.0, 64.7, 49.0, 43.3 s | 86.4 [82.9, 91.8] | 80.9 [68.8, 112.0] |
| 3 | 2 (x = 187.5, 262.5 m) | 4 | 90.6 [89.2, 92.0] | 0 never below; 4 below: 77.8, 69.2, 76.3, 75.6 s | 72.5 [66.9, 79.4] | 101.8 [91.6, 107.8] |

## All metrics

| Metric | Unit | Median [min, max] | n |
|---|---|---|---|
| Worst revisit gap (longest unseen), whole mission [0, end] | s | 106.6 [68.8, 112.0] | 14 |
| Cells unseen > 60 s at some time, whole mission | % | 75.0 [61.5, 82.5] | 14 |
| Cells unseen > 90 s at some time, whole mission | % | 4.4 [0.0, 42.5] | 14 |
| Cells unseen > 60 s, gap ending from the loss on | % | 74.9 [61.5, 82.0] | 14 |
| Cells unseen > 90 s, gap ending from the loss on | % | 4.4 [0.0, 42.5] | 14 |
| Coverage over the mission, mean | % | 95.1 [89.2, 96.6] | 14 |
| After / ((N-1)/N x P) | ratio | 1.45 [1.37, 1.47] | 14 |
| After / attainable with N-1 | ratio | 0.99 [0.94, 1.01] | 14 |
| Coverage deficit after the loss, against P | %.s | 476 [230, 1551] | 14 |
| Coverage deficit after the loss, against the no-loss N baseline | %.s | 473 [234, 1531] | 14 |
| Loss -> declared lost | s | 12.1 [11.8, 12.2] | 14 |
| Loss -> new assignment held by every survivor | s | 12.4 [12.2, 14.3] | 14 |
| Loss -> first survivor on its new loop | s | 12.4 [12.0, 13.3] | 14 |
| Largest open gap at L | s | 64.7 [60.7, 66.4] | 14 |
| Largest open gap at the end | s | 65.1 [64.6, 66.3] | 14 |
| Cells with no completed gap, before | % | 0.0 [0.0, 0.0] | 14 |
| Cells with no completed gap, after | % | 0.0 [0.0, 0.0] | 14 |
| Changes beyond the one forced by the loss | count | 0 [0, 0] | 14 |
| Min 3D separation, whole flight | m | 10.0 [10.0, 10.0] | 14 |
| Min 3D separation, from t = 10 s | m | 74.9 [56.8, 75.0] | 14 |
| Time below 8 m | s | 0.0 [0.0, 0.0] | 14 |
| Time braking, all agents | s | 1.0 [0.0, 12.8] | 14 |
| Time deaf, all agents | s | 0.0 [0.0, 0.0] | 14 |
| Infeasible reports accepted | count | 0 [0, 0] | 14 |
| Infeasible reports ignored | count | 0 [0, 0] | 14 |
| Setpoint rate, slowest agent | Hz | 20.0 [20.0, 20.0] | 14 |
| Setpoint period p99, worst agent | s | 0.051 [0.050, 0.051] | 14 |
| Setpoint period max, worst agent | s | 0.051 [0.050, 0.053] | 14 |
| Max ground speed, logged velocity (vx, vy; autopilot EKF estimate) | m/s | 11.49 [11.49, 11.50] | 14 |
| Max ground speed, position differences over 1 s (EKF positions) | m/s | 11.98 [11.45, 12.01] | 14 |
| Radio: loss rate | % | 20.0 [19.5, 20.5] | 14 |
| Radio: mean delay | s | 0.200 [0.200, 0.200] | 14 |
| Radio: max delay | s | 0.201 [0.201, 0.207] | 14 |

Both speeds are the autopilots' EKF estimates (logged velocity and logged position), not simulator truth.

Re-plans by trigger, all runs: lost 14; in the warm-up: none.
Agents' epoch columns vs ground re-plans: no mismatch.

## Per run

| Run | Seed | Victim | Kill t (s) | Lost declared t (s) | Recovery (s) | Dip (%) | Dip at, after L (s) | From L + 60 s, mean (%) | After recovery, mean (%) | Longest unseen, whole (s) | Min separation from 10 s (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| seed_1 | 1 | 1 | 64.2 | 76.2 | 0.0 (never below) | 91.6 | 49.8 | 95.7 | 95.1 | 108.3 | 75.0 |
| seed_2 | 2 | 1 | 88.9 | 101.0 | 0.0 (never below) | 90.7 | 25.1 | 95.5 | 94.9 | 106.0 | 74.9 |
| seed_3 | 3 | 3 | 67.2 | 79.4 | 77.8 | 79.4 | 60.8 | 91.2 | 94.9 | 91.6 | 74.9 |
| seed_4 | 4 | 1 | 67.1 | 79.2 | 62.9 | 80.8 | 54.9 | 94.4 | 94.6 | 109.5 | 74.9 |
| seed_5 | 5 | 3 | 78.8 | 90.9 | 69.2 | 73.6 | 34.2 | 92.0 | 94.2 | 100.0 | 75.0 |
| seed_6 | 6 | 1 | 83.8 | 95.6 | 37.2 | 87.0 | 34.2 | 94.7 | 92.9 | 108.0 | 74.9 |
| seed_7 | 7 | 1 | 69.8 | 82.0 | 59.2 | 80.9 | 52.2 | 94.8 | 94.7 | 107.2 | 74.9 |
| seed_8 | 8 | 2 | 66.9 | 79.0 | 0.0 (never below) | 91.8 | 80.1 | 95.5 | 95.9 | 68.8 | 74.9 |
| seed_9 | 9 | 2 | 74.0 | 86.1 | 48.0 | 82.9 | 43.0 | 94.3 | 93.7 | 74.9 | 74.9 |
| seed_10 | 10 | 2 | 77.3 | 89.4 | 64.7 | 84.3 | 35.7 | 93.3 | 93.9 | 112.0 | 72.6 |
| seed_11 | 11 | 3 | 73.7 | 85.8 | 76.3 | 66.9 | 38.3 | 89.2 | 94.4 | 107.8 | 67.6 |
| seed_12 | 12 | 3 | 74.4 | 86.6 | 75.6 | 70.1 | 39.6 | 89.9 | 94.4 | 107.8 | 56.8 |
| seed_16 | 16 | 2 | 71.0 | 83.1 | 49.0 | 86.4 | 45.0 | 95.1 | 94.7 | 74.3 | 74.9 |
| seed_17 | 17 | 2 | 75.7 | 87.6 | 43.3 | 86.7 | 40.3 | 94.2 | 94.2 | 74.6 | 74.9 |
