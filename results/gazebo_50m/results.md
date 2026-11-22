# gazebo_50m

12 runs, backend gazebo, N = 3. Cells: median [min, max] over runs; a reference of one run is a single value. Definitions: REPORT.md section 11.

## How to read this

- Mission t = 0 is 60 s after every drone is on its coverage loop, so the 60 s coverage window is already full at t = 0 (no start-up ramp).
- The agent is killed at a seeded time L in [60, 90] s mission time; the mission ends at 180 s.
- coverage(t) = % of 2 m cells seen in (t - 60 s, t], every 1 s. Windows: before = [L - 60 s, L); dip = lowest coverage(t) on [L, end]; from L + 60 s = [L + 60 s, end] (the victim's last sightings have left the window); after recovery = [L + recovery, end] (a run that never fell below 0.9 P counts as recovered at L); end of run = the last 30 s; mission mean and the no-loss references = [0, end].
- Longest unseen: before = [0, L), from L on = [L, end] (a gap counts where it ends). Steady revisit gap: before = [0, L), from L on = [R, end], R = declared lost. Churn: [0, L) and [L, end].
- Recovery = first time from which coverage stays >= 0.9 x P to the end. 0.0 s means it never fell below 0.9 x P, not an instant recovery.
- The task's (N-1)/N x P check (about 65 %) is passed trivially at this altitude: the lowest dip of any run is 65.5 %. The stronger comparison is against the no-loss N-1 reference ("attainable with N-1").
- Reference runs used: baseline_n3, ref_n2.

| Metric | Before [L - 60 s, L) | From L on [L, end] | From L + 60 s | After recovery [L + rec, end] | End of run (last 30 s) | Attainable with N-1 (no-loss ref) | (N-1)/N x P | n |
|---|---|---|---|---|---|---|---|---|
| Coverage, mean (%) | 97.7 [97.3, 97.9] | - | 94.0 [88.5, 95.7] | 94.4 [92.8, 96.1] | 95.4 [94.2, 95.7] | 95.2 (1 run) | 65.1 [64.9, 65.2] | 12 |
| Coverage, min (%); from L on: the dip | 95.8 [95.6, 95.9] | 80.3 [65.5, 92.0] | 86.9 [70.9, 92.9] | 88.7 [88.0, 92.0] | 92.0 [88.1, 92.9] | - | - | 12 |
| Dip: when, after L (s) | - | 40.9 [24.3, 79.1] | - | - | - | - | - | 12 |
| Worst revisit gap (longest unseen) (s) | 66.6 [66.5, 66.6] | 106.4 [67.8, 141.3] | - | - | - | - | - | 12 |
| Steady revisit gap (s); from R | 65.7 [60.8, 65.8] | 66.5 [66.4, 70.5] | - | - | - | - | - | 12 |
| Assignment churn (/agent/min) | 0.00 [0.00, 0.00] | 0.30 [0.26, 0.59] | - | - | - | - | - | 12 |

- No-loss N baseline (B), mean coverage: 97.6 % (1 run).
- No-loss N-1 reference (attainable with N-1), mean coverage: 95.2 % (1 run).
- Recovery to 0.9 x P: 3 of 12 runs never fell below it (recovery 0.0 s); 9 fell below, 9 of them back within the run, over those 9: 66.7 [38.1, 76.8] s.
- Recovery to 0.9 x B: 3 of 12 runs never fell below it (recovery 0.0 s); 9 fell below, 9 of them back within the run, over those 9: 66.2 [38.1, 76.8] s.
- Recovery to 0.9 x (N-1)/N x P (weaker reading): 12 of 12 runs recovered; over the recovered runs: 0.0 [0.0, 0.0] s.

## By victim

Cells: mean [min, max] over that victim's runs. Lanes: the victim's loop lanes in the first plan.

| Victim | Lanes before the loss | n runs | Coverage from L + 60 s, mean (%) | Recovery to 0.9 x P | Dip (%) | Longest unseen, whole (s) |
|---|---|---|---|---|---|---|
| 1 | 1 (x = 37.5 m) | 5 | 94.8 [93.9, 95.7] | 2 never below; 3 below: 63.8, 38.1, 60.2 s | 85.1 [78.2, 91.6] | 106.0 [104.5, 106.8] |
| 2 | 1 (x = 112.5 m) | 3 | 94.2 [92.7, 95.7] | 1 never below; 2 below: 49.0, 66.7 s | 85.6 [82.4, 92.0] | 84.4 [67.8, 111.2] |
| 3 | 2 (x = 187.5, 262.5 m) | 4 | 90.8 [88.5, 92.9] | 0 never below; 4 below: 76.8, 67.2, 76.4, 75.8 s | 70.4 [65.5, 75.9] | 113.1 [96.4, 141.3] |

## All metrics

| Metric | Unit | Median [min, max] | n |
|---|---|---|---|
| Worst revisit gap (longest unseen), whole mission [0, end] | s | 106.4 [67.8, 141.3] | 12 |
| Cells unseen > 60 s at some time, whole mission | % | 74.0 [62.3, 83.0] | 12 |
| Cells unseen > 90 s at some time, whole mission | % | 6.2 [0.0, 43.5] | 12 |
| Cells unseen > 60 s, gap ending from the loss on | % | 74.0 [62.3, 82.5] | 12 |
| Cells unseen > 90 s, gap ending from the loss on | % | 6.2 [0.0, 43.5] | 12 |
| Coverage over the mission, mean | % | 94.7 [88.9, 96.6] | 12 |
| After / ((N-1)/N x P) | ratio | 1.44 [1.36, 1.47] | 12 |
| After / attainable with N-1 | ratio | 0.99 [0.93, 1.01] | 12 |
| Coverage deficit after the loss, against P | %.s | 557 [226, 1607] | 12 |
| Coverage deficit after the loss, against the no-loss N baseline | %.s | 542 [222, 1579] | 12 |
| Loss -> declared lost | s | 12.1 [11.8, 12.2] | 12 |
| Loss -> new assignment held by every survivor | s | 12.4 [12.1, 14.3] | 12 |
| Loss -> first survivor on its new loop | s | 12.3 [12.1, 13.4] | 12 |
| Largest open gap at L | s | 64.4 [60.8, 64.8] | 12 |
| Largest open gap at the end | s | 64.6 [64.5, 66.2] | 12 |
| Cells with no completed gap, before | % | 0.0 [0.0, 0.0] | 12 |
| Cells with no completed gap, after | % | 0.0 [0.0, 0.0] | 12 |
| Changes beyond the one forced by the loss | count | 0 [0, 0] | 12 |
| Min 3D separation, whole flight | m | 10.0 [10.0, 10.0] | 12 |
| Min 3D separation, from t = 10 s | m | 74.9 [57.5, 75.0] | 12 |
| Time below 8 m | s | 0.0 [0.0, 0.0] | 12 |
| Time braking, all agents | s | 0.3 [0.0, 13.1] | 12 |
| Time deaf, all agents | s | 0.0 [0.0, 0.0] | 12 |
| Infeasible reports accepted | count | 0 [0, 0] | 12 |
| Infeasible reports ignored | count | 0 [0, 0] | 12 |
| Setpoint rate, slowest agent | Hz | 20.0 [20.0, 20.0] | 12 |
| Setpoint period p99, worst agent | s | 0.050 [0.050, 0.051] | 12 |
| Setpoint period max, worst agent | s | 0.051 [0.050, 0.052] | 12 |
| Max ground speed, logged velocity (vx, vy; autopilot EKF estimate) | m/s | 11.50 [11.49, 11.50] | 12 |
| Max ground speed, position differences over 1 s (EKF positions) | m/s | 11.97 [11.44, 12.01] | 12 |
| Radio: loss rate | % | 20.0 [19.5, 20.5] | 12 |
| Radio: mean delay | s | 0.200 [0.200, 0.200] | 12 |
| Radio: max delay | s | 0.201 [0.201, 0.201] | 12 |

Both speeds are the autopilots' EKF estimates (logged velocity and logged position), not simulator truth.

Re-plans by trigger, all runs: lost 12; in the warm-up: none.
Agents' epoch columns vs ground re-plans: no mismatch.

## Per run

| Run | Seed | Victim | Kill t (s) | Lost declared t (s) | Recovery (s) | Dip (%) | Dip at, after L (s) | From L + 60 s, mean (%) | After recovery, mean (%) | Longest unseen, whole (s) | Min separation from 10 s (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| seed_1 | 1 | 1 | 64.2 | 76.2 | 0.0 (never below) | 91.6 | 46.8 | 95.7 | 95.1 | 106.8 | 75.0 |
| seed_2 | 2 | 1 | 88.7 | 100.8 | 0.0 (never below) | 90.4 | 24.3 | 95.5 | 94.8 | 104.5 | 74.9 |
| seed_3 | 3 | 3 | 67.2 | 79.0 | 76.8 | 72.0 | 46.8 | 92.4 | 94.5 | 141.3 | 74.9 |
| seed_4 | 4 | 1 | 67.2 | 79.2 | 63.8 | 78.2 | 54.8 | 93.9 | 94.4 | 106.2 | 74.9 |
| seed_5 | 5 | 3 | 78.8 | 90.6 | 67.2 | 75.9 | 33.2 | 92.9 | 94.4 | 96.4 | 75.0 |
| seed_6 | 6 | 1 | 83.9 | 96.0 | 38.1 | 86.9 | 35.1 | 94.7 | 92.8 | 106.5 | 74.9 |
| seed_7 | 7 | 1 | 69.8 | 81.8 | 60.2 | 78.2 | 52.2 | 94.4 | 94.4 | 105.9 | 74.8 |
| seed_8 | 8 | 2 | 66.9 | 79.0 | 0.0 (never below) | 92.0 | 79.1 | 95.7 | 96.1 | 67.8 | 74.9 |
| seed_9 | 9 | 2 | 74.0 | 86.2 | 49.0 | 82.5 | 42.0 | 94.1 | 93.7 | 74.2 | 74.9 |
| seed_10 | 10 | 2 | 77.3 | 89.4 | 66.7 | 82.4 | 35.7 | 92.7 | 93.8 | 111.2 | 72.4 |
| seed_11 | 11 | 3 | 73.6 | 85.6 | 76.4 | 65.5 | 38.4 | 88.5 | 94.2 | 107.4 | 71.0 |
| seed_12 | 12 | 3 | 74.2 | 86.2 | 75.8 | 68.3 | 39.8 | 89.2 | 94.2 | 107.3 | 57.5 |
