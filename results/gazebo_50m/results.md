# gazebo_50m

12 runs, backend gazebo, N = 3. Cells: median [min, max] over runs; a reference of one run is a single value. Definitions: docs/metrics.md.

## How to read this

- Mission t = 0 is 60 s after every drone is on its coverage loop, so the 60 s coverage window is already full at t = 0 (no start-up ramp).
- The agent is killed at a seeded time L in [60, 90] s mission time; the mission ends at 180 s.
- coverage(t) = % of 2 m cells seen in (t - 60 s, t], every 1 s. Windows: before = [L - 60 s, L); dip = lowest coverage(t) on [L, end]; from L + 60 s = [L + 60 s, end] (the victim's last sightings have left the window); after recovery = [L + recovery, end] (a run that never fell below 0.9 P counts as recovered at L); end of run = the last 30 s; mission mean and the no-loss references = [0, end].
- Longest unseen: before = [0, L), from L on = [L, end] (a gap counts where it ends). Steady revisit gap: before = [0, L), from L on = [R, end], R = declared lost. Churn: [0, L) and [L, end].
- Recovery = first time from which coverage stays >= 0.9 x P to the end. 0.0 s means it never fell below 0.9 x P, not an instant recovery.
- The task's (N-1)/N x P check (about 65 %) is passed trivially at this altitude: the lowest dip of any run is 65.4 %. The stronger comparison is against the no-loss N-1 reference ("attainable with N-1").
- Reference runs used: baseline_n3, ref_n2.

| Metric | Before [L - 60 s, L) | From L on [L, end] | From L + 60 s | After recovery [L + rec, end] | End of run (last 30 s) | Attainable with N-1 (no-loss ref) | (N-1)/N x P | n |
|---|---|---|---|---|---|---|---|---|
| Coverage, mean (%) | 97.6 [97.2, 97.8] | - | 93.8 [88.1, 95.6] | 94.2 [92.7, 95.7] | 95.1 [93.8, 95.6] | 95.0 (1 run) | 65.1 [64.8, 65.2] | 12 |
| Coverage, min (%); from L on: the dip | 95.6 [95.3, 95.7] | 81.9 [65.4, 91.6] | 87.7 [70.7, 92.4] | 88.7 [87.7, 91.6] | 91.6 [87.5, 92.4] | - | - | 12 |
| Dip: when, after L (s) | - | 40.6 [24.3, 112.9] | - | - | - | - | - | 12 |
| Worst revisit gap (longest unseen) (s) | 66.8 [66.7, 67.0] | 106.4 [70.3, 140.9] | - | - | - | - | - | 12 |
| Steady revisit gap (s); from R | 65.9 [61.2, 66.0] | 66.6 [66.6, 70.2] | - | - | - | - | - | 12 |
| Assignment churn (/agent/min) | 0.00 [0.00, 0.00] | 0.30 [0.26, 0.59] | - | - | - | - | - | 12 |

- No-loss N baseline (B), mean coverage: 97.5 % (1 run).
- No-loss N-1 reference (attainable with N-1), mean coverage: 95.0 % (1 run).
- Recovery to 0.9 x P: 4 of 12 runs never fell below it (recovery 0.0 s); 8 fell below, 8 of them back within the run, over those 8: 65.5 [40.2, 77.3] s.
- Recovery to 0.9 x B: 4 of 12 runs never fell below it (recovery 0.0 s); 8 fell below, 8 of them back within the run, over those 8: 65.5 [40.2, 77.3] s.
- Recovery to 0.9 x (N-1)/N x P (weaker reading): 12 of 12 runs recovered; over the recovered runs: 0.0 [0.0, 0.0] s.

## By victim

Cells: mean [min, max] over that victim's runs. Lanes: the victim's loop lanes in the first plan.

| Victim | Lanes before the loss | n runs | Coverage from L + 60 s, mean (%) | Recovery to 0.9 x P | Dip (%) | Longest unseen, whole (s) |
|---|---|---|---|---|---|---|
| 1 | 1 (x = 37.5 m) | 5 | 94.9 [93.8, 95.6] | 3 never below; 2 below: 40.2, 62.2 s | 87.4 [78.1, 91.6] | 106.1 [104.8, 107.2] |
| 2 | 1 (x = 112.5 m) | 3 | 93.8 [92.3, 95.3] | 1 never below; 2 below: 49.0, 67.8 s | 85.0 [81.9, 91.2] | 85.6 [70.3, 112.0] |
| 3 | 2 (x = 187.5, 262.5 m) | 4 | 90.5 [88.1, 92.8] | 0 never below; 4 below: 63.7, 67.3, 77.3, 76.6 s | 70.0 [65.4, 75.9] | 113.1 [96.5, 140.9] |

## All metrics

| Metric | Unit | Median [min, max] | n |
|---|---|---|---|
| Worst revisit gap (longest unseen), whole mission [0, end] | s | 106.4 [70.3, 140.9] | 12 |
| Cells unseen > 60 s at some time, whole mission | % | 74.2 [62.5, 83.5] | 12 |
| Cells unseen > 90 s at some time, whole mission | % | 6.1 [0.0, 43.8] | 12 |
| Cells unseen > 60 s, gap ending from the loss on | % | 74.1 [62.4, 83.0] | 12 |
| Cells unseen > 90 s, gap ending from the loss on | % | 6.1 [0.0, 43.8] | 12 |
| Coverage over the mission, mean | % | 94.5 [88.7, 96.4] | 12 |
| After / ((N-1)/N x P) | ratio | 1.44 [1.35, 1.47] | 12 |
| After / attainable with N-1 | ratio | 0.99 [0.93, 1.01] | 12 |
| Coverage deficit after the loss, against P | %.s | 574 [241, 1613] | 12 |
| Coverage deficit after the loss, against the no-loss N baseline | %.s | 553 [230, 1584] | 12 |
| Loss -> declared lost | s | 12.1 [12.0, 12.2] | 12 |
| Loss -> new assignment held by every survivor | s | 12.8 [12.3, 14.3] | 12 |
| Loss -> first survivor on its new loop | s | 12.3 [12.3, 13.3] | 12 |
| Largest open gap at L | s | 64.7 [60.9, 65.0] | 12 |
| Largest open gap at the end | s | 64.7 [64.7, 65.6] | 12 |
| Cells with no completed gap, before | % | 0.0 [0.0, 0.0] | 12 |
| Cells with no completed gap, after | % | 0.0 [0.0, 0.0] | 12 |
| Changes beyond the one forced by the loss | count | 0 [0, 0] | 12 |
| Min 3D separation, whole flight | m | 10.0 [10.0, 10.0] | 12 |
| Min 3D separation, from t = 10 s | m | 74.9 [60.8, 75.0] | 12 |
| Time below 8 m | s | 0.0 [0.0, 0.0] | 12 |
| Time braking, all agents | s | 0.4 [0.0, 13.8] | 12 |
| Time deaf, all agents | s | 0.0 [0.0, 0.0] | 12 |
| Infeasible reports accepted | count | 0 [0, 0] | 12 |
| Infeasible reports ignored | count | 0 [0, 0] | 12 |
| Setpoint rate, slowest agent | Hz | 20.0 [20.0, 20.0] | 12 |
| Setpoint period p99, worst agent | s | 0.051 [0.050, 0.051] | 12 |
| Setpoint period max, worst agent | s | 0.054 [0.052, 0.114] | 12 |
| Max ground speed, logged velocity (vx, vy; autopilot EKF estimate) | m/s | 11.50 [11.49, 11.50] | 12 |
| Max ground speed, position differences over 1 s (EKF positions) | m/s | 11.96 [11.44, 11.99] | 12 |
| Radio: loss rate | % | 20.0 [19.5, 20.5] | 12 |
| Radio: mean delay | s | 0.200 [0.200, 0.200] | 12 |
| Radio: max delay | s | 0.203 [0.201, 0.247] | 12 |

Both speeds are the autopilots' EKF estimates (logged velocity and logged position), not simulator truth.

Re-plans by trigger, all runs: lost 12; in the warm-up: none.
Agents' epoch columns vs ground re-plans: no mismatch.

## Per run

| Run | Seed | Victim | Kill t (s) | Lost declared t (s) | Recovery (s) | Dip (%) | Dip at, after L (s) | From L + 60 s, mean (%) | After recovery, mean (%) | Longest unseen, whole (s) | Min separation from 10 s (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| seed_1 | 1 | 1 | 64.2 | 76.2 | 0.0 (never below) | 91.3 | 47.8 | 95.6 | 94.9 | 107.2 | 75.0 |
| seed_2 | 2 | 1 | 88.7 | 100.8 | 0.0 (never below) | 89.8 | 24.3 | 95.2 | 94.5 | 105.8 | 74.9 |
| seed_3 | 3 | 3 | 67.3 | 79.4 | 63.7 | 71.9 | 47.7 | 92.4 | 92.8 | 140.9 | 74.9 |
| seed_4 | 4 | 1 | 67.1 | 79.2 | 0.0 (never below) | 91.6 | 112.9 | 95.4 | 95.7 | 104.8 | 74.9 |
| seed_5 | 5 | 3 | 78.7 | 90.8 | 67.3 | 75.9 | 33.3 | 92.8 | 94.3 | 96.5 | 75.0 |
| seed_6 | 6 | 1 | 83.8 | 96.0 | 40.2 | 86.2 | 35.2 | 94.5 | 92.7 | 106.0 | 74.9 |
| seed_7 | 7 | 1 | 69.8 | 81.8 | 62.2 | 78.1 | 53.2 | 93.8 | 94.1 | 106.8 | 74.9 |
| seed_8 | 8 | 2 | 67.0 | 79.0 | 0.0 (never below) | 91.2 | 47.0 | 95.3 | 95.5 | 70.3 | 74.9 |
| seed_9 | 9 | 2 | 74.0 | 86.2 | 49.0 | 81.9 | 42.0 | 93.9 | 93.3 | 74.6 | 74.9 |
| seed_10 | 10 | 2 | 77.2 | 89.3 | 67.8 | 81.9 | 34.8 | 92.3 | 93.7 | 112.0 | 72.8 |
| seed_11 | 11 | 3 | 73.7 | 85.8 | 77.3 | 65.4 | 39.3 | 88.1 | 94.0 | 107.7 | 71.1 |
| seed_12 | 12 | 3 | 74.4 | 86.4 | 76.6 | 67.0 | 38.6 | 88.8 | 94.2 | 107.4 | 60.8 |
