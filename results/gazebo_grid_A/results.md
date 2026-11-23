# gazebo_grid_A

12 runs, backend gazebo, N = 3. Cells: median [min, max] over runs; a reference of one run is a single value. Definitions: "How to read this" below and analysis/metrics.py.

Code: commit c68ee2279695, f459190fd7c1 (from each run's meta.json).

## How to read this

- Mission t = 0 is 60 s after every drone is on its coverage loop, so the 60 s coverage window is already full at t = 0 (no start-up ramp).
- The agent is killed at a seeded time L in [60, 90] s mission time; the mission ends at 180 s.
- coverage(t) = % of 2 m cells seen in (t - 60 s, t], every 1 s. Windows: before = [L - 60 s, L); dip = lowest coverage(t) on [L, end]; from L + 60 s = [L + 60 s, end] (the victim's last sightings have left the window); after recovery = [L + recovery, end] (a run that never fell below 0.9 P counts as recovered at L); end of run = the last 30 s; mission mean and the no-loss references = [0, end].
- Longest unseen: before = [0, L), from L on = [L, end] (a gap counts where it ends). Steady revisit gap: before = [0, L), from L on = [R, end], R = declared lost. Churn: [0, L) and [L, end].
- Recovery = first time from which coverage stays >= 0.9 x P to the end. 0.0 s means it never fell below 0.9 x P, not an instant recovery.
- The task's (N-1)/N x P check (about 65 %) is passed trivially at this altitude: the lowest dip of any run is 70.8 %. The stronger comparison is against the no-loss N-1 reference ("attainable with N-1").
- Reference runs used: baseline_n3, ref_n2.

| Metric | Before [L - 60 s, L) | From L on [L, end] | From L + 60 s | After recovery [L + rec, end] | End of run (last 30 s) | Attainable with N-1 (no-loss ref) | (N-1)/N x P | n |
|---|---|---|---|---|---|---|---|---|
| Coverage, mean (%) | 97.5 [97.2, 97.8] | - | 94.6 [91.0, 95.6] | 94.5 [91.6, 95.8] | 95.4 [90.8, 95.6] | 95.2 (1 run) | 65.0 [64.8, 65.2] | 12 |
| Coverage, min (%); from L on: the dip | 95.7 [95.4, 95.8] | 84.0 [70.8, 91.2] | 88.5 [81.5, 94.0] | 88.1 [87.7, 91.2] | 91.8 [83.6, 94.2] | - | - | 12 |
| Dip: when, after L (s) | - | 51.9 [24.9, 60.9] | - | - | - | - | - | 12 |
| Worst revisit gap (longest unseen) (s) | 67.6 [67.5, 68.1] | 109.7 [74.6, 142.8] | - | - | - | - | - | 12 |
| Steady revisit gap (s); from R | 67.0 [57.8, 67.5] | 67.6 [67.6, 71.3] | - | - | - | - | - | 12 |
| Assignment churn (/agent/min) | 0.00 [0.00, 0.00] | 0.32 [0.25, 0.67] | - | - | - | - | - | 12 |

- No-loss N baseline (B), mean coverage: 97.7 % (1 run).
- No-loss N-1 reference (attainable with N-1), mean coverage: 95.2 % (1 run).
- Recovery to 0.9 x P: 3 of 12 runs never fell below it (recovery 0.0 s); 9 fell below, 9 of them back within the run, over those 9: 63.8 [51.0, 74.8] s.
- Recovery to 0.9 x B: 2 of 12 runs never fell below it (recovery 0.0 s); 10 fell below, 10 of them back within the run, over those 10: 63.4 [51.0, 74.8] s.
- Recovery to 0.9 x (N-1)/N x P (weaker reading): 12 of 12 runs recovered; over the recovered runs: 0.0 [0.0, 0.0] s.

## By victim

Cells: mean [min, max] over that victim's runs. Lanes: the victim's loop lanes in the first plan.

| Victim | Lanes before the loss | n runs | Coverage from L + 60 s, mean (%) | Recovery to 0.9 x P | Dip (%) | Longest unseen, whole (s) |
|---|---|---|---|---|---|---|
| 1 | 1 (x = 37.5 m) | 4 | 95.1 [94.6, 95.6] | 2 never below; 2 below: 59.9, 58.8 s | 85.8 [80.4, 91.2] | 107.9 [106.1, 109.8] |
| 2 | 1 (x = 112.5 m) | 4 | 94.7 [93.7, 95.2] | 1 never below; 3 below: 63.9, 51.0, 62.9 s | 87.0 [84.3, 89.9] | 107.8 [74.6, 123.9] |
| 3 | 2 (x = 187.5, 262.5 m) | 4 | 92.3 [91.0, 93.5] | 0 never below; 4 below: 64.0, 74.8, 65.9, 63.8 s | 75.5 [70.8, 77.6] | 115.6 [95.1, 142.8] |

## All metrics

| Metric | Unit | Median [min, max] | n |
|---|---|---|---|
| Worst revisit gap (longest unseen), whole mission [0, end] | s | 109.7 [74.6, 142.8] | 12 |
| Cells unseen > 60 s at some time, whole mission | % | 75.1 [60.7, 83.2] | 12 |
| Cells unseen > 90 s at some time, whole mission | % | 6.6 [0.0, 28.0] | 12 |
| Cells unseen > 60 s, gap ending from the loss on | % | 74.4 [60.0, 83.0] | 12 |
| Cells unseen > 90 s, gap ending from the loss on | % | 6.6 [0.0, 28.0] | 12 |
| Coverage over the mission, mean | % | 95.1 [91.6, 96.7] | 12 |
| After / ((N-1)/N x P) | ratio | 1.45 [1.40, 1.47] | 12 |
| After / attainable with N-1 | ratio | 0.99 [0.96, 1.00] | 12 |
| Coverage deficit after the loss, against P | %.s | 471 [177, 1101] | 12 |
| Coverage deficit after the loss, against the no-loss N baseline | %.s | 473 [191, 1092] | 12 |
| Loss -> declared lost | s | 12.2 [11.8, 12.2] | 12 |
| Loss -> new assignment held by every survivor | s | 13.2 [12.2, 15.4] | 12 |
| Loss -> first survivor on its new loop | s | 12.4 [12.0, 12.5] | 12 |
| Largest open gap at L | s | 65.2 [64.4, 67.4] | 12 |
| Largest open gap at the end | s | 65.1 [64.9, 66.0] | 12 |
| Cells with no completed gap, before | % | 0.0 [0.0, 1.2] | 12 |
| Cells with no completed gap, after | % | 0.0 [0.0, 0.0] | 12 |
| Changes beyond the one forced by the loss | count | 0 [0, 0] | 12 |
| Min 3D separation, whole flight | m | 10.0 [10.0, 10.0] | 12 |
| Min 3D separation, from t = 10 s | m | 74.8 [72.7, 75.0] | 12 |
| Time below 8 m | s | 0.0 [0.0, 0.0] | 12 |
| Time braking, all agents | s | 0.1 [0.0, 8.4] | 12 |
| Time deaf, all agents | s | 0.0 [0.0, 0.0] | 12 |
| Infeasible reports accepted | count | 0 [0, 0] | 12 |
| Infeasible reports ignored | count | 0 [0, 0] | 12 |
| Setpoint rate, slowest agent | Hz | 20.0 [20.0, 20.0] | 12 |
| Setpoint period p99, worst agent | s | 0.050 [0.050, 0.051] | 12 |
| Setpoint period max, worst agent | s | 0.057 [0.050, 0.070] | 12 |
| Max ground speed, logged velocity (vx, vy; as the autopilot reports it) | m/s | 11.50 [11.49, 11.50] | 12 |
| Max ground speed, position differences over 1 s (reported positions) | m/s | 11.99 [11.46, 12.01] | 12 |
| Radio: loss rate | % | 19.7 [19.4, 20.5] | 12 |
| Radio: mean delay | s | 0.200 [0.200, 0.200] | 12 |
| Radio: max delay | s | 0.205 [0.201, 0.233] | 12 |

Both speeds are what the autopilots report (logged velocity and position). Under Gazebo that is the simulator's own state: its JSON backend sets AHRS_EKF_TYPE 10 (no EKF), so a GPS fault there changes nothing.

Re-plans by trigger, all runs: lost 12; in the warm-up: none.
Agents' epoch columns vs ground re-plans: no mismatch.

## Per run

| Run | Seed | Victim | Kill t (s) | Lost declared t (s) | Recovery (s) | Dip (%) | Dip at, after L (s) | From L + 60 s, mean (%) | After recovery, mean (%) | Longest unseen, whole (s) | Min separation from 10 s (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| d1_t60 | 101 | 1 | 60.0 | 72.2 | 0.0 (never below) | 87.8 | 56.0 | 95.4 | 94.3 | 109.8 | 73.1 |
| d1_t70 | 104 | 1 | 70.1 | 82.2 | 59.9 | 80.4 | 52.9 | 94.6 | 94.5 | 109.6 | 74.9 |
| d1_t80 | 107 | 1 | 80.2 | 92.2 | 58.8 | 83.7 | 40.8 | 94.8 | 94.5 | 106.1 | 74.8 |
| d1_t90 | 110 | 1 | 90.1 | 102.2 | 0.0 (never below) | 91.2 | 24.9 | 95.6 | 95.0 | 106.1 | 74.9 |
| d2_t60 | 102 | 2 | 60.1 | 72.2 | 63.9 | 84.3 | 60.9 | 94.7 | 95.2 | 123.9 | 74.8 |
| d2_t70 | 105 | 2 | 70.0 | 82.2 | 51.0 | 87.4 | 48.0 | 95.2 | 94.7 | 74.6 | 74.8 |
| d2_t80 | 108 | 2 | 80.1 | 92.0 | 62.9 | 86.3 | 33.9 | 93.7 | 94.0 | 112.2 | 72.7 |
| d2_t90 | 111 | 2 | 90.1 | 102.2 | 0.0 (never below) | 89.9 | 60.9 | 95.2 | 95.8 | 120.3 | 74.9 |
| d3_t60 | 103 | 3 | 60.0 | 71.8 | 64.0 | 76.5 | 53.0 | 93.5 | 93.9 | 127.1 | 74.9 |
| d3_t70 | 106 | 3 | 70.2 | 82.3 | 74.8 | 70.8 | 45.8 | 92.4 | 94.5 | 142.8 | 75.0 |
| d3_t80 | 109 | 3 | 80.1 | 92.2 | 65.9 | 77.1 | 55.9 | 92.2 | 93.5 | 95.1 | 74.8 |
| d3_t90 | 112 | 3 | 90.2 | 102.4 | 63.8 | 77.6 | 50.8 | 91.0 | 91.6 | 97.4 | 75.0 |
