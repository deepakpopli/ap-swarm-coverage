# gazebo_grid_B_old

12 runs, backend gazebo, N = 3. Cells: median [min, max] over runs; a reference of one run is a single value. Definitions: "How to read this" below and analysis/metrics.py.

Code: commit c68ee2279695 (from each run's meta.json).

## How to read this

- Mission t = 0 is 60 s after every drone is on its coverage loop, so the 60 s coverage window is already full at t = 0 (no start-up ramp).
- The agent is killed at time L in [60, 90] s mission time (seeded, or chosen with --kill-at); the mission ends at 180 s.
- coverage(t) = % of 2 m cells seen in (t - 60 s, t], every 1 s. Windows: before = [L - 60 s, L); dip = lowest coverage(t) on [L, end]; from L + 60 s = [L + 60 s, end] (the victim's last sightings have left the window); after recovery = [L + recovery, end] (a run that never fell below 0.9 P counts as recovered at L); end of run = the last 30 s; mission mean and the no-loss references = [0, end].
- Longest unseen: before = [0, L), from L on = [L, end] (a gap counts where it ends). Steady revisit gap: before = [0, L), from L on = [R, end], R = declared lost. Churn: [0, L) and [L, end].
- Recovery = first time from which coverage stays >= 0.9 x P to the end. 0.0 s means it never fell below 0.9 x P, not an instant recovery.
- The task's (N-1)/N x P check (about 67 %) is passed trivially at this altitude: the lowest dip of any run is 59.4 %. The stronger comparison is against the no-loss N-1 reference ("attainable with N-1").
- Reference runs used: none.

| Metric | Before [L - 60 s, L) | From L on [L, end] | From L + 60 s | After recovery [L + rec, end] | End of run (last 30 s) | Attainable with N-1 (no-loss ref) | (N-1)/N x P | n |
|---|---|---|---|---|---|---|---|---|
| Coverage, mean (%) | 100.0 [100.0, 100.0] | - | 68.6 [64.5, 73.4] | - | 69.5 [64.4, 73.7] | - | 66.7 [66.7, 66.7] | 12 |
| Coverage, min (%); from L on: the dip | 100.0 [100.0, 100.0] | 63.9 [59.4, 66.2] | 63.9 [59.7, 66.2] | - | 63.9 [59.4, 66.2] | - | - | 12 |
| Dip: when, after L (s) | - | 82.4 [60.0, 112.0] | - | - | - | - | - | 12 |
| Worst revisit gap (longest unseen) (s) | 58.4 [57.5, 61.2] | 150.5 [130.9, 167.3] | - | - | - | - | - | 12 |
| Steady revisit gap (s); from R | 58.3 [56.7, 58.4] | 76.7 [50.5, 99.3] | - | - | - | - | - | 12 |
| Assignment churn (/agent/min) | 0.00 [0.00, 0.00] | 0.32 [0.25, 0.67] | - | - | - | - | - | 12 |

- No-loss N baseline (B), mean coverage: -.
- No-loss N-1 reference (attainable with N-1), mean coverage: -.
- Recovery to 0.9 x P: 0 of 12 runs never fell below it (recovery 0.0 s); 12 fell below, 0 of them back within the run.
- Recovery to 0.9 x (N-1)/N x P (weaker reading): 12 of 12 runs recovered; over the recovered runs: 0.0 [0.0, 62.0] s.

## By victim

Cells: mean [min, max] over that victim's runs. Lanes: the victim's loop lanes in the first plan.

| Victim | Lanes before the loss | n runs | Coverage from L + 60 s, mean (%) | Recovery to 0.9 x P | Dip (%) | Longest unseen, whole (s) | Zone oldest, whole run (s) | Zone oldest from L + 40 s (s) |
|---|---|---|---|---|---|---|---|---|
| 1 | 2 (x = 42.5, 65 m) | 4 | 67.8 [64.6, 69.6] | 0 never below; 4 below, none back | 62.8 [59.4, 64.0] | 149.8 [144.2, 152.7] | 26.9 [26.5, 27.4] | 26.6 [26.2, 27.1] |
| 2 | 1 (x = 150 m) | 4 | 71.0 [68.1, 73.4] | 0 never below; 4 below, none back | 64.7 [63.0, 66.2] | 147.9 [130.9, 167.3] | 54.5 [37.3, 65.4] | 26.2 [26.2, 26.3] |
| 3 | 2 (x = 235, 257.5 m) | 4 | 67.8 [64.5, 69.5] | 0 never below; 4 below, none back | 62.8 [59.7, 64.0] | 149.7 [143.9, 153.0] | 27.2 [26.5, 28.5] | 26.5 [26.2, 27.2] |

## All metrics

| Metric | Unit | Median [min, max] | n |
|---|---|---|---|
| Worst revisit gap (longest unseen), whole mission [0, end] | s | 150.5 [130.9, 167.3] | 12 |
| Cells unseen > 60 s at some time, whole mission | % | 49.0 [44.5, 70.2] | 12 |
| Cells unseen > 90 s at some time, whole mission | % | 35.3 [15.3, 39.5] | 12 |
| Cells unseen > 60 s, gap ending from the loss on | % | 49.0 [44.5, 70.2] | 12 |
| Cells unseen > 90 s, gap ending from the loss on | % | 35.3 [15.3, 39.5] | 12 |
| Coverage over the mission, mean | % | 89.0 [86.5, 92.6] | 12 |
| After / ((N-1)/N x P) | ratio | 1.03 [0.97, 1.10] | 12 |
| Coverage deficit after the loss, against P | %.s | 1990 [1340, 2438] | 12 |
| Loss -> declared lost | s | 12.1 [12.0, 12.2] | 12 |
| Loss -> new assignment held by every survivor | s | 12.4 [12.3, 14.3] | 12 |
| Loss -> first survivor on its new loop | s | 12.4 [12.3, 13.4] | 12 |
| Largest open gap at L | s | 54.7 [54.0, 56.5] | 12 |
| Largest open gap at the end | s | 150.5 [130.9, 167.3] | 12 |
| Cells with no completed gap, before | % | 0.0 [0.0, 0.0] | 12 |
| Cells with no completed gap, after | % | 17.5 [10.0, 27.2] | 12 |
| Changes beyond the one forced by the loss | count | 0 [0, 0] | 12 |
| Min 3D separation, whole flight | m | 10.0 [10.0, 10.0] | 12 |
| Min 3D separation, from t = 10 s | m | 58.0 [24.2, 84.7] | 12 |
| Time below 8 m | s | 0.0 [0.0, 0.0] | 12 |
| Time braking, all agents | s | 1.6 [0.0, 6.0] | 12 |
| Time deaf, all agents | s | 0.0 [0.0, 0.0] | 12 |
| Infeasible reports accepted | count | 0 [0, 0] | 12 |
| Infeasible reports ignored | count | 0 [0, 0] | 12 |
| Setpoint rate, slowest agent | Hz | 20.0 [20.0, 20.0] | 12 |
| Setpoint period p99, worst agent | s | 0.051 [0.050, 0.051] | 12 |
| Setpoint period max, worst agent | s | 0.051 [0.051, 0.053] | 12 |
| Max ground speed, logged velocity (vx, vy; as the autopilot reports it) | m/s | 11.50 [11.49, 11.50] | 12 |
| Max ground speed, position differences over 1 s (reported positions) | m/s | 11.98 [11.50, 12.01] | 12 |
| Radio: loss rate | % | 19.7 [19.5, 20.5] | 12 |
| Radio: mean delay | s | 0.200 [0.200, 0.200] | 12 |
| Radio: max delay | s | 0.201 [0.201, 0.204] | 12 |
| Priority zone: longest unseen (limit 30 s) | s | 27.3 [26.5, 65.4] | 12 |
| Priority zone: time all seen within 30 s | % | 100.0 [80.7, 100.0] | 12 |
| Priority zone: oldest from L + 40 s (the new split) | s | 26.2 [26.2, 27.2] | 12 |
| Priority zone: longest with no part of it seen | s | 16.3 [16.1, 35.6] | 12 |

Both speeds are what the autopilots report (logged velocity and position). Under Gazebo that is the simulator's own state: its JSON backend sets AHRS_EKF_TYPE 10 (no EKF), so a GPS fault there changes nothing.

Re-plans by trigger, all runs: lost 12; in the warm-up: none.
Agents' epoch columns vs ground re-plans: no mismatch.

## Per run

| Run | Seed | Victim | Kill t (s) | Lost declared t (s) | Recovery (s) | Dip (%) | Dip at, after L (s) | From L + 60 s, mean (%) | After recovery, mean (%) | Longest unseen, whole (s) | Min separation from 10 s (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| d1_t60 | 101 | 1 | 60.2 | 72.2 | not recovered | 63.8 | 111.8 | 68.5 | - | 150.5 | 38.9 |
| d1_t70 | 104 | 1 | 70.2 | 82.4 | not recovered | 64.0 | 81.8 | 68.6 | - | 152.7 | 79.3 |
| d1_t80 | 107 | 1 | 80.1 | 92.2 | not recovered | 64.0 | 71.9 | 69.6 | - | 151.7 | 72.9 |
| d1_t90 | 110 | 1 | 90.0 | 102.2 | not recovered | 59.4 | 60.0 | 64.6 | - | 144.2 | 84.7 |
| d2_t60 | 102 | 2 | 60.2 | 72.2 | not recovered | 63.9 | 93.8 | 68.1 | - | 167.3 | 42.9 |
| d2_t70 | 105 | 2 | 70.1 | 82.2 | not recovered | 63.0 | 84.9 | 70.7 | - | 150.6 | 29.9 |
| d2_t80 | 108 | 2 | 80.1 | 92.2 | not recovered | 65.6 | 82.9 | 71.8 | - | 143.0 | 24.2 |
| d2_t90 | 111 | 2 | 90.1 | 102.2 | not recovered | 66.2 | 82.9 | 73.4 | - | 130.9 | 32.8 |
| d3_t60 | 103 | 3 | 60.0 | 72.1 | not recovered | 63.8 | 112.0 | 68.5 | - | 150.4 | 43.2 |
| d3_t70 | 106 | 3 | 70.1 | 82.2 | not recovered | 63.9 | 81.9 | 68.5 | - | 153.0 | 83.8 |
| d3_t80 | 109 | 3 | 80.1 | 92.2 | not recovered | 64.0 | 71.9 | 69.5 | - | 151.6 | 73.0 |
| d3_t90 | 112 | 3 | 90.1 | 102.2 | not recovered | 59.7 | 60.9 | 64.5 | - | 143.9 | 84.7 |
