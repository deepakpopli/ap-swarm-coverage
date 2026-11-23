# gazebo_grid_B

12 runs, backend gazebo, N = 3. Cells: median [min, max] over runs; a reference of one run is a single value. Definitions: "How to read this" below and analysis/metrics.py.

Code: commit 7c9bb15f7d9c (from each run's meta.json).

## How to read this

- Mission t = 0 is 60 s after every drone is on its coverage loop, so the 60 s coverage window is already full at t = 0 (no start-up ramp).
- The agent is killed at time L in [60, 90] s mission time (seeded, or chosen with --kill-at); the mission ends at 180 s.
- coverage(t) = % of 2 m cells seen in (t - 60 s, t], every 1 s. Windows: before = [L - 60 s, L); dip = lowest coverage(t) on [L, end]; from L + 60 s = [L + 60 s, end] (the victim's last sightings have left the window); after recovery = [L + recovery, end] (a run that never fell below 0.9 P counts as recovered at L); end of run = the last 30 s; mission mean and the no-loss references = [0, end].
- Longest unseen: before = [0, L), from L on = [L, end] (a gap counts where it ends). Steady revisit gap: before = [0, L), from L on = [R, end], R = declared lost. Churn: [0, L) and [L, end].
- Recovery = first time from which coverage stays >= 0.9 x P to the end. 0.0 s means it never fell below 0.9 x P, not an instant recovery.
- The task's (N-1)/N x P check (about 67 %) is passed trivially at this altitude: the lowest dip of any run is 68.6 %. The stronger comparison is against the no-loss N-1 reference ("attainable with N-1").
- Reference runs used: none.

| Metric | Before [L - 60 s, L) | From L on [L, end] | From L + 60 s | After recovery [L + rec, end] | End of run (last 30 s) | Attainable with N-1 (no-loss ref) | (N-1)/N x P | n |
|---|---|---|---|---|---|---|---|---|
| Coverage, mean (%) | 100.0 [100.0, 100.0] | - | 79.4 [76.3, 81.8] | - | 80.0 [77.5, 81.2] | - | 66.7 [66.7, 66.7] | 12 |
| Coverage, min (%); from L on: the dip | 100.0 [100.0, 100.0] | 73.2 [68.6, 79.4] | 75.1 [68.6, 80.0] | - | 79.7 [72.0, 80.1] | - | - | 12 |
| Dip: when, after L (s) | - | 62.5 [51.9, 80.8] | - | - | - | - | - | 12 |
| Worst revisit gap (longest unseen) (s) | 58.3 [57.4, 61.1] | 155.4 [128.7, 156.7] | - | - | - | - | - | 12 |
| Steady revisit gap (s); from R | 58.2 [56.8, 58.5] | 59.8 [53.8, 96.6] | - | - | - | - | - | 12 |
| Assignment churn (/agent/min) | 0.00 [0.00, 0.00] | 0.57 [0.50, 0.67] | - | - | - | - | - | 12 |

- No-loss N baseline (B), mean coverage: -.
- No-loss N-1 reference (attainable with N-1), mean coverage: -.
- Recovery to 0.9 x P: 0 of 12 runs never fell below it (recovery 0.0 s); 12 fell below, 0 of them back within the run.
- Recovery to 0.9 x (N-1)/N x P (weaker reading): 12 of 12 runs recovered; over the recovered runs: 0.0 [0.0, 0.0] s.

## By victim

Cells: mean [min, max] over that victim's runs. Lanes: the victim's loop lanes in the first plan.

| Victim | Lanes before the loss | n runs | Coverage from L + 60 s, mean (%) | Recovery to 0.9 x P | Dip (%) | Longest unseen, whole (s) | Zone oldest, whole run (s) | Zone oldest from L + 40 s (s) |
|---|---|---|---|---|---|---|---|---|
| 1 | 2 (x = 42.5, 65 m) | 4 | 79.9 [76.3, 81.8] | 0 never below; 4 below, none back | 74.7 [68.6, 79.4] | 152.7 [143.7, 156.2] | 28.9 [26.4, 34.7] | 23.9 [23.6, 24.6] |
| 2 | 1 (x = 150 m) | 4 | 78.5 [77.4, 79.3] | 0 never below; 4 below, none back | 72.6 [71.4, 75.0] | 145.8 [128.7, 156.7] | 51.1 [43.7, 58.7] | 24.8 [23.5, 28.5] |
| 3 | 2 (x = 235, 257.5 m) | 4 | 79.4 [77.6, 80.4] | 0 never below; 4 below, none back | 74.0 [70.7, 79.1] | 153.1 [144.1, 156.4] | 27.3 [26.4, 28.2] | 24.1 [23.4, 25.9] |

## All metrics

| Metric | Unit | Median [min, max] | n |
|---|---|---|---|
| Worst revisit gap (longest unseen), whole mission [0, end] | s | 155.4 [128.7, 156.7] | 12 |
| Cells unseen > 60 s at some time, whole mission | % | 41.6 [34.4, 65.1] | 12 |
| Cells unseen > 90 s at some time, whole mission | % | 20.5 [13.5, 25.5] | 12 |
| Cells unseen > 60 s, gap ending from the loss on | % | 41.6 [34.4, 65.1] | 12 |
| Cells unseen > 90 s, gap ending from the loss on | % | 20.5 [13.5, 25.5] | 12 |
| Coverage over the mission, mean | % | 91.6 [89.0, 94.5] | 12 |
| After / ((N-1)/N x P) | ratio | 1.19 [1.14, 1.23] | 12 |
| Coverage deficit after the loss, against P | %.s | 1511 [1002, 1986] | 12 |
| Loss -> declared lost | s | 12.1 [12.0, 12.2] | 12 |
| Loss -> new assignment held by every survivor | s | 12.9 [12.6, 14.3] | 12 |
| Loss -> first survivor on its new loop | s | 20.0 [17.4, 27.1] | 12 |
| Largest open gap at L | s | 54.6 [54.0, 56.5] | 12 |
| Largest open gap at the end | s | 155.4 [128.7, 156.7] | 12 |
| Cells with no completed gap, before | % | 0.0 [0.0, 0.0] | 12 |
| Cells with no completed gap, after | % | 17.9 [13.6, 20.1] | 12 |
| Changes beyond the one forced by the loss | count | 0 [0, 0] | 12 |
| Min 3D separation, whole flight | m | 10.0 [10.0, 10.0] | 12 |
| Min 3D separation, from t = 10 s | m | 84.7 [68.3, 84.8] | 12 |
| Time below 8 m | s | 0.0 [0.0, 0.0] | 12 |
| Time braking, all agents | s | 0.0 [0.0, 4.0] | 12 |
| Time deaf, all agents | s | 0.0 [0.0, 0.0] | 12 |
| Infeasible reports accepted | count | 0 [0, 0] | 12 |
| Infeasible reports ignored | count | 0 [0, 0] | 12 |
| Setpoint rate, slowest agent | Hz | 20.0 [20.0, 20.0] | 12 |
| Setpoint period p99, worst agent | s | 0.051 [0.050, 0.051] | 12 |
| Setpoint period max, worst agent | s | 0.051 [0.050, 0.053] | 12 |
| Max ground speed, logged velocity (vx, vy; as the autopilot reports it) | m/s | 11.49 [11.49, 11.50] | 12 |
| Max ground speed, position differences over 1 s (reported positions) | m/s | 11.57 [11.45, 12.01] | 12 |
| Radio: loss rate | % | 19.7 [19.5, 20.5] | 12 |
| Radio: mean delay | s | 0.200 [0.200, 0.200] | 12 |
| Radio: max delay | s | 0.201 [0.201, 0.202] | 12 |
| Priority zone: longest unseen (limit 30 s) | s | 28.2 [26.4, 58.7] | 12 |
| Priority zone: time all seen within 30 s | % | 100.0 [80.1, 100.0] | 12 |
| Priority zone: oldest from L + 40 s (the new split) | s | 23.6 [23.4, 28.5] | 12 |
| Priority zone: longest with no part of it seen | s | 19.1 [17.8, 36.8] | 12 |

Both speeds are what the autopilots report (logged velocity and position). Under Gazebo that is the simulator's own state: its JSON backend sets AHRS_EKF_TYPE 10 (no EKF), so a GPS fault there changes nothing.

Re-plans by trigger, all runs: lost 12; in the warm-up: none.
Agents' epoch columns vs ground re-plans: no mismatch.

## Per run

| Run | Seed | Victim | Kill t (s) | Lost declared t (s) | Recovery (s) | Dip (%) | Dip at, after L (s) | From L + 60 s, mean (%) | After recovery, mean (%) | Longest unseen, whole (s) | Min separation from 10 s (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| d1_t60 | 101 | 1 | 60.2 | 72.2 | not recovered | 68.6 | 75.8 | 76.3 | - | 155.3 | 68.3 |
| d1_t70 | 104 | 1 | 70.1 | 82.2 | not recovered | 75.9 | 57.9 | 81.0 | - | 155.7 | 84.7 |
| d1_t80 | 107 | 1 | 80.1 | 92.2 | not recovered | 74.8 | 51.9 | 81.8 | - | 156.2 | 84.7 |
| d1_t90 | 110 | 1 | 90.2 | 102.2 | not recovered | 79.4 | 80.8 | 80.4 | - | 143.7 | 84.7 |
| d2_t60 | 102 | 2 | 60.2 | 72.4 | not recovered | 71.4 | 67.8 | 78.6 | - | 156.7 | 84.7 |
| d2_t70 | 105 | 2 | 70.0 | 82.2 | not recovered | 71.9 | 64.0 | 78.8 | - | 150.8 | 84.7 |
| d2_t80 | 108 | 2 | 80.0 | 92.2 | not recovered | 75.0 | 56.0 | 79.3 | - | 147.1 | 84.7 |
| d2_t90 | 111 | 2 | 90.2 | 102.2 | not recovered | 72.0 | 69.8 | 77.4 | - | 128.7 | 84.8 |
| d3_t60 | 103 | 3 | 60.0 | 72.1 | not recovered | 74.4 | 61.0 | 79.6 | - | 156.4 | 81.6 |
| d3_t70 | 106 | 3 | 70.0 | 82.2 | not recovered | 70.7 | 59.0 | 77.6 | - | 155.5 | 75.1 |
| d3_t80 | 109 | 3 | 80.1 | 92.2 | not recovered | 71.6 | 52.9 | 80.4 | - | 156.4 | 84.7 |
| d3_t90 | 112 | 3 | 90.1 | 102.2 | not recovered | 79.1 | 69.9 | 80.0 | - | 144.1 | 84.7 |
