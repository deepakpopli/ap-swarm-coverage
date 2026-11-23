# gazebo_grid_B

12 runs, backend gazebo, N = 3. Cells: median [min, max] over runs; a reference of one run is a single value. Definitions: "How to read this" below and analysis/metrics.py.

Code: commit dcc35c7fea34 (from each run's meta.json).

## How to read this

- Mission t = 0 is 60 s after every drone is on its coverage loop, so the 60 s coverage window is already full at t = 0 (no start-up ramp).
- The agent is killed at time L in [60, 90] s mission time (seeded, or chosen with --kill-at); the mission ends at 180 s.
- coverage(t) = % of 2 m cells seen in (t - 60 s, t], every 1 s. Windows: before = [L - 60 s, L); dip = lowest coverage(t) on [L, end]; from L + 60 s = [L + 60 s, end] (the victim's last sightings have left the window); after recovery = [L + recovery, end] (a run that never fell below 0.9 P counts as recovered at L); end of run = the last 30 s; mission mean and the no-loss references = [0, end].
- Longest unseen: before = [0, L), from L on = [L, end] (a gap counts where it ends). Steady revisit gap: before = [0, L), from L on = [R, end], R = declared lost. Churn: [0, L) and [L, end].
- Recovery = first time from which coverage stays >= 0.9 x P to the end. 0.0 s means it never fell below 0.9 x P, not an instant recovery.
- The task's (N-1)/N x P check (at most 66.7 %) is passed trivially at this altitude: the lowest dip of any run is 68.7 %. The stronger comparison is against the no-loss N-1 reference ("attainable with N-1").
- Reference runs used: none.

| Metric | Before [L - 60 s, L) | From L on [L, end] | From L + 60 s | After recovery [L + rec, end] | End of run (last 30 s) | Attainable with N-1 (no-loss ref) | (N-1)/N x P | n |
|---|---|---|---|---|---|---|---|---|
| Coverage, mean (%) | 100.0 [100.0, 100.0] | - | 79.0 [76.3, 81.1] | - | 80.1 [77.5, 81.0] | - | 66.7 [66.7, 66.7] | 12 |
| Coverage, min (%); from L on: the dip | 100.0 [100.0, 100.0] | 74.5 [68.7, 80.1] | 75.3 [68.7, 80.1] | - | 79.3 [72.1, 80.1] | - | - | 12 |
| Dip: when, after L (s) | - | 65.9 [49.0, 88.9] | - | - | - | - | - | 12 |
| Worst revisit gap (longest unseen) (s) | 60.2 [58.3, 64.1] | 155.4 [128.4, 176.4] | - | - | - | - | - | 12 |
| Steady revisit gap (s); from R | 58.4 [57.2, 58.6] | 65.1 [56.3, 83.7] | - | - | - | - | - | 12 |
| Assignment churn (/agent/min) | 0.00 [0.00, 0.00] | 0.57 [0.50, 0.67] | - | - | - | - | - | 12 |

- No-loss N baseline (B), mean coverage: -.
- No-loss N-1 reference (attainable with N-1), mean coverage: -.
- Recovery to 0.9 x P: 0 of 12 runs never fell below it (recovery 0.0 s); 12 fell below, 0 of them back within the run.
- Recovery to 0.9 x (N-1)/N x P (weaker reading): 12 of 12 runs recovered; over the recovered runs: 0.0 [0.0, 0.0] s.

## By victim

Cells: mean [min, max] over that victim's runs. Lanes: the victim's loop lanes in the first plan.

| Victim | Lanes before the loss | n runs | Coverage from L + 60 s, mean (%) | Recovery to 0.9 x P | Dip (%) | Longest unseen, whole (s) | Zone oldest, whole run (s) | Zone oldest from L + 40 s (s) |
|---|---|---|---|---|---|---|---|---|
| 1 | 2 (x = 42.5, 65 m) | 4 | 79.3 [76.3, 81.1] | 0 never below; 4 below, none back | 74.3 [68.7, 77.8] | 152.8 [143.8, 156.4] | 28.7 [26.5, 33.8] | 24.3 [23.5, 26.1] |
| 2 | 1 (x = 150 m) | 4 | 78.6 [77.4, 79.8] | 0 never below; 4 below, none back | 72.3 [70.7, 75.0] | 148.9 [128.4, 170.3] | 50.3 [42.8, 57.3] | 23.8 [23.3, 24.1] |
| 3 | 2 (x = 235, 257.5 m) | 4 | 79.2 [77.5, 80.9] | 0 never below; 4 below, none back | 75.1 [70.6, 80.1] | 157.9 [143.8, 176.4] | 27.7 [26.5, 30.6] | 24.1 [23.8, 24.8] |

## All metrics

| Metric | Unit | Median [min, max] | n |
|---|---|---|---|
| Worst revisit gap (longest unseen), whole mission [0, end] | s | 155.4 [128.4, 176.4] | 12 |
| Cells unseen > 60 s at some time, whole mission | % | 41.9 [34.2, 65.1] | 12 |
| Cells unseen > 90 s at some time, whole mission | % | 19.9 [13.3, 26.2] | 12 |
| Cells unseen > 60 s, gap ending from the loss on | % | 41.9 [34.2, 65.1] | 12 |
| Cells unseen > 90 s, gap ending from the loss on | % | 19.9 [13.3, 26.2] | 12 |
| Coverage over the mission, mean | % | 91.7 [89.0, 94.5] | 12 |
| After / ((N-1)/N x P) | ratio | 1.18 [1.14, 1.22] | 12 |
| Coverage deficit after the loss, against P | %.s | 1505 [1001, 1994] | 12 |
| Loss -> declared lost | s | 12.1 [12.0, 12.2] | 12 |
| Loss -> new assignment held by every survivor | s | 13.0 [12.6, 14.5] | 12 |
| Loss -> first survivor on its new loop | s | 20.9 [13.3, 27.8] | 12 |
| Largest open gap at L | s | 54.9 [54.0, 57.0] | 12 |
| Largest open gap at the end | s | 155.4 [128.4, 176.4] | 12 |
| Cells with no completed gap, before | % | 0.0 [0.0, 0.0] | 12 |
| Cells with no completed gap, after | % | 17.9 [14.5, 20.0] | 12 |
| Changes beyond the one forced by the loss | count | 0 [0, 0] | 12 |
| Min 3D separation, whole flight | m | 10.0 [10.0, 10.0] | 12 |
| Min 3D separation, from t = 10 s | m | 84.7 [68.7, 84.8] | 12 |
| Time below 8 m | s | 0.0 [0.0, 0.0] | 12 |
| Time braking, all agents | s | 0.0 [0.0, 2.6] | 12 |
| Time deaf, all agents | s | 0.0 [0.0, 0.0] | 12 |
| Infeasible reports accepted | count | 0 [0, 0] | 12 |
| Infeasible reports ignored | count | 0 [0, 0] | 12 |
| Setpoint rate, slowest agent | Hz | 20.0 [20.0, 20.0] | 12 |
| Setpoint period p99, worst agent | s | 0.051 [0.050, 0.051] | 12 |
| Setpoint period max, worst agent | s | 0.052 [0.051, 0.067] | 12 |
| Max ground speed, logged velocity (vx, vy; as the autopilot reports it) | m/s | 11.49 [11.49, 11.50] | 12 |
| Max ground speed, position differences over 1 s (reported positions) | m/s | 11.45 [11.45, 12.00] | 12 |
| Radio: loss rate | % | 19.8 [19.5, 20.5] | 12 |
| Radio: mean delay | s | 0.200 [0.200, 0.200] | 12 |
| Radio: max delay | s | 0.202 [0.201, 0.205] | 12 |
| Priority zone: longest unseen (limit 30 s) | s | 29.3 [26.5, 57.3] | 12 |
| Priority zone: time all seen within 30 s | % | 100.0 [80.7, 100.0] | 12 |
| Priority zone: oldest from L + 40 s (the new split) | s | 23.9 [23.3, 26.1] | 12 |
| Priority zone: longest with no part of it seen | s | 19.3 [17.9, 37.0] | 12 |

Both speeds are what the autopilots report (logged velocity and position). Under Gazebo that is the simulator's own state: its JSON backend sets AHRS_EKF_TYPE 10 (no EKF), so a GPS fault there changes nothing.

Re-plans by trigger, all runs: lost 12; in the warm-up: none.
Agents' epoch columns vs ground re-plans: no mismatch.

## Per run

| Run | Seed | Victim | Kill t (s) | Lost declared t (s) | Recovery (s) | Dip (%) | Dip at, after L (s) | From L + 60 s, mean (%) | After recovery, mean (%) | Longest unseen, whole (s) | Min separation from 10 s (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| d1_t60 | 101 | 1 | 60.0 | 72.2 | not recovered | 68.7 | 76.0 | 76.3 | - | 155.5 | 68.7 |
| d2_t60 | 102 | 2 | 60.1 | 72.2 | not recovered | 71.2 | 67.9 | 78.6 | - | 170.3 | 84.7 |
| d3_t60 | 103 | 3 | 60.0 | 72.2 | not recovered | 74.6 | 71.0 | 78.2 | - | 176.4 | 82.6 |
| d1_t70 | 104 | 1 | 70.1 | 82.2 | not recovered | 76.2 | 57.9 | 81.1 | - | 155.5 | 84.8 |
| d2_t70 | 105 | 2 | 70.0 | 82.2 | not recovered | 70.7 | 64.0 | 78.8 | - | 150.4 | 84.7 |
| d3_t70 | 106 | 3 | 70.0 | 82.0 | not recovered | 70.6 | 59.0 | 77.5 | - | 155.3 | 75.3 |
| d1_t80 | 107 | 1 | 80.1 | 92.2 | not recovered | 74.5 | 50.9 | 80.8 | - | 156.4 | 84.7 |
| d2_t80 | 108 | 2 | 80.1 | 92.2 | not recovered | 75.0 | 55.9 | 79.8 | - | 146.4 | 84.7 |
| d3_t80 | 109 | 3 | 80.0 | 92.1 | not recovered | 75.0 | 49.0 | 80.3 | - | 156.1 | 84.7 |
| d1_t90 | 110 | 1 | 90.2 | 102.4 | not recovered | 77.8 | 73.8 | 79.2 | - | 143.8 | 84.7 |
| d2_t90 | 111 | 2 | 90.0 | 102.2 | not recovered | 72.1 | 71.0 | 77.4 | - | 128.4 | 84.7 |
| d3_t90 | 112 | 3 | 90.1 | 102.2 | not recovered | 80.1 | 88.9 | 80.9 | - | 143.8 | 84.7 |
