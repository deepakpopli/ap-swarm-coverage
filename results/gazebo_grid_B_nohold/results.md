# gazebo_grid_B_nohold

12 runs, backend gazebo, N = 3. Cells: median [min, max] over runs; a reference of one run is a single value. Definitions: "How to read this" below and analysis/metrics.py.

Code: commit e8f0526e89de (from each run's meta.json).

## How to read this

- Mission t = 0 is 60 s after every drone is on its coverage loop, so the 60 s coverage window is already full at t = 0 (no start-up ramp).
- The agent is killed at time L in [60, 90] s mission time (seeded, or chosen with --kill-at); the mission ends at 180 s.
- coverage(t) = % of 2 m cells seen in (t - 60 s, t], every 1 s. Windows: before = [L - 60 s, L); dip = lowest coverage(t) on [L, end]; from L + 60 s = [L + 60 s, end] (the victim's last sightings have left the window); after recovery = [L + recovery, end] (a run that never fell below 0.9 P counts as recovered at L); end of run = the last 30 s; mission mean and the no-loss references = [0, end].
- Longest unseen: before = [0, L), from L on = [L, end] (a gap counts where it ends). Steady revisit gap: before = [0, L), from L on = [R, end], R = declared lost. Churn: [0, L) and [L, end].
- Recovery = first time from which coverage stays >= 0.9 x P to the end. 0.0 s means it never fell below 0.9 x P, not an instant recovery.
- The task's (N-1)/N x P check (about 67 %) is passed trivially at this altitude: the lowest dip of any run is 66.7 %. The stronger comparison is against the no-loss N-1 reference ("attainable with N-1").
- Reference runs used: none.

| Metric | Before [L - 60 s, L) | From L on [L, end] | From L + 60 s | After recovery [L + rec, end] | End of run (last 30 s) | Attainable with N-1 (no-loss ref) | (N-1)/N x P | n |
|---|---|---|---|---|---|---|---|---|
| Coverage, mean (%) | 100.0 [100.0, 100.0] | - | 79.1 [77.1, 81.9] | - | 80.1 [77.6, 82.1] | - | 66.7 [66.7, 66.7] | 12 |
| Coverage, min (%); from L on: the dip | 100.0 [100.0, 100.0] | 73.2 [66.7, 80.7] | 74.5 [67.4, 80.7] | - | 80.0 [72.5, 80.7] | - | - | 12 |
| Dip: when, after L (s) | - | 62.4 [47.9, 86.9] | - | - | - | - | - | 12 |
| Worst revisit gap (longest unseen) (s) | 58.5 [58.2, 61.4] | 155.4 [128.6, 156.8] | - | - | - | - | - | 12 |
| Steady revisit gap (s); from R | 58.2 [56.8, 58.5] | 58.9 [53.7, 66.2] | - | - | - | - | - | 12 |
| Assignment churn (/agent/min) | 0.00 [0.00, 0.00] | 0.57 [0.50, 0.67] | - | - | - | - | - | 12 |

- No-loss N baseline (B), mean coverage: -.
- No-loss N-1 reference (attainable with N-1), mean coverage: -.
- Recovery to 0.9 x P: 0 of 12 runs never fell below it (recovery 0.0 s); 12 fell below, 0 of them back within the run.
- Recovery to 0.9 x (N-1)/N x P (weaker reading): 12 of 12 runs recovered; over the recovered runs: 0.0 [0.0, 0.0] s.

## By victim

Cells: mean [min, max] over that victim's runs. Lanes: the victim's loop lanes in the first plan.

| Victim | Lanes before the loss | n runs | Coverage from L + 60 s, mean (%) | Recovery to 0.9 x P | Dip (%) | Longest unseen, whole (s) | Zone oldest, whole run (s) | Zone oldest from L + 40 s (s) |
|---|---|---|---|---|---|---|---|---|
| 1 | 2 (x = 42.5, 65 m) | 4 | 79.9 [77.1, 81.9] | 0 never below; 4 below, none back | 75.2 [66.7, 80.7] | 152.7 [143.7, 156.2] | 30.4 [26.8, 34.3] | 27.4 [23.9, 31.9] |
| 2 | 1 (x = 150 m) | 4 | 78.5 [77.4, 79.2] | 0 never below; 4 below, none back | 72.8 [71.1, 75.8] | 145.7 [128.6, 156.8] | 51.7 [45.8, 58.7] | 26.4 [22.9, 30.2] |
| 3 | 2 (x = 235, 257.5 m) | 4 | 79.8 [78.0, 81.3] | 0 never below; 4 below, none back | 74.0 [70.7, 80.1] | 153.0 [144.1, 156.4] | 27.5 [26.4, 28.6] | 25.1 [23.8, 27.0] |

## All metrics

| Metric | Unit | Median [min, max] | n |
|---|---|---|---|
| Worst revisit gap (longest unseen), whole mission [0, end] | s | 155.4 [128.6, 156.8] | 12 |
| Cells unseen > 60 s at some time, whole mission | % | 42.7 [33.2, 53.5] | 12 |
| Cells unseen > 90 s at some time, whole mission | % | 20.5 [12.8, 26.1] | 12 |
| Cells unseen > 60 s, gap ending from the loss on | % | 42.7 [33.2, 53.5] | 12 |
| Cells unseen > 90 s, gap ending from the loss on | % | 20.5 [12.8, 26.1] | 12 |
| Coverage over the mission, mean | % | 91.7 [89.3, 94.5] | 12 |
| After / ((N-1)/N x P) | ratio | 1.19 [1.16, 1.23] | 12 |
| Coverage deficit after the loss, against P | %.s | 1508 [1000, 1939] | 12 |
| Loss -> declared lost | s | 12.1 [11.9, 12.2] | 12 |
| Loss -> new assignment held by every survivor | s | 13.1 [12.5, 14.4] | 12 |
| Loss -> first survivor on its new loop | s | 21.3 [13.4, 27.3] | 12 |
| Largest open gap at L | s | 54.6 [54.0, 56.4] | 12 |
| Largest open gap at the end | s | 155.4 [128.6, 156.8] | 12 |
| Cells with no completed gap, before | % | 0.0 [0.0, 0.0] | 12 |
| Cells with no completed gap, after | % | 18.2 [13.8, 20.2] | 12 |
| Changes beyond the one forced by the loss | count | 0 [0, 0] | 12 |
| Min 3D separation, whole flight | m | 10.0 [10.0, 10.0] | 12 |
| Min 3D separation, from t = 10 s | m | 84.7 [59.0, 84.8] | 12 |
| Time below 8 m | s | 0.0 [0.0, 0.0] | 12 |
| Time braking, all agents | s | 0.0 [0.0, 4.3] | 12 |
| Time deaf, all agents | s | 0.0 [0.0, 0.0] | 12 |
| Infeasible reports accepted | count | 0 [0, 0] | 12 |
| Infeasible reports ignored | count | 0 [0, 0] | 12 |
| Setpoint rate, slowest agent | Hz | 20.0 [20.0, 20.0] | 12 |
| Setpoint period p99, worst agent | s | 0.051 [0.050, 0.051] | 12 |
| Setpoint period max, worst agent | s | 0.051 [0.050, 0.052] | 12 |
| Max ground speed, logged velocity (vx, vy; as the autopilot reports it) | m/s | 11.49 [11.49, 11.50] | 12 |
| Max ground speed, position differences over 1 s (reported positions) | m/s | 11.97 [11.44, 12.01] | 12 |
| Radio: loss rate | % | 19.7 [19.5, 20.5] | 12 |
| Radio: mean delay | s | 0.200 [0.200, 0.200] | 12 |
| Radio: max delay | s | 0.201 [0.201, 0.202] | 12 |
| Priority zone: longest unseen (limit 30 s) | s | 30.6 [26.4, 58.7] | 12 |
| Priority zone: time all seen within 30 s | % | 97.5 [79.0, 100.0] | 12 |
| Priority zone: oldest from L + 40 s (the new split) | s | 26.0 [22.9, 31.9] | 12 |
| Priority zone: longest with no part of it seen | s | 22.3 [18.0, 36.8] | 12 |

Both speeds are what the autopilots report (logged velocity and position). Under Gazebo that is the simulator's own state: its JSON backend sets AHRS_EKF_TYPE 10 (no EKF), so a GPS fault there changes nothing.

Re-plans by trigger, all runs: lost 12; in the warm-up: none.
Agents' epoch columns vs ground re-plans: no mismatch.

## Per run

| Run | Seed | Victim | Kill t (s) | Lost declared t (s) | Recovery (s) | Dip (%) | Dip at, after L (s) | From L + 60 s, mean (%) | After recovery, mean (%) | Longest unseen, whole (s) | Min separation from 10 s (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| d1_t60 | 101 | 1 | 60.1 | 72.1 | not recovered | 75.0 | 60.9 | 79.0 | - | 155.4 | 83.3 |
| d1_t70 | 104 | 1 | 70.0 | 82.2 | not recovered | 66.7 | 60.0 | 77.1 | - | 155.4 | 59.0 |
| d1_t80 | 107 | 1 | 80.1 | 92.2 | not recovered | 78.5 | 47.9 | 81.6 | - | 156.2 | 84.6 |
| d1_t90 | 110 | 1 | 90.1 | 102.0 | not recovered | 80.7 | 84.9 | 81.9 | - | 143.7 | 84.8 |
| d2_t60 | 102 | 2 | 60.2 | 72.4 | not recovered | 71.1 | 68.8 | 78.6 | - | 156.8 | 84.7 |
| d2_t70 | 105 | 2 | 70.1 | 82.2 | not recovered | 72.0 | 63.9 | 79.0 | - | 150.8 | 84.7 |
| d2_t80 | 108 | 2 | 80.1 | 92.2 | not recovered | 75.8 | 63.9 | 79.2 | - | 146.6 | 84.7 |
| d2_t90 | 111 | 2 | 90.1 | 102.2 | not recovered | 72.5 | 69.9 | 77.4 | - | 128.6 | 84.8 |
| d3_t60 | 103 | 3 | 60.2 | 72.2 | not recovered | 73.9 | 60.8 | 79.6 | - | 156.3 | 81.6 |
| d3_t70 | 106 | 3 | 70.0 | 82.2 | not recovered | 70.7 | 59.0 | 78.0 | - | 155.4 | 75.2 |
| d3_t80 | 109 | 3 | 80.1 | 92.0 | not recovered | 71.5 | 52.9 | 80.4 | - | 156.4 | 84.7 |
| d3_t90 | 112 | 3 | 90.1 | 102.2 | not recovered | 80.1 | 86.9 | 81.3 | - | 144.1 | 84.7 |
