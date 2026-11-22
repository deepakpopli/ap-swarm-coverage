# sitl_50m

5 runs, backend sitl, N = 3. Cells: median [min, max] over runs. Definitions: docs/metrics.md.

| Metric | Before the loss | After the loss (from L + 60 s) | End of run (last 30 s) | Attainable with N-1 | (N-1)/N x before | n |
|---|---|---|---|---|---|---|
| Worst revisit gap (longest unseen) (s) | 66.2 [66.2, 66.3] | 104.0 [90.4, 105.6] | - | - | - | 5 |
| Coverage, mean (%) | 97.8 [97.5, 97.9] | 94.2 [91.3, 95.9] | 95.8 [95.2, 95.9] | 95.4 [95.4, 95.4] | 65.2 [65.0, 65.3] | 5 |
| Coverage, min (%) | 96.0 [96.0, 96.1] | 85.7 [79.2, 93.2] | 93.0 [92.1, 93.2] | - | - | 5 |
| Steady revisit gap (both sightings inside the phase) (s) | 63.7 [60.6, 65.3] | 66.0 [66.0, 66.2] | - | - | - | 5 |
| Assignment churn (/agent/min) | 0.00 [0.00, 0.00] | 0.33 [0.26, 0.59] | - | - | - | 5 |

Revisit gap phases differ from the coverage phases. Longest unseen: before = [0, L), after = [L, end]; a gap belongs to the phase where it ends, a gap still open at the end counts as its age then. Steady gap: before = [0, L), after = [R, end] with R = lost declared, survivors only.

- Recovery to 0.9 x P: 5 of 5 runs recovered; over the recovered runs: 62.9 [0.0, 77.8] s.
- Recovery to 0.9 x the no-loss N baseline (97.7 [97.7, 97.7] %): 5 of 5 runs recovered; over the recovered runs: 62.9 [0.0, 77.8] s.
- Recovery to 0.9 x (N-1)/N x P (weaker reading): 5 of 5 runs recovered; over the recovered runs: 0.0 [0.0, 0.0] s.

| Metric | Unit | Median [min, max] | n |
|---|---|---|---|
| Worst revisit gap (longest unseen), whole mission [0, end] | s | 104.0 [90.4, 105.6] | 5 |
| Cells unseen > 60 s at some time, whole mission | % | 75.1 [61.4, 77.2] | 5 |
| Cells unseen > 90 s at some time, whole mission | % | 5.1 [0.0, 32.1] | 5 |
| Cells unseen > 60 s, gap ending from the loss on | % | 75.0 [61.4, 77.1] | 5 |
| Cells unseen > 90 s, gap ending from the loss on | % | 5.1 [0.0, 32.1] | 5 |
| Coverage over the mission, mean | % | 94.9 [91.7, 96.3] | 5 |
| No-loss N runs, mean | % | 97.7 [97.7, 97.7] | 1 |
| After / ((N-1)/N x P) | ratio | 1.45 [1.40, 1.47] | 5 |
| After / attainable with N-1 | ratio | 0.99 [0.96, 1.00] | 5 |
| Dip: lowest coverage after the loss | % | 79.2 [75.5, 92.1] | 5 |
| Dip: time after the loss | s | 54.9 [26.3, 113.8] | 5 |
| Coverage deficit after the loss, against P | %.s | 531 [254, 1111] | 5 |
| Coverage deficit after the loss, against the no-loss N baseline | %.s | 529 [269, 1094] | 5 |
| Loss -> declared lost | s | 12.1 [12.0, 12.2] | 5 |
| Loss -> new assignment held by every survivor | s | 12.4 [12.3, 14.5] | 5 |
| Loss -> first survivor on its new loop | s | 12.3 [12.3, 12.5] | 5 |
| Largest open gap at L | s | 63.8 [60.8, 64.4] | 5 |
| Largest open gap at the end | s | 64.5 [64.3, 64.5] | 5 |
| Cells with no completed gap, before | % | 0.0 [0.0, 0.0] | 5 |
| Cells with no completed gap, after | % | 0.0 [0.0, 0.0] | 5 |
| Changes beyond the one forced by the loss | count | 0 [0, 0] | 5 |
| Min 3D separation, whole flight | m | 10.0 [10.0, 10.0] | 5 |
| Min 3D separation, from t = 10 s | m | 74.9 [74.9, 75.0] | 5 |
| Time below 8 m | s | 0.0 [0.0, 0.0] | 5 |
| Time braking, all agents | s | 0.0 [0.0, 2.0] | 5 |
| Time deaf, all agents | s | 0.0 [0.0, 0.0] | 5 |
| Infeasible reports accepted | count | 0 [0, 0] | 5 |
| Infeasible reports ignored | count | 0 [0, 0] | 5 |
| Setpoint rate, slowest agent | Hz | 20.0 [20.0, 20.0] | 5 |
| Setpoint period p99, worst agent | s | 0.051 [0.051, 0.059] | 5 |
| Setpoint period max, worst agent | s | 0.052 [0.051, 0.102] | 5 |
| Max ground speed | m/s | 11.66 [11.65, 11.66] | 5 |
| Radio: loss rate | % | 20.0 [19.5, 20.1] | 5 |
| Radio: mean delay | s | 0.200 [0.200, 0.200] | 5 |
| Radio: max delay | s | 0.202 [0.201, 0.222] | 5 |

Re-plans by trigger, all runs: lost 5; in the warm-up: none.
Agents' epoch columns vs ground re-plans: no mismatch.

## Per run

| Run | Seed | Victim | Kill t (s) | Lost declared t (s) | Recovery (s) | After, mean (%) | Longest unseen, whole (s) | Min separation from 10 s (m) |
|---|---|---|---|---|---|---|---|---|
| seed_1 | 1 | 1 | 64.2 | 76.4 | 0.0 | 95.9 | 105.6 | 75.0 |
| seed_2 | 2 | 1 | 88.7 | 100.8 | 0.0 | 95.8 | 104.0 | 74.9 |
| seed_3 | 3 | 3 | 67.2 | 79.4 | 77.8 | 91.3 | 90.4 | 74.9 |
| seed_4 | 4 | 1 | 67.1 | 79.2 | 62.9 | 94.2 | 105.4 | 74.9 |
| seed_5 | 5 | 3 | 78.8 | 90.8 | 67.2 | 93.1 | 96.5 | 75.0 |
