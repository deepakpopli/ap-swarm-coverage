# What the re-plan buys: Mission A with and without re-planning

Same 6 seeds (same victim, same kill time). "Without": the ground station is killed at t = 40 s,
so nobody re-plans the loss and the survivors keep flying their old lanes. Gazebo, N = 3, 50 m.
Median [min, max] over 6 runs. Source: table.json, made by tools/ablation_table.py.

| Metric | With re-plan | Without re-plan |
|---|---|---|
| Coverage from L+60 s, mean (%) | 94.8 [92.4, 95.6] | 74.3 [52.0, 75.0] |
| Coverage from L+60 s, lowest (%) | 90.6 [82.5, 92.4] | 71.9 [52.0, 72.5] |
| Dip, lowest from the loss on (%) | 88.0 [71.9, 91.6] | 71.9 [52.0, 72.5] |
| Last 30 s (%) | 95.3 [94.9, 95.6] | 74.2 [52.0, 75.1] |
| Longest unseen after the loss (s) | 105.9 [96.5, 140.9] | 160.0 [134.2, 176.8] |
| Recovered to 0.9 x pre-loss | 6 of 6 | 0 of 6 |

| Seed | Victim | Kill t (s) | With: from L+60 mean (%) | Without (%) |
|---|---|---|---|---|
| 1 | 1 | 64.0 | 95.6 | 74.2 |
| 2 | 1 | 88.7 | 95.2 | 74.4 |
| 3 | 3 | 67.1 | 92.4 | 52.0 |
| 4 | 1 | 67.1 | 95.4 | 74.6 |
| 5 | 3 | 78.7 | 92.8 | 52.0 |
| 6 | 1 | 83.8 | 94.5 | 75.0 |

Without a re-plan the dead drone's lanes are never flown again: coverage settles near 74 % when
a one-lane drone dies and at 52 % when the two-lane drone dies, and nothing recovers. With the
re-plan the same losses settle near 95 %. The longest-unseen figure without re-plan is cut off by
the 180 s mission end, so the real gap is longer. One run (without, seed 5) ended at a real-time
factor of 0.998; its coverage is flat at 52 %, so this does not change the result.
