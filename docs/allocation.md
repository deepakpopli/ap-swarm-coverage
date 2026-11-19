# Allocation

Code: `swarm/allocator.py` (pure, deterministic), used by `swarm/ground.py`.

## Rules

1. **Shares.** The area is cut into equal full-height vertical strips, one per agent, handed out
   west to east by the agents' current x (ties by id). Neighbours stay neighbours, so paths do
   not cross.
2. **Lanes.** North-south, `ceil(strip width / swath)` of them, evenly spread over the strip.
   The swath is the camera footprint across track at the cruise altitude (85 m at 50 m). Lane
   ends are inset by half the along-track footprint minus `SLOT_REACH_M` (24 - 5 = 19 m at
   50 m): the agent counts a slot as reached up to 5 m before it and turns there, and the
   camera must still reach the area's edge. (With a 24 m inset the outermost 2-3 m of the
   area were never seen behind an out-and-back lane in a Gazebo run.)
3. **Slots.** Transit points run from the agent to the nearest lane end. Then comes the
   repeating loop (`loop_from` = its first index). Consecutive slots are at most
   `SLOT_SPACING_M` = 80 m apart: 7.7 s at cruise, about 10 s from a standing start, against the
   15 s slot timeout. A single-lane strip is flown out and back, so its loop has every point
   twice. Nothing relies on slots being unique.
4. **Stability.** An agent whose share is unchanged and whose previous plan has no
   blacklisted point keeps its previous assignment unchanged. Re-plans for one agent cause no
   churn for the others.

## Infeasible slots

The blacklist is a set of `(agent, slot_key)` pairs. `slot_key` is the slot rounded to whole
metres. The allocator never issues a blacklisted pair. For an agent whose plan would contain
one, it does this, in order:

1. Try the other loop variants (other entry end, other first-lane direction), nearest entry
   first, and take the first one that has no blacklisted point.
2. Otherwise, drop the blacklisted points from the nearest variant and re-space its legs. A
   leg whose new points would land on a blacklisted point is cut into one more piece. With
   n and n + 1 pieces, no inner point is shared.
3. If every point is blacklisted, the agent holds where it is until a pair expires. Only a
   short Mission B lane, with 3 distinct points, can get there.

**Why this cannot oscillate.**

* The ground blacklists a pair for `BLACKLIST_COOLDOWN_S` = 60 s and never re-issues it in
  that time.
* Each INFEASIBLE report causes at most one re-plan. A repeat for a pair already on the
  blacklist, or for a point no longer in the agent's plan, is logged and ignored. Expiry
  does not trigger a re-plan.
* Every other agent keeps its plan unchanged (rule 4).

So a report changes one agent's plan once, to a plan without that point, and nothing feeds
back.

## Mission B: priority zone

The zone (60 x 60 m) must be seen at least every `PRIORITY_REVISIT_S` = 30 s.

* The agent nearest the zone gets a strip one swath wide, centred on the zone in x. The strip
  is shifted to stay inside the area, and its lane is at the strip centre. That agent flies a
  single out-and-back lane whose ends are each at most `REACH_M` north or south of the zone
  centre.
* A point on an out-and-back lane is revisited after 2 x (distance to the far end) / speed +
  `TURN_S`. So `REACH_M = (30 - margin 2 - TURN_S 6) x 10.4 / 2 = 114.4 m`. `TURN_S` = 6 s:
  the largest measured time lost in a 180 degree turn, 4.6 s (SITL, 11.5 m/s; Gazebo 4.5 s,
  `results/spike/motion_bounds_*.json`), plus 1.4 s margin. In a Gazebo Mission B run a lane end
  cost about 2.5 s (revisit of the zone centre 24.3-24.6 s at 11 m/s against 22.0 s of
  straight flight), so the margin is large.
* The margin pays for points 30 m off the zone centre. The footprint's 24 m half-length leaves
  6 m, which is 0.5 s each way. About 1 s is left for flying below cruise speed near the ends.
* The other agents split the area west and east of the zone strip, in proportion to width.
  With 3 agents, one goes west and one goes east. If a side is narrower than half a swath, or
  nobody is left for it, it is added to the zone agent's share and stays uncovered.
* When an agent is lost, the zone keeps its agent first, and the rest is re-split.

The parts of the zone agent's share that its camera never sweeps are returned as
`Assignment.uncovered` and logged in the `assign` record. Priority beats completeness.

| Zone (xmin, ymin, xmax, ymax) | Lane ends (y) | Worst zone revisit | Uncovered |
|---|---|---|---|
| middle (120, 120, 180, 180) | 35.6 to 264.4 | 29.2 s | 16.6 m strips at the south and north edge: 2 822 m² (3.1 %) |
| corner (0, 0, 60, 60) | 19 to 144.4 | 29.2 s | x 0 to 85, y 163.4 to 300: 11 611 m² (12.9 %) |
| (20, 200, 80, 260) | 115.6 to 281 | 29.2 s | 7.5 m sliver on the west edge, plus x 7.5 to 92.5, y 0 to 96.6: 10 461 m² |

Revisit is computed from the plan: 2 x (distance from where the point leaves the footprint to
the far end) / 10.4 + 6 s, worst point of the zone. 50 m altitude, 3 agents. Uncovered parts
count from where the agent turns, 5 m before a lane end.

## Test hook: swap

`allocate(..., reverse=True)` hands the strips out east to west. Only the ground's `--swap-at`
uses it (scenario "swap" of `run_mission.py`): one re-plan that makes every agent cross the
others at cruise altitude, to stress the safety filter. Later re-plans use the normal order.

## Radio determinism (`sim/radio.py`)

**What the seed fixes.** Whether one copy is dropped is a hash of `(seed, src, dst, seq)`.
The decision does not depend on arrival order, timing, or other traffic. With the same seed,
the k-th message from A to B is dropped in every run or in none.

**What real time decides.**

* Which message gets seq k (STATE or INFEASIBLE).
* Which messages fall inside a blackout or mute window. Windows are judged by arrival time
  at the radio.
* Whether a receiver had registered when a broadcast arrived.
* The exact delivery times: the delay is a timer, with about 1 ms of jitter.

The per-pair loss rate matches `loss` statistically. The exact set of lost messages repeats
only up to that timing.

## Ground restart

A restarted ground hears agents that already report an epoch. It then:

1. Listens for 3 s.
2. Continues 1000 epochs above the highest epoch heard. An agent ignores epochs not newer
   than its own, so it can never see an epoch number twice with different contents.
3. Reuses t0 from the old `ground.jsonl`.
4. Plans for every agent it heard flying.
