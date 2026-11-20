# Allocation

Code: `swarm/allocator.py` (pure, deterministic), used by `swarm/ground.py`. The ground station
runs it; agents only fly the slots they are sent. It outputs assignments (share, slots,
loop_from), never velocities or times.

## Lanes and lane groups

1. **Lanes never move.** `ceil(area width / swath)` north-south lanes over the whole area,
   evenly spread. The swath is the camera footprint across track at the cruise altitude: 85 m
   at 50 m, so 4 lanes at x = 37.5, 112.5, 187.5, 262.5. Lane ends are inset by half the
   along-track footprint minus `SLOT_REACH_M` (24 - 5 = 19 m): the agent turns 5 m before a
   slot and the camera must still reach the area's edge.
2. **Groups.** The live agents get contiguous lane groups, one closed loop (boustrophedon plus
   the leg back) per agent. The cut is the one whose longest loop is shortest (loop time = length
   at 11.5 m/s + measured corner costs `TURN90_S` 2.45 s, `TURN180_S` 3.65 s); ties go to the
   first cut. At 50 m: N = 1: 4 lanes (149.9 s); N = 2: 2+2 (68.4 s); N = 3: 1+1+2 (52.9, 52.9,
   68.4 s); N = 4: 1+1+1+1 (52.9 s, each lane out and back).
3. **Owners.** Groups go to agents by the smallest total distance from each agent to its
   group's loop; ties: the first permutation of the ids in ascending order. From the start line
   agent 3 gets the east pair. An agent on its loop is nearest to its own group, so a re-plan
   for any other reason keeps the owners.
4. **More agents than lanes** (N = 5 at 50 m): one lane each; the agent left over (agent 3 from
   the start line) gets no slots and holds at its private hold altitude, out of the lanes' way,
   until a loss frees a lane. Splitting the loop work five ways was rejected: the pieces would be
   flown back and forth and two agents would meet head-on at every shared end. The ground counts
   a spare as on station.
5. **Slots.** Transit points from the agent to the nearest lane end of its loop, then the loop
   (`loop_from` = its first index). Consecutive slots are at most `SLOT_SPACING_M` = 80 m apart
   (79 m: 6.9 s at cruise, under 10 s from a standing start, against the 15 s slot timeout).
6. **Stability.** An agent whose share is unchanged and whose plan has no blacklisted point
   keeps its previous assignment object unchanged: no churn.
7. **On a loss** the cut for N - 1 is made on the same lanes (N = 3 -> 2: 2+2). Lanes never move,
   so a survivor flies ground it already knows; a survivor whose group is unchanged keeps its plan.
8. **Active slots apart.** No two agents' simultaneously active slots (transit points and the
   slot each starts on) may be closer than D_MIN + 2 x SLOT_REACH = 18 m: a swap run once issued
   two transit slots 0.2 m apart and stalled for 15 s. A new plan that clashes keeps its transit
   leg but its points move along it, off every other agent's active slots (`_clear_leg`). This is
   a planner courtesy, not a separation guarantee: transits can still cross between slots, and
   separation is the safety filter's job.

## Infeasible slots

The blacklist is a set of `(agent, slot_key)` pairs (`slot_key` = the slot rounded to whole
metres). The allocator never issues a blacklisted pair. For an agent whose plan would contain one:

1. Try the other loop variants (other entry end, other first-lane direction), nearest entry
   first, and take the first one without a blacklisted point.
2. Otherwise drop the blacklisted points from the nearest variant and re-space its legs. A leg
   whose new points would land on a blacklisted point is cut into one more piece (n and n + 1
   pieces share no inner point).
3. If every point is blacklisted (only a zone lane, 3 distinct points, can get there), the agent
   holds where it is until a pair expires.

**Why this cannot oscillate.** The ground blacklists a pair for `BLACKLIST_COOLDOWN_S` = 60 s
and never re-issues it in that time. Each INFEASIBLE report causes at most one re-plan; a repeat
for a pair already blacklisted, or for a point no longer in the agent's plan, is logged and
ignored; expiry does not re-plan. Every other agent keeps its plan (rule 6). So a report changes
one agent's plan once, to a plan without that point, and nothing feeds back.

## Mission B: priority zone

The zone (60 x 60 m) must be seen at least every `PRIORITY_REVISIT_S` = 30 s.

* **Owner.** One agent flies an out-and-back lane through the zone: the previous owner while it
  lives, else the agent nearest the zone centre. Every other agent covers the whole area in the
  normal lane groups, as if it were the whole fleet, so nothing is ever left unassigned.
* **Zone lane x.** Half way between two area lanes (or a lane and the area edge), so it keeps
  clear of the others' lanes; of those, the one whose footprint covers most of the zone. Middle
  zone (120-180): x = 150, 37.5 m from the lanes at 112.5 and 187.5. Corner zone (0-60):
  x = 18.75, 18.75 m from the lane at 37.5. Agents on neighbouring lanes still pass each other,
  so the safety filter acts more than in Mission A.
* **Lane ends** at most `REACH_M` north and south of the zone centre (clipped to the normal lane
  ends). A point is revisited after 2 x (distance to the far end) / speed + `TURN_S`, so
  `REACH_M = (30 - margin 2 - TURN_S 6) x 11.5 / 2 = 126.5 m`. `TURN_S` = 6 s bounds a full-speed
  reversal (5.9-6.0 s measured); the controller's slowed lane-end turn costs about 3.6 s. The
  margin pays for zone points up to 30 m off centre. Worst zone point from the plan: 29.0 s for
  both zones.
* **When the owner is lost**, the nearest survivor takes the zone lane first and the rest
  re-split the area. With 3 agents that leaves one agent for all four lanes (a 150 s loop).
  **Honest limit:** the 30 s cannot hold through the owner's loss: the zone waits for the
  12 s loss declaration plus the new owner's flight to the lane. With one agent left it flies
  the zone lane only.

## Test hook: swap

`allocate(..., reverse=True)` hands the groups out in reverse order. Only the ground's
`--swap-at` uses it (scenario "swap" of `run_mission.py`): one re-plan that makes the agents
cross at cruise altitude, to stress the safety filter. Later re-plans use the normal order.

## Radio determinism (`sim/radio.py`)

Whether one copy is dropped is a hash of `(seed, src, dst, seq)`, so with the same seed the
k-th message from A to B is dropped in every run or in none. Real time still decides which
message gets seq k, which messages fall inside a blackout or mute window, whether a receiver
had registered when a broadcast arrived, and the exact delivery times (about 1 ms jitter). The
per-pair loss rate matches `loss` statistically; the exact set of lost messages repeats only up
to that timing.

## Ground restart

A restarted ground that hears agents already reporting an epoch listens for 3 s, continues 1000
epochs above the highest epoch heard (an agent ignores epochs not newer than its own, so it never
sees one number twice with different contents), reuses t0 from the old `ground.jsonl`, and plans
for every agent it heard flying.

## Single point of failure

The allocator runs only on the ground. If the ground dies, agents keep flying their last
assignment (they never need the ground to keep flying), but nobody re-plans: a loss after that
leaves the dead agent's lanes unflown. Measured in scenario "groundloss" (`run_scenarios.sh`).
