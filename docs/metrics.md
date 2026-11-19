# Metric definitions

Code: `analysis/metrics.py` (one run), `analysis/report.py` (a set of runs). Every number comes from
the camera footprint (`docs/sensor.md`) and each agent's own logged position. Nothing comes from
what the agents told each other.

**Notation.** t is mission time in seconds. t = 0 one coverage window (60 s) after every agent
has reached a slot of its coverage loop.
L = loss time = meta `kill_wall - t0_wall`. R = the time the ground declared the agent lost.
W = 60 s. N = number of agents. A *sighting* is one agent log row (20 Hz) of a live agent: every
2 m cell whose centre lies inside that row's footprint counts as seen at that row's time.
The victim's rows stop counting at L.

**Warm-up.** Once every agent is on station, the ground starts the mission clock one window
later (t0 = now + 60 s), so the swarm observes for one full window first. Sightings count from
t = -W. coverage(0) therefore comes from a full window, and no "before" window holds a start-up
ramp. (This replaces an earlier rule that started from an empty map at t = 0.)

**Unseen time and the warm-up.** Sightings in the 60 s warm-up count, as they do for coverage,
so an unseen gap may start before t = 0 (at the latest t = -W: a cell never seen counts from
there). Only its end must lie in the phase.

| Metric | Definition |
|---|---|
| Worst revisit gap (longest unseen) | The task's metric: the longest time any cell went unseen. Per cell, the longest gap between consecutive sightings, over all cells. Three phases: whole [0, end], before [0, L), after [L, end]. A gap counts in the phase where it ends, so one that opens before L and closes after it counts in full in after and whole. A gap still open at the end counts as its age at the end (after and whole; before ignores the gap open at L). Also: % of cells whose longest unseen time exceeded 60 s and 90 s. |
| coverage(t) | % of cells seen in (t - W, t], evaluated every 1 s. Always absolute %, never "% of before". |
| Coverage, mean | Mean of coverage(t) over [0, end]. |
| Before (coverage) | [L - W, L): the last full window before the loss. |
| After (coverage) | [L + W, end]: from the moment the lost agent's last sightings have left the window. Survivor data only. Called "after the loss (from L + 60 s)", because it can contain ticks that have not recovered. |
| End of run | The last 30 s. |
| P | Mean of coverage(t) over Before. |
| Coverage after | Mean and min of coverage(t) over After. |
| (N-1)/N x P | The task's bar. After is also reported as a ratio to it. |
| Attainable with N-1 | Mean coverage over [0, end] of no-loss runs with N-1 agents (`--ref`). Omitted when no such runs are given, never estimated. |
| Steady revisit gap (both sightings inside the phase) | Formerly "worst completed revisit gap". Longest time between two consecutive sightings of the same cell, both inside the phase, over all cells. Phases: before = [0, L), after = [R, end] (survivors only). |
| Largest open gap | Largest (phase end - last sighting) over all cells. A cell never seen counts from t = 0. Shows whether the completed gap is cut short by the phase end. |
| No completed gap | % of cells with fewer than two sightings in the phase. |
| Recovery | First tick t >= L from which coverage(t) >= 0.9 P until the end, minus L. The recovery must hold for at least 10 s before the end. 0 if coverage never fell below 0.9 P; none = "not recovered within the run". Runs that do not recover are counted ("k of n recovered"), and the median is taken over the recovered runs only. |
| Recovery, weaker reading | The same against 0.9 x (N-1)/N x P. |
| Dip | Minimum of coverage(t) for t >= L, and when it happened. |
| Deficit | Sum over 1 s ticks t >= L of max(0, P - coverage(t)), in %.s. Needs no threshold. |
| Recovery steps | Seconds from L to each of: declared lost (R); new epoch in the log of every survivor; first survivor at slot index >= its loop_from under the new epoch; recovered. |
| Churn | Ground "assign" events: number of agents in "changed" that are still live, per live agent-minute. Assignments before t = 0 (the initial plan and any warm-up re-plan) are excluded. Counted over [0, L) and [L, end], or [0, end] without a loss. |
| Extra changes | Changes in [L, end] minus those of the first "lost" re-plan. Expected 0. |
| Epoch cross-check | Per agent: epoch changes in its own log vs ground re-plans while it was live. A mismatch is listed. Two re-plans within one lost resend would also cause one. |
| Separation | Minimum true 3D distance between live agents, with positions interpolated on a 20 Hz grid. Reported over the whole log (this includes the 10 m start line and the warm-up) and from t = 10 s. Also: seconds below 8 m, over the whole log. |
| Flag times | Per agent, seconds with "blocked", "brake" or "deaf" in [0, end]. A flagged row counts until the next row. |
| Infeasible | Number of ground "infeasible" events, accepted and ignored. |
| Control | From t_wall spacing of log rows (one row per setpoint): slowest agent's mean rate, worst p99 and max period, max ground speed. |
| Radio | The radio's totals. Loss rate = dropped_loss / (sent - dropped_window). |
| Mission B | Priority zone: the largest age (time since the last sighting) of any zone cell at any moment of [0, end]; the last sighting may lie in the warm-up, and a cell never seen counts from -W. The limit is 30 s. Also the % of 1 s ticks at which every zone cell was seen within the last 30 s. |
| Mission C | Was the blacked-out agent declared lost (expected no)? Re-plans from the blackout start (expected 0). Its deaf time. Time from the blackout end until its status is "cover" again. |
| Scenario runs | Only the generic churn, safety and infeasible blocks. |
