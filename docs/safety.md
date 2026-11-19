# Safety filter (swarm/safety.py)

## Invariant

For every pair of agents the 3D distance stays >= D_MIN = 8 m. Each agent enforces it alone,
using only the last STATE it received from each neighbour. Per neighbour, one of two holds:

- **vertical**: the altitude intervals the two agents can be in are >= 8 m apart, or
- **horizontal**: we stay outside the neighbour's keep-out disc and close on it no faster than
  we can still stop before its edge.

Both agents of a pair apply the same rules, so each takes half of the closing speed.

## Altitudes and vertical separation

Every agent only ever targets two altitudes: cruise (ALT_CRUISE_M, 50 m by default) or its own
hold altitude `config.hold_alt(id)` = cruise +10, -10, +20, -20, ... m for ids 1, 2, 3, 4, ...
No hold altitude equals cruise and no two agents share one, for any N.

Altitude intervals (tracking error +-1 m: measured 0.08 m steady, about 1 m as take-off ends):

- own: within 1 m of the altitude we now target, or between that and where we are;
- neighbour: within 1 m of cruise or of its hold altitude, or between those and its last
  reported z (this covers the take-off climb); unbounded if its status is "failsafe";
- two agents are vertically separated when their intervals are >= 8 m apart.

So a holding agent is separated from neighbours whose hold altitude is on the other side of
cruise or closer to cruise (10 - 2 x 1 = 8 m). A cruising agent is never separated from anyone:
it always treats every neighbour horizontally. Physically, a hovering agent at its hold altitude
is never in the cruise plane and never at another agent's hold altitude.

## Keep-out disc

Neighbour j sampled its position at `t_pos` (STATE field). age = now - t_pos; it includes the
radio delay. Predicted centre = last position + last velocity x min(age, T_GO).

How far j can be from that centre:

- Up to T_GO, j may change its velocity by at most min(A_MAX t, DV_MAX). A_MAX = 8 m/s^2
  (measured peak 7.8), DV_MAX = 12.5 m/s (speed cap 11.5 + measured overshoot 0.35: a full stop).
  Drift = 0.5 A_MAX t^2 until t = DV_MAX / A_MAX = 1.56 s, then + DV_MAX per second.
- T_GO = 4.6 s is the latest age at which a silent j has started to brake:
  T_GAP 1.0 s (its radio failed at most 1 s after its last sample that reached us; five STATEs
  lost in a row at 20 % loss has p = 3e-4) + RADIO_DELAY 0.2 s (packets already in flight still
  reach it) + DEAF_STOP 3.0 s (then it heard nobody for 3 s) + T_REACT 0.4 s.
- After T_GO the centre is frozen and j can still slide its braking distance:
  V_MAX^2 / (2 A_BRAKE) = 144 / 6 = 24 m, at <= 12 m/s.

R(age) = D_MIN + MARGIN (1 m, position error of both agents) + drift. From the code:

| age (s) | 0.2  | 1     | 3     | 10 (and every age >= 6.6) |
|---------|------|-------|-------|---------------------------|
| R (m)   | 9.16 | 13.00 | 36.73 | 80.73                     |

The velocity limit plans against R(age + 0.6 s): the disc as it will be if the next three STATEs
are lost (fresh: 11.56 m), so a lost message does not put us inside the disc we just stopped at.

## Closing-speed limit and brake

v_stop(gap) = largest v with STOP_OFFSET + v T_REACT + v^2 / (2 A_BRAKE) <= gap, with
STOP_OFFSET 2 m (measured overshoot <= 1.23 m), T_REACT 0.4 s (measured autopilot reaction
0.2-0.3 s + one 50 ms tick), A_BRAKE 3 m/s^2 (measured 4.2 on average: 14-15 m from 10 m/s).
v_stop(5 / 10 / 20 / 30 m) = 3.21 / 5.83 / 9.26 / 11.82 m/s. At 10 m/s the model needs 22.7 m;
the measured stop takes 13.9-15.0 m.

For every neighbour we are not vertically separated from, in increasing id order:

1. The commanded closing speed toward the disc centre is cut to v_stop(planning gap) / 2; the
   tangential part is kept. A final uniform scale-down makes all limits hold at once.
2. **Brake**: if the vehicle's *actual* closing speed exceeds v_stop(gap to R) / 2, the command
   is zero horizontal velocity. The autopilot link re-aims its position target from the current
   position every tick, so zero means "brake as hard as you can" (the old link kept flying to a
   point ahead on the old heading for ~1.5 s, which let two crossing drones come within 6.2 m).

## Tie-break

Head-on (less than 1 m/s of tangential speed left after the cut), the removed speed is added to
the agent's right of the line to the neighbour. Both agents of a head-on pair step right and pass
left side to left side instead of mirroring each other.

## Hold, blocked, deaf

- **Hold**: z target = own hold altitude; the horizontal command still goes through the filter,
  so the agent keeps flying its path up there. Return to cruise when every neighbour that is not
  separated from the path down (or up) to cruise has been >= 5 m outside its disc for 1 s.
  Going from cruise to hold never needs a check: a cruising agent already treats everyone.
- **Blocked**: coverage asks for >= 1 m/s but for 3 s the filter lets through < 0.5 m/s in that
  direction (stopped, braking, or sliding sideways). The agent goes to hold and flies on there.
- **Deaf**: no message from anyone for DEAF_STOP_S = 3 s: zero horizontal velocity, go to the hold
  altitude. When messages return the agent is in hold and uses the normal return rule.
- **Before take-off**: `clear_for_takeoff` requires every known neighbour >= 9 m away
  horizontally (after listening 1 s). Take-off climbs straight up to the hold altitude.

## When the constraints cannot be met

Zero horizontal velocity always meets the closing-speed limits. If we are already inside a disc
(a neighbour flew at us), we brake and may only move away from it; the neighbour's own filter
stops it. The filter never reports infeasibility itself: if it keeps us from a slot, coverage's
15 s slot timer reports it (one feedback path).

## Residual risks

- **One-way radio failure**: a neighbour that still hears others but whose transmitter is dead
  never goes deaf and keeps flying; its disc grows to 80.73 m, then it is forgotten after
  LOST_S = 12 s and becomes invisible.
- **Forgotten neighbours**: after LOST_S nothing is known. A silent agent hovering at its hold
  altitude is safe from cruisers and holders, but an agent returning from a higher hold altitude
  to cruise passes through the hold altitudes in between (e.g. id 3: 70 -> 50 passes 60).
- **A dead drone falls** through every lower hold altitude and through the cruise plane.
- **Motion beyond the measured bounds**: DV_MAX covers a full stop, not a reversal (up to 24 m/s
  change). The coverage controller slows before sharp turns, but a filter sidestep can turn the
  velocity by up to 90 degrees, and a failsafe autopilot mode (RTL, LAND) may do anything.
- The A_BRAKE, T_REACT and A_MAX values come from one airframe model in SITL and Gazebo.
