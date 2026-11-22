# Decisions

What I picked, why, and what it costs me. Newest at the bottom.

## Dead or just silent?

My first idea was a 1 Hz heartbeat with a 3 s timeout. Did the maths and it falls apart.

Loss is 20 % per message and each loss is independent, so k lost in a row is 0.2^k. At 1 Hz a
3 s timeout trips after 2 or 3 lost heartbeats, that is 4 % or 0.8 % per heartbeat. A 3 minute
mission has about 180 heartbeats per drone, so a perfectly healthy drone gets called dead
1 to 7 times per mission. And Mission C is a 10 s blackout, so anything under 10 s kills a
drone that is only quiet.

The 200 ms delay is not the problem. It shifts every message by the same amount, the gaps stay
the same.

So: state goes out at 5 Hz and doubles as the heartbeat, with two levels.

- suspect after 2 s (10 losses in a row, about 1 in 10 million): just a flag on the ground.
  The safety filter doesn't wait for it, it keeps further away from a neighbour the older
  its last message is, from the first missed one
- lost after 12 s (about 60 in a row, never happens by chance): the allocator gives its lanes
  away

The cost is that after a real death the others start on its lanes 12 s late. Over the radio a
dead drone and a quiet one look exactly the same, so waiting longer than the longest silence
is the only safe rule if the radio is all you have.

## A second way to tell: look at it

The radio can't tell dead from quiet, a camera could. The idea: when a neighbour goes suspect,
a drone in range turns its gimbal to where the neighbour should be (last position + velocity x
age) and checks the image with OpenCV. Not there: call it lost early instead of waiting 12 s.
There: it's only quiet, leave its lanes alone. It has to come from pixels, not from Gazebo's
true positions, otherwise it's just reading the other drone's state a different way.

I tried it on a few runs and it didn't work reliably. It "saw" its own propeller arm (the
camera sits inside the frame) and the horizon edge. A wrong "seen" is worse than nothing, it
keeps a dead drone's lanes unflown. So it's not in this submission.

## mavlink for everything

pymavlink, no ROS. With 20 % loss I want small messages sent often, not big ones sent rarely.
More messages per second means shorter silences, and a SiK telemetry radio only gives about
64 kbit/s.

| | JSON (first version) | mavlink 2 |
|---|---|---|
| one state message | 185 bytes | 51 bytes (our SWARM_STATE) |
| 3 drones at 5 Hz | about 22 kbit/s | about 6.1 kbit/s |

These are bytes on the wire: 10 header, the payload, 2 CRC, no signing. For scale, the
standard GLOBAL_POSITION_INT is 40 bytes on the wire, but it isn't enough on its own (below).

mavlink doesn't make the radio lose less. It makes each message cheap, so I can afford 5 Hz,
and every packet has a CRC so a corrupted one is dropped instead of misread.

GLOBAL_POSITION_INT on its own is not enough. No plan epoch, no active slot, no status, and
its clock is time since boot, which the other drones can't use. So state, assignments,
infeasible reports and start are a small dialect in `swarm/swarm.xml`, generated with mavgen.

## Why 3 drones and not 4 or 5

My first plan was 5: four drones on one lane each and one spare. I thought when one dies the
spare just takes its lane and nothing degrades, and the whole question of the task is whether
the swarm degrades gracefully. So I went with 3, the minimum allowed and the hardest case: 4
lanes split 1+1+2, and after a loss the two survivors have to cover the whole field between
them.

More drones is still the right answer for a real deployment.
