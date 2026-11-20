"""Safety filter tests: FakeVehicles flown in stepped time over a lossy, delayed fake radio."""
import itertools
import math

import pytest

from fakes import FakeRadio, SimClock
from sim.fake_vehicle import FakeVehicle
from swarm import config
from swarm.coverage import NoProgress
from swarm.safety import Neighbour, SafetyFilter, SafetyParams, clear_for_takeoff

CRUISE = config.ALT_CRUISE_M
TICK_S = 1.0 / config.CONTROL_HZ
PHYSICS_S = 0.01               # vehicles move and distances are checked every 10 ms
SEEDS = (0, 1, 2)


class Flyer:
    """One agent: vehicle, filter and link, flying straight through its waypoints."""

    def __init__(self, aid, start, waypoints, clock, radio, hold=False):
        self.id = aid
        self.vehicle = FakeVehicle(aid, start, clock=clock)
        self.filter = SafetyFilter(aid)
        self.filter.holding = hold
        self.hold = hold
        self.link = radio.link(aid)
        self.waypoints = list(waypoints)
        self.progress = NoProgress()   # as CoverageController does for its active slot
        self.log = []          # (t, pos, holding, flags) per tick

    def nominal(self):
        x, y, _ = self.vehicle.pos
        if len(self.waypoints) > 1 and math.dist((x, y), self.waypoints[0]) < 5.0:
            self.waypoints.pop(0)
            self.progress.reset()
        gx, gy = self.waypoints[0]
        d = math.hypot(gx - x, gy - y)
        if d < 0.5:
            return 0.0, 0.0
        speed = min(config.WP_SPD_MS, math.sqrt(2 * 1.5 * d))
        return speed * (gx - x) / d, speed * (gy - y) / d

    def tick(self, now, send_state):
        for m in self.link.recv_all():
            self.filter.heard(now)
            if m["type"] == "STATE":
                self.filter.update_neighbour(m["src"], m["pos"], m["vel"], m["t_pos"],
                                             m["status"])
        v = self.vehicle
        if send_state:
            self.link.send("all", "STATE", pos=list(v.pos), vel=list(v.vel), t_pos=now,
                           status="cover")
        far = math.dist(v.pos[:2], self.waypoints[0]) > config.SLOT_REACH_M
        stuck = far and self.progress.update(now, math.dist(v.pos[:2], self.waypoints[0]))
        cmd = self.filter.step(now, tuple(v.pos), tuple(v.vel), self.nominal(), self.hold, stuck)
        v.send_setpoint(cmd.vx, cmd.vy, cmd.z)
        self.log.append((now, tuple(v.pos), cmd.holding, cmd.flags))
        return cmd

    def at_goal(self):
        return math.dist(self.vehicle.pos[:2], self.waypoints[-1]) < config.SLOT_REACH_M


def fly(specs, seconds, seed, blackouts=()):
    """specs: (id, start_xyz, waypoints[, hold]). Returns (flyers, min 3D distance).

    20 % loss, 200 ms delay, a ground node heard by all once a second. Stops early once
    every agent has been at its last waypoint for 2 s."""
    clock = SimClock()
    radio = FakeRadio(clock, loss=config.RADIO_LOSS, delay_s=config.RADIO_DELAY_S, seed=seed)
    radio.blackouts = [(n, clock() + a, clock() + b) for n, a, b in blackouts]
    ground = radio.link(config.GROUND_ID)
    flyers = [Flyer(*s[:3], clock, radio, *s[3:]) for s in specs]
    d_min = math.inf
    at_goal_s = 0.0
    for k in range(int(seconds / TICK_S)):
        if all(f.at_goal() for f in flyers):
            at_goal_s += TICK_S
        else:
            at_goal_s = 0.0
        if at_goal_s > 2.0:
            break
        for _ in range(round(TICK_S / PHYSICS_S)):
            clock.advance(PHYSICS_S)
            for f in flyers:
                f.vehicle.poll()
            for a, b in itertools.combinations(flyers, 2):
                d_min = min(d_min, math.dist(a.vehicle.pos, b.vehicle.pos))
        if k % round(config.CONTROL_HZ) == 0:
            ground.send("all", "START", t0=0.0)
        for f in flyers:
            f.tick(clock(), k % round(config.CONTROL_HZ / config.STATE_HZ) == 0)
    return flyers, d_min


# ---- the model the tests fly on ------------------------------------------------------------

def test_fake_vehicle_matches_measured_autopilot():
    """Speed-up, cap, stop and climb numbers from results/spike/speed_cap.md."""
    v = FakeVehicle(1, (0.0, 0.0, CRUISE), clock=lambda: 0.0)
    t_95 = None
    for k in range(400):                                  # 20 s at 20 Hz, asking for 18 m/s
        v.send_setpoint(18.0, 0.0, CRUISE)
        v.step(TICK_S)
        if t_95 is None and v.vel[0] >= 0.95 * 10.0:
            t_95 = (k + 1) * TICK_S
    assert 3.0 <= t_95 <= 4.2                              # measured 3.5-4.2 s to 95 % of 10
    assert config.WP_SPD_MS - 0.5 <= v.vel[0] <= config.WP_SPD_MS
    while v.vel[0] > 10.0:                                 # slow to 10 m/s, then stop
        v.send_setpoint(10.0, 0.0, CRUISE)
        v.step(TICK_S)
    x0, t_stop = v.pos[0], 0.0
    while math.hypot(v.vel[0], v.vel[1]) >= 0.3:
        v.send_setpoint(0.0, 0.0, CRUISE)
        v.step(TICK_S)
        t_stop += TICK_S
    assert 13.0 <= v.pos[0] - x0 <= 16.0                   # measured 13.9-15.0 m
    assert 2.2 <= t_stop <= 3.0                            # measured 2.4-2.9 s
    t_climb = 0.0
    while v.pos[2] < CRUISE + 9.0:
        v.send_setpoint(0.0, 0.0, CRUISE + 10.0)
        v.step(TICK_S)
        t_climb += TICK_S
        assert v.vel[2] <= 2.92                            # measured max 2.92 m/s
    assert 4.5 <= t_climb <= 6.0                           # measured 5.1 s


# ---- formulas ------------------------------------------------------------------------------

def test_keep_out_radius_numbers_in_the_doc():
    p = SafetyParams()
    assert p.t_go_s == pytest.approx(4.6)
    assert [round(p.radius_m(a), 2) for a in (0.2, 1.0, 3.0, 10.0)] == [9.22, 14.5, 42.09, 88.49]


def test_hold_and_cruise_agents_are_vertically_separated():
    """A holder at its hold altitude ignores a cruising neighbour with a hold on the other side."""
    f = SafetyFilter(1)                                    # hold altitude cruise + 10
    f.update_neighbour(2, (0.0, 0.0, CRUISE), (0.0, 0.0, 0.0), 0.0, "cover")   # hold cruise - 10
    f.heard(0.0)
    cmd = f.step(0.0, (0.0, 0.0, config.hold_alt(1)), (0.0, 0.0, 0.0), (5.0, 0.0), True)
    assert (cmd.vx, cmd.flags) == (5.0, [])


def test_failsafe_neighbour_is_never_vertically_separated():
    f = SafetyFilter(1)
    f.update_neighbour(2, (15.0, 0.0, CRUISE), (0.0, 0.0, 0.0), 0.0, "failsafe")
    f.heard(0.0)
    cmd = f.step(0.0, (0.0, 0.0, config.hold_alt(1)), (0.0, 0.0, 0.0), (5.0, 0.0), True)
    assert cmd.vx < 5.0 and "filtered" in cmd.flags


def test_brakes_on_actual_speed_not_command():
    """Coverage asks for nothing, but the vehicle still closes fast on a disc 20 m ahead."""
    f = SafetyFilter(1)
    f.holding = False
    f.update_neighbour(2, (30.0, 0.0, CRUISE), (0.0, 0.0, 0.0), 0.0, "cover")
    f.heard(0.0)
    cmd = f.step(0.1, (0.0, 0.0, CRUISE), (11.5, 0.0, 0.0), (0.0, 0.0), False)
    assert (cmd.vx, cmd.vy, "brake" in cmd.flags) == (0.0, 0.0, True)


def test_clear_for_takeoff():
    nbrs = {2: Neighbour((10.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0, False)}
    assert clear_for_takeoff((0.0, 0.0, 0.0), nbrs)
    nbrs[3] = Neighbour((3.0, 4.0, 60.0), (0.0, 0.0, 0.0), 0.0, False)   # hovering above us
    assert not clear_for_takeoff((0.0, 0.0, 0.0), nbrs)


def test_silent_neighbour_is_dropped_after_lost_s():
    f = SafetyFilter(1)
    f.update_neighbour(2, (50.0, 0.0, CRUISE), (0.0, 0.0, 0.0), 0.0, "cover")
    f.heard(0.0)
    f.step(config.LOST_S - 0.1, (0.0, 0.0, CRUISE), (0.0, 0.0, 0.0), (0.0, 0.0), False)
    assert 2 in f.nbrs
    cmd = f.step(config.LOST_S + 0.1, (0.0, 0.0, CRUISE), (0.0, 0.0, 0.0), (0.0, 0.0), False)
    assert 2 not in f.nbrs and cmd.events[0] == {"event": "neighbour_lost", "nbr": 2}


# ---- flown scenarios -----------------------------------------------------------------------

@pytest.mark.parametrize("angle_deg", [30, 60, 90, 120, 180])
@pytest.mark.parametrize("seed", SEEDS)
def test_crossing_at_cruise_speed(angle_deg, seed):
    """Two agents cross at angle_deg (180 = head-on): both keep >= 8 m and both get through."""
    a = math.radians(angle_deg)
    far = 150.0
    flyers, d_min = fly([(1, (-far, 0.0, CRUISE), [(far, 0.0)]),
                         (2, (-far * math.cos(a), -far * math.sin(a), CRUISE),
                          [(far * math.cos(a), far * math.sin(a))])], 60.0, seed)
    assert d_min >= config.D_MIN_M
    assert all(f.at_goal() for f in flyers)


@pytest.mark.parametrize("n", [3, 5])
@pytest.mark.parametrize("seed", SEEDS)
def test_agents_converging_on_one_point(n, seed):
    """n agents on a 120 m circle all fly through its centre to the opposite side."""
    specs = []
    for i in range(n):
        c, s = math.cos(2 * math.pi * i / n), math.sin(2 * math.pi * i / n)
        specs.append((i + 1, (120 * c, 120 * s, CRUISE), [(-120 * c, -120 * s)]))
    flyers, d_min = fly(specs, 90.0, seed)
    assert d_min >= config.D_MIN_M
    assert all(f.at_goal() for f in flyers)


@pytest.mark.parametrize("seed", SEEDS)
def test_blackout_while_a_neighbour_approaches(seed):
    """Agent 1 is cut off (both ways) for 10 s as agent 2 crosses its path at 90 degrees."""
    flyers, d_min = fly([(1, (-150.0, 0.0, CRUISE), [(150.0, 0.0)]),
                         (2, (0.0, -150.0, CRUISE), [(0.0, 150.0)])],
                        60.0, seed, blackouts=[(1, 8.0, 18.0)])
    assert d_min >= config.D_MIN_M
    assert any("deaf" in flags for _, _, _, flags in flyers[0].log)
    assert all(f.at_goal() for f in flyers)


@pytest.mark.parametrize("seed", SEEDS)
def test_cruiser_passes_under_a_forgotten_deaf_agent(seed):
    """Agent 1 hovers deaf at its hold altitude; once forgotten, agent 2 flies straight under."""
    flyers, d_min = fly([(1, (0.0, 0.0, config.hold_alt(1)), [(0.0, 0.0)], True),
                         (2, (-300.0, 0.0, CRUISE), [(200.0, 0.0)])],
                        60.0, seed, blackouts=[(1, 2.0, 1000.0)])
    holder, cruiser = flyers
    closest_xy = min(math.dist(a[1][:2], b[1][:2]) for a, b in zip(holder.log, cruiser.log))
    assert d_min >= config.D_MIN_M
    assert closest_xy < 2.0                                # it really flew through x, y
    assert cruiser.at_goal()


@pytest.mark.parametrize("seed", SEEDS)
def test_blocked_agent_passes_at_hold_altitude_and_returns(seed):
    """Agent 1 must pass between two hovering agents 18 m apart: blocked, over them, back down."""
    flyers, d_min = fly([(1, (-150.0, 0.0, CRUISE), [(0.0, 0.0), (150.0, 0.0)]),
                         (2, (0.0, 9.0, CRUISE), [(0.0, 9.0)]),
                         (4, (0.0, -9.0, CRUISE), [(0.0, -9.0)])], 100.0, seed)
    log = flyers[0].log
    assert d_min >= config.D_MIN_M
    assert any("blocked" in flags for _, _, _, flags in log)
    assert max(pos[2] for _, pos, _, _ in log) > config.hold_alt(1) - 1.0
    assert flyers[0].at_goal()
    assert not log[-1][2] and abs(log[-1][1][2] - CRUISE) < 1.0   # back at cruise


def test_stuck_sends_a_cruiser_to_its_hold_altitude_once():
    f = SafetyFilter(1)
    f.holding = False
    f.heard(0.0)
    cmd = f.step(0.0, (0.0, 0.0, CRUISE), (0.0, 0.0, 0.0), (5.0, 0.0), False, True)
    assert cmd.holding and cmd.z == config.hold_alt(1) and "blocked" in cmd.flags
    assert cmd.events == [{"event": "blocked"}]
    cmd = f.step(0.05, (0.0, 0.0, CRUISE), (0.0, 0.0, 0.0), (5.0, 0.0), False, True)
    assert cmd.events == [] and "blocked" in cmd.flags
