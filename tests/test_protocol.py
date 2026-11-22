"""Wire format: every plan the allocator makes fits one SWARM_ASSIGN and survives it exactly."""
import itertools

from swarm import config, protocol
from swarm import swarm_mav as mav
from swarm.allocator import allocate, slot_key

ZONES = (None, (120.0, 120.0, 180.0, 180.0), (0.0, 240.0, 60.0, 300.0))
CORNERS = ((-50.0, -50.0), (350.0, 350.0), (-50.0, 350.0), (350.0, -50.0))


def wire(src, msg):
    packer = mav.MAVLink(None, srcSystem=src, srcComponent=protocol.COMPONENT)
    data = msg.pack(packer)
    return packer.decode(bytearray(data)), len(data)


def plans():
    """Plans of 1..5 agents (so also a single survivor flying every lane), from the start line
    and from far corners, with and without a Mission B zone, then re-planned 4 times, each time
    blacklisting one more slot per agent (4 active is the most SLOT_TIMEOUT_S and
    BLACKLIST_COOLDOWN_S allow)."""
    for n, zone, corner in itertools.product(range(1, 6), ZONES, (None,) + CORNERS):
        ids = list(range(1, n + 1))
        pos = {a: corner or config.start_xy(a - 1, n) for a in ids}
        blacklist, previous = set(), None
        for k in range(5):
            previous = allocate(ids, pos, blacklist, previous, zone)
            yield from previous.values()
            for a, asg in previous.items():
                if asg.slots:
                    bad = asg.slots[(asg.loop_from + 3 * k) % len(asg.slots)]
                    blacklist.add((a, slot_key(bad)))


def test_every_plan_fits_and_round_trips_exactly():
    largest = max(plans(), key=lambda asg: len(asg.slots))
    assert len(largest.slots) <= protocol.MAX_SLOTS - 10     # room for what no test found
    msg, size_b = wire(config.GROUND_ID, protocol.assign(3, 1234, **largest.to_dict()))
    assert size_b <= 12 + 255
    assert protocol.assign_slots(msg) == [tuple(p) for p in largest.slots]
    assert (msg.target_system, msg.epoch, msg.loop_from) == (3, 1234, largest.loop_from)
    for asg in plans():
        msg, _ = wire(config.GROUND_ID, protocol.assign(1, 1, **asg.to_dict()))
        assert protocol.assign_slots(msg) == [tuple(p) for p in asg.slots]


def test_full_assign_round_trips():
    slots = [(round(-3276.7 + 13.1 * i, 1), round(3276.7 - 7.3 * i, 1))
             for i in range(protocol.MAX_SLOTS)]
    msg, size_b = wire(config.GROUND_ID, protocol.assign(1, 7, [0, 300, 0, 300], slots, 5))
    assert protocol.assign_slots(msg) == slots and size_b <= 12 + 255


def test_state_start_infeasible_round_trip():
    msg, _ = wire(2, protocol.state(1.7e9 + 0.123456, (10.5, -3.25, 50.0), (11.0, 0.5, -0.1),
                                    -1, -1, "failsafe"))
    assert msg.get_srcSystem() == 2 and msg.get_srcComponent() == protocol.COMPONENT
    assert msg.time_usec == 1_700_000_000_123_456 and (msg.x, msg.y, msg.z) == (10.5, -3.25, 50.0)
    assert (msg.epoch, msg.slot, protocol.STATUSES[msg.status]) == (-1, -1, "failsafe")
    msg, _ = wire(config.GROUND_ID, protocol.start(1.7e9))
    assert msg.t0_usec / 1e6 == 1.7e9 and protocol.destination(msg) is None
    msg, _ = wire(2, protocol.infeasible(9, 4, (150.3, 19.0)))
    assert protocol.destination(msg) == config.GROUND_ID
    assert [msg.slot_x / 10, msg.slot_y / 10] == [150.3, 19.0]


def test_statuses_follow_the_dialect_enum():
    assert protocol.STATUSES == ["takeoff", "ready", "cover", "hold", "failsafe"]
