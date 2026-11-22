"""Message and log formats shared by the agents, the ground station, the radio and the analysis.

Every agent-agent and agent-ground message is one MAVLink 2 packet in one UDP datagram, from the
dialect swarm/swarm.xml (generated into swarm/swarm_mav.py, see setup.sh). It is sent to the
radio emulator, which drops or delays it and forwards it. Nothing else connects the nodes.
Header: source system = sender id (agents 1..N, the ground GROUND_ID), source component
MAV_COMP_ID_ONBOARD_COMPUTER, so it never looks like an autopilot.

    SWARM_STATE       agent -> all, at STATE_HZ: own position, velocity, epoch, slot, status
    SWARM_START       ground -> all, repeated: t0, wall time of mission t = 0
    SWARM_ASSIGN      ground -> target_system, repeated at ASSIGN_HZ (also to a lost agent, with
                      no slots: it holds; see Ground.send_due). epoch increases on every re-plan.
    SWARM_INFEASIBLE  agent -> the sender of its plan (the ground, or a backup), repeated until a
                      newer epoch arrives: the agent could not reach this slot within
                      SLOT_TIMEOUT_S.

With SWARM_BACKUP=1 an agent may send ASSIGN and START too (swarm/backup.py). An agent keeps
the ASSIGN with the highest epoch; equal epochs: the lower sender id, the ground counting as 0.
The allocator sends assignments only, never velocities.

Agent log (agent_<id>.csv), one row per control tick, the only input of the offline metrics:
"""
from swarm import config
from swarm import swarm_mav as mav

AGENT_CSV_COLUMNS = [
    "t_wall", "t_mission",            # s; empty until START, negative during the warm-up
    "x", "y", "z", "vx", "vy", "vz",  # own state from the own autopilot, area ENU frame
    "yaw",                            # rad, 0 = north, clockwise
    "status", "epoch", "slot",
    "nom_vx", "nom_vy",               # what the coverage controller asked for
    "cmd_vx", "cmd_vy", "cmd_z",      # what the safety filter let through
    "nbr_dist", "nbr_age",            # nearest neighbour as believed from its last message
    "flags",                          # "|"-joined: filtered, brake, deaf
]

COMPONENT = 191   # MAV_COMP_ID_ONBOARD_COMPUTER in common.xml, which the dialect leaves out
_ASSIGN = mav.MAVLink_swarm_assign_message
MAX_SLOTS = _ASSIGN.array_lengths[_ASSIGN.ordered_fieldnames.index("slot_x")]
# "takeoff", "ready", ... in SWARM_STATUS order: the logs keep the names
STATUSES = [e.name[len("SWARM_STATUS_"):].lower()
            for _, e in sorted(mav.enums["SWARM_STATUS"].items()) if not e.name.endswith("_END")]


def usec(t_s) -> int:
    return round(t_s * 1e6)


def dm(v_m) -> int:
    return round(v_m * 10)


def destination(msg):
    """The one receiver of a unicast message, or None for everyone but the sender."""
    return getattr(msg, "target_system", None)


def state(t_pos, pos, vel, epoch, slot, status):
    return mav.MAVLink_swarm_state_message(usec(t_pos), *pos, *vel, epoch, slot,
                                           STATUSES.index(status))


def start(t0):
    return mav.MAVLink_swarm_start_message(usec(t0))


def assign(agent_id, epoch, share, slots, loop_from):
    assert len(slots) <= MAX_SLOTS, f"{len(slots)} slots do not fit in SWARM_ASSIGN"
    pad = [0] * (MAX_SLOTS - len(slots))
    return _ASSIGN(agent_id, epoch, loop_from, len(slots), share or [0.0] * 4,
                   [dm(x) for x, _ in slots] + pad, [dm(y) for _, y in slots] + pad)


def assign_slots(msg) -> list:
    """SWARM_ASSIGN slots as [(x, y)] in metres, exactly the allocator's 0.1 m values."""
    return [(x / 10, y / 10) for x, y in zip(msg.slot_x[:msg.slot_count],
                                             msg.slot_y[:msg.slot_count])]


def infeasible(epoch, slot, slot_xy, target=config.GROUND_ID):
    """To whoever sent the plan: the ground, or an agent running the backup allocator."""
    return mav.MAVLink_swarm_infeasible_message(target, epoch, slot, dm(slot_xy[0]),
                                                dm(slot_xy[1]))
