"""Message and log formats shared by the agents, the ground station and the analysis.

Every agent-agent and agent-ground message is one UDP datagram with one JSON object. It is sent
to the radio emulator, which drops or delays it and forwards it. Nothing else connects the nodes.

Envelope, added by the sender:
    {"src": int, "dst": int | "all", "seq": int, "t": wall time at send, "type": str, ...body}
    src and dst: 0 is the ground station, 1..N are agents, "all" is everyone but the sender.

Messages
    HELLO       node -> radio only: {"port": int}. Registers the node's listen port.
    STATE       agent -> all, at STATE_HZ:
                {"pos": [x, y, z], "vel": [vx, vy, vz],
                 "t_pos": wall time the position was sampled,
                 "epoch": int, "slot": int, "status": str}
                status: "takeoff", "ready", "cover", "hold" (deaf or blocked), "failsafe"
                (the autopilot left GUIDED mode; the agent no longer controls the vehicle).
    START       ground -> all, repeated: {"t0": wall time of mission t = 0}
    ASSIGN      ground -> one agent, repeated at ASSIGN_HZ:
                {"epoch": int,            # increases on every re-plan
                 "share": [xmin, xmax, ymin, ymax],
                 "slots": [[x, y], ...],  # ordered goal slots
                 "loop_from": int}        # after the last slot, continue from this index
    INFEASIBLE  agent -> ground, repeated until a newer epoch arrives:
                {"epoch": int, "slot": int, "slot_xy": [x, y]}
                The agent could not reach this slot within SLOT_TIMEOUT_S.

The allocator sends assignments only, never velocities.

Agent log (agent_<id>.csv), one row per control tick, the only input of the offline metrics:
"""
import json

AGENT_CSV_COLUMNS = [
    "t_wall", "t_mission",            # s; t_mission is empty before the mission starts
    "x", "y", "z", "vx", "vy", "vz",  # own state from the own autopilot, area ENU frame
    "yaw",                            # rad, 0 = north, clockwise
    "status", "epoch", "slot",
    "nom_vx", "nom_vy",               # what the coverage controller asked for
    "cmd_vx", "cmd_vy", "cmd_z",      # what the safety filter let through
    "nbr_dist", "nbr_age",            # nearest neighbour as believed from its last message
    "flags",                          # "|"-joined: filtered, brake, blocked, deaf
]


def encode(src, dst, seq, t, mtype, **body):
    msg = {"src": src, "dst": dst, "seq": seq, "t": t, "type": mtype}
    msg.update(body)
    return json.dumps(msg, separators=(",", ":")).encode()


def decode(data):
    return json.loads(data.decode())
