"""Shared constants. Every process imports this file. Units are in the names:
_M metres, _S seconds, _MS metres per second, _HZ hertz.

Per run, from the environment: SWARM_ALT=<metres> sets the cruise altitude (default 50),
SWARM_BACKUP=1 the onboard backup allocator (swarm/backup.py; off by default).
run_mission.py passes its environment to every child, so all processes agree.
"""
import math
import os

# ---- area and mission (local ENU metres: origin = south-west corner, x east, y north, z up)
AREA = (0.0, 300.0, 0.0, 300.0)                # xmin, xmax, ymin, ymax
ORIGIN_LLA = (-35.363262, 149.165237, 584.0)   # lat, lon, altitude AMSL of the area origin
START_SPACING_M = 10.0                         # agents start in a line on the south edge
MISSION_S = 180.0
COVERAGE_WINDOW_S = 60.0                       # "seen within the last 60 s"


def start_xy(index, n):
    """Start of agent number `index` (0..n-1): a line centred on the south edge."""
    x_mid = (AREA[0] + AREA[1]) / 2.0
    return (x_mid + START_SPACING_M * (index - (n - 1) / 2.0), AREA[2])


# ---- flight
V_MAX_MS = 12.0          # task limit
# Cruise speed and the autopilot's WP_SPD. The vehicle overshoots WP_SPD by a tracking error
# that grows with PSC_NE_JERK; the low jerk, not a lower cruise, keeps it under V_MAX_MS
# (measured 11.71 m/s worst).
WP_SPD_MS = 11.5
CONTROL_HZ = 20.0        # setpoint rate to the autopilot (task: 10 Hz or faster)
ALT_CRUISE_M = float(os.environ.get("SWARM_ALT", "50"))
LAYER_STEP_M = 10.0      # vertical spacing of hold altitudes: D_MIN_M + 2 x 1 m tracking error


def hold_alt(agent_id):
    """Private hold altitude: cruise +10, -10, +20, -20, ... m for ids 1, 2, 3, 4, ...: never
    cruise, never shared, so a holding agent is vertically clear of everyone holding or cruising."""
    step = (agent_id + 1) // 2
    sign = 1.0 if agent_id % 2 == 1 else -1.0
    return ALT_CRUISE_M + sign * step * LAYER_STEP_M


# ---- safety
D_MIN_M = 8.0            # required minimum separation between any two agents
DEAF_STOP_S = 3.0        # an agent that has heard nobody for this long stops and holds

# ---- radio and liveness
STATE_HZ = 5.0           # agent state broadcast rate
ASSIGN_HZ = 1.0          # the ground re-sends the current assignment (idempotent by epoch)
RADIO_LOSS = 0.20
RADIO_DELAY_S = 0.200
SUSPECT_S = 2.0          # silent this long: suspect
LOST_S = 12.0            # silent this long: lost. Longer than Mission C's 10 s blackout.

# ---- onboard backup allocator (SWARM_BACKUP=1; swarm/backup.py). Off = ground only.
BACKUP = os.environ.get("SWARM_BACKUP") == "1"
# The ground sends each agent 2 messages a second (its ASSIGN, START): 12 s silence is ~24 lost
# in a row at 20 % loss, and longer than a 10 s blackout, so a quiet ground is not a dead one.
GROUND_LOST_S = LOST_S

# ---- feasibility feedback
SLOT_TIMEOUT_S = 15.0    # a slot not reached this long after becoming active is infeasible
SLOT_REACH_M = 5.0       # a slot counts as reached inside this horizontal radius
SLOT_SPACING_M = 80.0    # the allocator never puts consecutive slots further apart
BLACKLIST_COOLDOWN_S = 60.0

# ---- Mission B
PRIORITY_REVISIT_S = 30.0

# ---- network ports (a per-run --port-base is added to each)
RADIO_PORT = 14000
GROUND_PORT = 14099
# MAVLink system ids: the ground 255 (0 is broadcast); agents 1..N, the same as their own
# autopilot's MAV_SYSID; SITL instance = id - 1
GROUND_ID = 255


def agent_port(agent_id):
    return 14100 + agent_id


def sitl_tcp_port(agent_id):
    """TCP port of the agent's own autopilot (ArduPilot SITL: 5760 + 10 x instance)."""
    return 5760 + 10 * (agent_id - 1)


# ---- frames (flat-earth conversion; fine over 300 m)
M_PER_DEG_LAT = 110540.0
M_PER_DEG_LON = 111320.0 * math.cos(math.radians(ORIGIN_LLA[0]))


def lla_to_enu(lat, lon):
    return ((lon - ORIGIN_LLA[1]) * M_PER_DEG_LON, (lat - ORIGIN_LLA[0]) * M_PER_DEG_LAT)


def enu_to_lla(x, y):
    return (ORIGIN_LLA[0] + y / M_PER_DEG_LAT, ORIGIN_LLA[1] + x / M_PER_DEG_LON)
