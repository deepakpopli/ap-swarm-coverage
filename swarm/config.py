"""Shared constants. Every process imports these; nothing here is tuned per run, except the
nominal altitude: SWARM_ALT=<metres> in the environment (default 60) moves ALT_NOMINAL and the
+-10 m ALT_BAND with it. run_mission.py passes its environment to every child, so all processes
of a run agree."""
import math
import os

# ---- area / mission (local ENU metres, origin = SW corner of the area, x east, y north)
AREA = (0.0, 300.0, 0.0, 300.0)            # xmin, xmax, ymin, ymax
ORIGIN_LLA = (-35.363262, 149.165237, 584.0)  # lat, lon, AMSL of the area origin
START_Y = 0.0                              # agents start in a line on the south edge
START_SPACING = 10.0
MISSION_S = 180.0

def start_xy(i, n):
    """Start position of agent index i (0..n-1): a line 10 m apart, centred on the south edge."""
    return (150.0 + START_SPACING * (i - (n - 1) / 2.0), START_Y)

# ---- flight
V_MAX = 12.0            # hard limit, m/s (also set on the autopilot)
V_CRUISE = 10.0         # nominal coverage speed
ALT_BAND_HALF = 10.0


def _alt_nominal(env=os.environ):
    v = float(env.get("SWARM_ALT", "60"))
    if not ALT_BAND_HALF < v <= 110.0:      # above ~112 m the camera misses the GSD requirement
        raise ValueError(f"SWARM_ALT={v}: must be in ({ALT_BAND_HALF:g}, 110]")
    return v


ALT_NOMINAL = _alt_nominal()   # m above the area origin (default 60)
# the "fixed altitude band"; the safety fallback separates inside it by ID (default (50, 70))
ALT_BAND = (ALT_NOMINAL - ALT_BAND_HALF, ALT_NOMINAL + ALT_BAND_HALF)
CONTROL_HZ = 10.0       # setpoint rate to the autopilot (requirement: >= 10 Hz)

# ---- safety
D_MIN = 8.0             # required minimum separation, m
DEAF_STOP_S = 3.0       # an agent that has heard nobody for this long stops horizontally

# ---- comms
STATE_HZ = 5.0          # agent state broadcast rate
ASSIGN_HZ = 1.0         # ground re-sends the current assignment (loss tolerant, idempotent by epoch)
RADIO_LOSS = 0.20
RADIO_DELAY_S = 0.200
SUSPECT_S = 2.0         # silent this long -> suspect
LOST_S = 12.0           # silent this long -> lost (longer than Mission C's 10 s blackout)

# ---- feasibility
SLOT_TIMEOUT_S = 15.0
SLOT_REACH_M = 5.0      # a slot counts as reached inside this horizontal radius
BLACKLIST_COOLDOWN_S = 60.0

# ---- ports (all + PORT_BASE offset so experiments never clash)
RADIO_PORT = 14000
GROUND_PORT = 14099
def agent_port(agent_id):
    return 14100 + agent_id
GROUND_ID = 0           # agent ids are 1..N (same as MAV_SYSID); SITL instance = id - 1

# ---- frames
def lla_to_enu(lat, lon):
    la0, lo0, _ = ORIGIN_LLA
    return ((lon - lo0) * 111320.0 * math.cos(math.radians(la0)), (lat - la0) * 110540.0)

def enu_to_lla(x, y):
    la0, lo0, _ = ORIGIN_LLA
    return (la0 + y / 110540.0, lo0 + x / (111320.0 * math.cos(math.radians(la0))))
