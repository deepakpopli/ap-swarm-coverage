"""MavVehicle: the Vehicle protocol (swarm/vehicle_api.py) over MAVLink/TCP to ONE ArduCopter.

Each agent talks only to its own autopilot (SITL instance i on tcp 5760 + 10*i).

Frames and units
  pos:  area ENU metres. x east, y north from config.lla_to_enu(lat, lon) of GLOBAL_POSITION_INT;
        z = GLOBAL_POSITION_INT.relative_alt (m above home; home altitude == config.ORIGIN_LLA alt).
  vel:  ENU m/s from GLOBAL_POSITION_INT vx (north), vy (east), vz (down).
  yaw:  rad, 0 = north, clockwise positive (ATTITUDE.yaw is already in this convention).

Speed limit (two layers; see results/spike/speed_cap.md for the measurements)
  1. ArduPilot: setpoints are POSITION-ONLY targets (SET_POSITION_TARGET_GLOBAL_INT, velocity
     and acceleration ignored). Guided mode then runs AC_PosControl::input_pos_NED_m(), whose
     trajectory shaper limits horizontal speed to WP_SPD (set to config.V_MAX below). Velocity
     setpoints bypass that limit, so we never send them.
  2. This code: send_setpoint() clamps the commanded horizontal speed to config.V_MAX.

How a velocity command becomes a position target ("rabbit")
  send_setpoint(vx, vy, z) moves a reference point (the rabbit) at (vx, vy) and sends it as the
  position target. If the rabbit moves at v the autopilot's shaped trajectory also moves at v
  (it chases the rabbit with a constant lag). When the command drops to zero the rabbit stops
  and the autopilot brakes onto it without overshoot. The rabbit is never allowed further ahead
  of the vehicle than the autopilot's steady-state lag for the commanded speed (+2 m), so it
  cannot run away when the vehicle is slower than the command (command above WP_SPD, a climb,
  the vehicle is blocked) and the vehicle never "catches up" faster than commanded.
  Altitude changes are fed in steps small enough not to slow the horizontal motion.
  Corner behaviour: the rabbit is ~14 m ahead at 10 m/s (WP_ACC 6, PSC_NE_JERK 12), so a sharp
  turn at full speed rounds the corner by up to that much; stop first for clean 90 deg turns.

Liveness
  link_ok is False once nothing has been received for LINK_TIMEOUT_S. poll() never blocks and
  never raises when the autopilot dies (EOF / reset are caught and the socket is closed).
"""
import errno
import math
import socket
import time

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from swarm import config
from swarm.vehicle_api import VehicleState

LINK_TIMEOUT_S = 1.0
TELEMETRY_HZ = 20.0         # requested rate for GLOBAL_POSITION_INT and ATTITUDE
GUIDED = 4                  # ArduCopter custom mode number
LEAD_MARGIN_M = 2.0         # m allowed beyond the shaper's steady-state lag (see rabbit_lead_max())
DZ_MIN_M = 5.0              # m, smallest altitude step put into one position target

# Parameters applied before takeoff. WP_SPD is the autopilot-side horizontal speed limit for
# position targets in GUIDED (m/s in this ArduPilot build; older builds use WPNAV_SPEED in cm/s).
SPEED_PARAMS = {
    "WP_SPD": config.V_MAX,     # m/s, horizontal speed limit of the position shaper
    "WP_ACC": 6.0,              # m/s^2, horizontal accel of the shaper (default 2.5); shorter stops
    "PSC_NE_JERK": 12.0,        # m/s^3, horizontal jerk of the shaper (default 5); shorter lag/stops
    "WP_SPD_UP": 4.0,           # m/s climb (default 2.5)
    "WP_SPD_DN": 4.0,           # m/s descent (default 1.5); equal to climb so altitude_target() works both ways
    "ATC_ANGLE_MAX": 45.0,      # deg lean limit (default 30 cannot reach 10 m/s cruise on SITL's quad)
    "GUID_OPTIONS": 0,          # plain AC_PosControl path for position targets (not WPNav)
}


def shaper_lag(v, accel, jerk):
    """Steady-state distance (m) between a target moving at v (m/s) and ArduCopter's shaped position.

    From shape_pos_vel_accel_xy() + sqrt_controller() + sqrt_controller_accel() in
    ~/ardupilot/libraries/AP_Math/control.cpp with k = jerk/accel: the shaper's velocity
    command for position error e is vc(e) plus a closing-rate bias -accel*v/(k*vc) (sqrt region)
    or -v (linear region). Steady state means that sum equals v; solve for e.
    The vehicle itself trails the shaped position by about another 1 m at 10 m/s (measured).
    """
    k = jerk / accel
    if v <= 0.0:
        return 0.0
    e_linear = 2.0 * v / k
    if e_linear <= accel / k ** 2:
        return e_linear
    vc = 0.5 * (v + math.sqrt(v * v + 4.0 * accel * v / k))
    return vc * vc / (2.0 * accel) + accel / (2.0 * k * k)

# SET_POSITION_TARGET_* type_mask bits
_IGNORE_POS = 0x007
_IGNORE_VEL = 0x038
_IGNORE_ACC = 0x1C0
_IGNORE_YAW = 0x400
_IGNORE_YAW_RATE = 0x800
MASK_POS_ONLY = _IGNORE_VEL | _IGNORE_ACC | _IGNORE_YAW_RATE          # position + yaw
MASK_VEL_ONLY = _IGNORE_POS | _IGNORE_ACC | _IGNORE_YAW_RATE          # velocity + yaw
MASK_POS_VEL = _IGNORE_ACC | _IGNORE_YAW_RATE                         # position + velocity + yaw


class _SocketWriter:
    """File-like adapter so pymavlink's MAVLink object can send on our non-blocking socket."""

    def __init__(self, owner):
        self.owner = owner

    def write(self, buf):
        self.owner._send_bytes(buf)


class MavVehicle:
    """One agent's view of its own ArduCopter over MAVLink TCP.

    agent_id: 1..N, equal to the autopilot's MAV_SYSID.
    port:     autopilot TCP port (default 5760 + 10*(agent_id-1), i.e. base_instance 0).
    params:   dict of parameters applied in arm_and_takeoff() (default SPEED_PARAMS).
    clamp_in_code: second-layer clamp of horizontal speed to config.V_MAX in send_setpoint().
                   Only the speed-cap test turns this off.
    """

    def __init__(self, agent_id, port=None, host="127.0.0.1", params=None, clamp_in_code=True):
        self.agent_id = agent_id
        self.host = host
        self.port = port if port is not None else 5760 + 10 * (agent_id - 1)
        self.params = dict(SPEED_PARAMS if params is None else params)
        self.clamp_in_code = clamp_in_code
        self.sock = None
        self.mav = None
        self.last_rx = 0.0           # wall time of the last message from the autopilot
        self.armed = False
        self.custom_mode = None
        self.pos = None              # (x, y, z) area ENU m
        self.vel = (0.0, 0.0, 0.0)   # ENU m/s
        self.yaw = 0.0               # rad, compass convention
        self.t_pos = 0.0             # wall time of the last GLOBAL_POSITION_INT
        self.param_values = {}       # name -> value echoed by the autopilot
        self.statustext = []         # recent STATUSTEXT strings (for debugging pre-arm failures)
        self.rabbit = None           # (x, y) current position target, area ENU m
        self.t_last_setpoint = None
        self.last_target = None      # (x, y, z) last position target sent

    # ---- connection ------------------------------------------------------------------
    def connect(self, timeout=60.0):
        """Open TCP to the autopilot, wait for its heartbeat, request telemetry rates. Blocks <= timeout."""
        deadline = time.time() + timeout
        while True:
            try:
                self.sock = socket.create_connection((self.host, self.port), timeout=2.0)
                break
            except OSError:
                if time.time() > deadline:
                    raise RuntimeError(f"agent {self.agent_id}: cannot connect to {self.host}:{self.port}")
                time.sleep(0.5)
        self.sock.setblocking(False)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.mav = mavlink.MAVLink(_SocketWriter(self), srcSystem=200 + self.agent_id,
                                   srcComponent=mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
        self.mav.robust_parsing = True
        while self.last_rx == 0.0 or self.custom_mode is None:
            self.poll()
            if time.time() > deadline:
                raise RuntimeError(f"agent {self.agent_id}: no heartbeat")
            time.sleep(0.05)
        self.request_rates()

    def request_rates(self, hz=TELEMETRY_HZ):
        """Ask the autopilot to stream GLOBAL_POSITION_INT and ATTITUDE at `hz`."""
        interval_us = int(1e6 / hz)
        for msg_id in (mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, mavlink.MAVLINK_MSG_ID_ATTITUDE):
            self._command(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, interval_us)

    def close(self):
        """Close the socket (idempotent)."""
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    # ---- receive -----------------------------------------------------------------------
    def poll(self):
        """Read and handle everything waiting on the socket. Never blocks, never raises on link loss."""
        while self.sock is not None:
            try:
                data = self.sock.recv(65536)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                self.close()          # reset / broken: the autopilot is gone
                return
            if not data:
                self.close()          # EOF: the autopilot process closed (or died)
                return
            try:
                msgs = self.mav.parse_buffer(data) or []
            except mavlink.MAVError:
                msgs = []
            for m in msgs:
                self._handle(m)

    def _handle(self, m):
        if m.get_srcSystem() != self.agent_id:
            return
        self.last_rx = time.time()
        t = m.get_type()
        if t == "GLOBAL_POSITION_INT":
            x, y = config.lla_to_enu(m.lat * 1e-7, m.lon * 1e-7)
            self.pos = (x, y, m.relative_alt * 1e-3)
            self.vel = (m.vy * 0.01, m.vx * 0.01, -m.vz * 0.01)
            self.t_pos = self.last_rx
        elif t == "ATTITUDE":
            self.yaw = m.yaw
        elif t == "HEARTBEAT" and m.get_srcComponent() == mavlink.MAV_COMP_ID_AUTOPILOT1:
            self.armed = bool(m.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.custom_mode = m.custom_mode
        elif t == "PARAM_VALUE":
            self.param_values[m.param_id.rstrip("\x00")] = m.param_value
        elif t == "STATUSTEXT":
            self.statustext = (self.statustext + [m.text])[-20:]

    # ---- state -------------------------------------------------------------------------
    def link_ok(self):
        """True if the autopilot has sent something within the last LINK_TIMEOUT_S."""
        return self.sock is not None and time.time() - self.last_rx <= LINK_TIMEOUT_S

    def state(self):
        """Latest VehicleState, or None before the first position fix."""
        if self.pos is None:
            return None
        return VehicleState(t=self.t_pos, pos=self.pos, vel=self.vel, yaw=self.yaw,
                            armed=self.armed, link_ok=self.link_ok())

    # ---- send helpers ------------------------------------------------------------------
    def _send_bytes(self, buf):
        if self.sock is None:
            return
        try:
            self.sock.send(buf)
        except (BlockingIOError, InterruptedError):
            pass                      # socket buffer full: drop, the next setpoint replaces it
        except OSError:
            self.close()

    def _command(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
        """COMMAND_LONG to our autopilot (fire and forget)."""
        self.mav.command_long_send(self.agent_id, 1, cmd, 0, p1, p2, p3, p4, p5, p6, p7)

    def _wait(self, cond, timeout, step=0.05):
        """Poll until cond() is true or timeout (s) passes; return cond()."""
        deadline = time.time() + timeout
        while not cond():
            if time.time() > deadline or self.sock is None:
                return cond()
            time.sleep(step)
            self.poll()
        return True

    # ---- parameters, arming, takeoff ---------------------------------------------------
    def set_param(self, name, value, timeout=3.0, retries=3):
        """Set one parameter and wait for the autopilot to echo it. Returns the echoed value or None."""
        for _ in range(retries):
            self.param_values.pop(name, None)
            self.mav.param_set_send(self.agent_id, 1, name.encode(), float(value),
                                    mavlink.MAV_PARAM_TYPE_REAL32)
            if self._wait(lambda: name in self.param_values, timeout):
                return self.param_values[name]
        return None

    def apply_params(self):
        """Apply self.params; raise if any is not echoed back with the requested value."""
        for name, value in self.params.items():
            got = self.set_param(name, value)
            if got is None or abs(got - value) > 1e-3:
                raise RuntimeError(f"agent {self.agent_id}: param {name}={value} not accepted (got {got})")

    def set_mode_guided(self):
        self._command(mavlink.MAV_CMD_DO_SET_MODE, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, GUIDED)

    def arm_and_takeoff(self, alt, timeout=180.0):
        """Apply params, GUIDED, arm (retry until pre-arm checks pass), take off to `alt` m.

        Blocks until z >= alt - 1 m. Raises RuntimeError on timeout or link loss.
        """
        deadline = time.time() + timeout
        self.apply_params()
        while not self.armed:
            if time.time() > deadline or self.sock is None:
                raise RuntimeError(f"agent {self.agent_id}: arm failed: {self.statustext[-3:]}")
            self.set_mode_guided()
            self._wait(lambda: self.custom_mode == GUIDED, 1.0)
            self._command(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            self._wait(lambda: self.armed, 2.0)
        self._command(mavlink.MAV_CMD_NAV_TAKEOFF, p7=alt)
        if not self._wait(lambda: self.pos is not None and self.pos[2] >= alt - 1.0,
                          max(1.0, deadline - time.time()), step=0.02):
            raise RuntimeError(f"agent {self.agent_id}: takeoff timeout (z={self.pos and self.pos[2]})")
        self.rabbit = None

    # ---- setpoints ---------------------------------------------------------------------
    def send_position_target(self, x, y, z, yaw=0.0):
        """Position-only GUIDED target: area ENU x, y (m), z (m above home), yaw (rad, compass).

        Sent as SET_POSITION_TARGET_GLOBAL_INT in MAV_FRAME_GLOBAL_RELATIVE_ALT_INT so it does not
        depend on where each vehicle's EKF origin is. ArduCopter limits the horizontal speed of
        the resulting trajectory to WP_SPD.
        """
        lat, lon = config.enu_to_lla(x, y)
        self.mav.set_position_target_global_int_send(
            0, self.agent_id, 1, mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MASK_POS_ONLY,
            int(round(lat * 1e7)), int(round(lon * 1e7)), z, 0, 0, 0, 0, 0, 0, yaw, 0)
        self.last_target = (x, y, z)

    def send_velocity_target(self, vx, vy, vz_up=0.0, yaw=0.0):
        """Velocity-only GUIDED target (ENU m/s). NOT speed-limited by WP_SPD; kept for tests only."""
        self.mav.set_position_target_local_ned_send(
            0, self.agent_id, 1, mavlink.MAV_FRAME_LOCAL_NED, MASK_VEL_ONLY,
            0, 0, 0, vy, vx, -vz_up, 0, 0, 0, yaw, 0)

    def send_pos_vel_target(self, x, y, z, vx, vy, yaw=0.0):
        """Position + velocity feed-forward target (ENU). The feed-forward part is NOT limited by
        WP_SPD (only the position correction is); kept for tests only."""
        lat, lon = config.enu_to_lla(x, y)
        self.mav.set_position_target_global_int_send(
            0, self.agent_id, 1, mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MASK_POS_VEL,
            int(round(lat * 1e7)), int(round(lon * 1e7)), z, vy, vx, 0, 0, 0, 0, yaw, 0)

    def send_setpoint(self, vx, vy, z):
        """Fly horizontal velocity (vx east, vy north; m/s, area ENU) at altitude z (m). Yaw north.

        Call at config.CONTROL_HZ. Internally advances the rabbit and sends it as a position-only
        target (see module docstring); the autopilot caps the speed at WP_SPD.
        """
        if self.pos is None:
            return
        now = time.time()
        speed = math.hypot(vx, vy)
        if self.clamp_in_code and speed > config.V_MAX:
            vx, vy = vx * config.V_MAX / speed, vy * config.V_MAX / speed
        if self.rabbit is None or self.t_last_setpoint is None:
            self.rabbit = (self.pos[0], self.pos[1])
            dt = 0.0
        else:
            dt = min(max(now - self.t_last_setpoint, 0.0), 0.5)   # a stalled caller must not jump the rabbit
        self.t_last_setpoint = now
        rx, ry = self.rabbit[0] + vx * dt, self.rabbit[1] + vy * dt
        # keep the rabbit no further ahead than the autopilot needs to fly at the commanded speed
        dx, dy = rx - self.pos[0], ry - self.pos[1]
        lead = math.hypot(dx, dy)
        lead_max = self.rabbit_lead_max(math.hypot(vx, vy))
        if lead > lead_max:
            rx = self.pos[0] + dx * lead_max / lead
            ry = self.pos[1] + dy * lead_max / lead
            lead = lead_max
        self.rabbit = (rx, ry)
        self.send_position_target(rx, ry, self.altitude_target(z, lead))

    def rabbit_lead_max(self, v_cmd):
        """Largest rabbit lead (m) for commanded speed v_cmd (m/s).

        lag(v) is what the autopilot needs to fly at v. Using max(command, current speed) means:
        - a vehicle that fell behind (e.g. slowed by a climb) catches up at about v_cmd, not faster;
        - when the command drops, the rabbit is never pulled back towards a vehicle that is
          still braking (its current speed keeps the limit large), so stops do not overshoot.
        """
        v_now = math.hypot(self.vel[0], self.vel[1])
        acc, jerk = self.params.get("WP_ACC", 2.5), self.params.get("PSC_NE_JERK", 5.0)
        return shaper_lag(max(v_cmd, v_now), acc, jerk) + LEAD_MARGIN_M

    def altitude_target(self, z_cmd, lead):
        """Altitude (m) to put into the position target for commanded altitude z_cmd.

        ArduCopter shapes position targets along the straight 3D line to the target and limits
        the speed along it by WP_SPD horizontally and WP_SPD_UP/DN vertically (kinematic_limit in
        AP_Math/control.cpp). A big altitude step relative to the horizontal lead would therefore
        slow the horizontal motion. Limiting the step to 0.8 * lead * min(WP_SPD_UP, WP_SPD_DN) / WP_SPD
        keeps the horizontal speed unaffected; DZ_MIN_M keeps climbs/descents brisk when hovering
        (at cruise lead ~15 m and 4 m/s vertical limits it still leaves 12 m/s horizontally).
        The vehicle still converges to z_cmd exactly.
        """
        z_now = self.pos[2]
        v_vert = min(self.params.get("WP_SPD_UP", 2.5), self.params.get("WP_SPD_DN", 1.5))
        ratio = v_vert / self.params.get("WP_SPD", config.V_MAX)
        dz_max = max(DZ_MIN_M, 0.8 * lead * ratio)
        return z_now + max(-dz_max, min(dz_max, z_cmd - z_now))

    def reset_rabbit(self):
        """Forget the rabbit; the next send_setpoint() starts it at the vehicle's position."""
        self.rabbit = None
