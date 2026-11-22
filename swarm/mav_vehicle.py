"""MavVehicle: the Vehicle protocol (swarm/vehicle_api.py) over MAVLink TCP to one ArduCopter.

Frames: pos is area ENU metres from config.lla_to_enu(GLOBAL_POSITION_INT lat, lon); z is
relative_alt, metres above home (home altitude = the area origin's). vel is ENU m/s.
yaw is rad, 0 = north, clockwise.

send_setpoint(vx, vy, z) sends ONE position-only target per call,
    target_xy = current_xy + (vx, vy) * lead_s(speed),
because WP_SPD caps position targets but not velocity targets (measured in SITL and Gazebo), and
re-aiming from the current position makes a filter cut act at once (zero velocity = brake).
The altitude step per target is capped at ALT_STEP_M: the autopilot caps speed along the 3D line,
so a large step would slow the horizontal flight.

Failsafe: poll() sends a HEARTBEAT every second. MAV_GCS_SYSID makes this link the autopilot's
GCS, so if this program stops, the autopilot LANDS where it is after FS_GCS_TIMEOUT.
"""
import math
import socket
import time
from typing import Callable, Dict, Optional

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from swarm import config
from swarm.vehicle_api import VehicleState, lane_yaw

LAG_S = 0.15             # s, vehicle behind the autopilot's shaped position (see lead_s)
ALT_STEP_M = 5.0         # m, largest altitude change put into one target
LINK_TIMEOUT_S = 1.0     # silence after which link_ok turns False
HEARTBEAT_S = 1.0
TELEMETRY_HZ = 20.0      # rate of GLOBAL_POSITION_INT
GUIDED = 4               # ArduCopter custom_mode number of GUIDED
COMPANION_SYSID = 200    # our MAVLink system id
POSITION_ONLY = 0x09F8   # SET_POSITION_TARGET type_mask: use position and yaw, ignore the rest

# Set before arming and read back. Every failsafe that can choose RTL is set to Land or off:
# RTL would fly home straight across the other agents' areas.
PARAMS = {
    "WP_SPD": config.WP_SPD_MS,    # m/s, caps the speed of position targets in GUIDED
    "WP_ACC": 6.0,                 # m/s^2 (default 2.5): stop from cruise in ~20 m, not ~28 m
    "PSC_NE_JERK": 4.0,            # m/s^3 (default 5): at 12 it overshot WP_SPD after a brake
    "WP_SPD_UP": 4.0,              # m/s (default 2.5): an ALT_STEP_M climb must not slow cruise
    "WP_SPD_DN": 4.0,              # m/s (default 1.5): same for descents
    "ATC_ANGLE_MAX": 45.0,         # deg (default 30): the SITL quad tops out at 10.3 m/s at 30
    "MAV_GCS_SYSID": COMPANION_SYSID,  # our heartbeat is the GCS heartbeat
    "FS_GCS_ENABLE": 5,            # 5 = Land when our heartbeat stops (1, 3, 4, 6 can RTL)
    "FS_GCS_TIMEOUT": 3,           # s without our heartbeat before it lands
    "FS_EKF_ACTION": 1,            # 1 = Land
    "FS_THR_ENABLE": 3,            # 3 = Land on RC loss (copter.parm sets 1 = RTL)
    "FS_DR_ENABLE": 1,             # 1 = Land when dead reckoning starts (default 2 = RTL)
    "BATT_FS_LOW_ACT": 0,          # battery failsafes off (2 = RTL)
    "BATT_FS_CRT_ACT": 0,
    "FENCE_ENABLE": 0,             # no fence (its action is RTL)
}


def lead_s(speed_ms: float) -> float:
    """Lead time that makes ArduCopter fly toward the target at speed_ms.

    The speed of the position shaper depends only on the distance e to the target, and not in
    proportion. Steady state of shape_pos_vel_accel_xy() (AP_Math/control.cpp:356) with
    a = WP_ACC and k = PSC_NE_JERK / WP_ACC:
      linear part, e <= a / k^2:  speed = k e - speed           ->  e = 2 speed / k
      sqrt part:                  speed = c - a speed / (k c),  c = sqrt(2 a (e - a / 2k^2))
    The vehicle trails the shaped position by about LAG_S seconds, which is added.
    """
    a = PARAMS["WP_ACC"]
    k = PARAMS["PSC_NE_JERK"] / a
    if 2.0 * speed_ms / k <= a / k ** 2:
        return 2.0 / k + LAG_S
    c = (speed_ms + math.sqrt(speed_ms ** 2 + 4.0 * a * speed_ms / k)) / 2.0
    e_m = c ** 2 / (2.0 * a) + a / (2.0 * k ** 2)
    return e_m / speed_ms + LAG_S


class MavVehicle:
    """One agent's own ArduCopter (MAV_SYSID = agent_id) on TCP port config.sitl_tcp_port()."""

    def __init__(self, agent_id: int):
        self.agent_id = agent_id
        self.port = config.sitl_tcp_port(agent_id)
        self.sock = None
        self.out = b""                # packed bytes the socket has not taken yet
        self.mav = mavlink.MAVLink(self, srcSystem=COMPANION_SYSID,
                                   srcComponent=mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
        self.mav.robust_parsing = True
        self.last_rx_s = 0.0          # wall time of the last message from the autopilot
        self.last_heartbeat_s = 0.0   # wall time of our last heartbeat
        self.armed = False
        self.custom_mode = None
        self.pos = None
        self.vel = (0.0, 0.0, 0.0)
        self.yaw = 0.0
        self.yaw_cmd = 0.0            # rad, the yaw we ask for (lane_yaw)
        self.t_pos = 0.0              # wall time the last position was received
        self.params = {}              # parameter name -> value echoed by the autopilot
        self.acks = {}                # command id -> MAV_RESULT of its COMMAND_ACK
        self.statustext = []          # last few STATUSTEXT strings, for error messages

    # ---- link ------------------------------------------------------------------------
    def connect(self, timeout_s: float = 60.0) -> None:
        """Open TCP, wait for the first heartbeat and position, ask for 20 Hz telemetry."""
        deadline = time.time() + timeout_s
        while self.sock is None:
            try:
                self.sock = socket.create_connection(("127.0.0.1", self.port), timeout=1.0)
            except OSError:
                if time.time() > deadline:
                    raise RuntimeError(f"agent {self.agent_id}: no autopilot on {self.port}")
                time.sleep(0.5)
        self.sock.setblocking(False)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        if not self._wait(lambda: self.custom_mode is not None, deadline - time.time()):
            self._fail("no heartbeat")
        interval_us = int(1e6 / TELEMETRY_HZ)
        self._command(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                      mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, interval_us)
        if not self._wait(lambda: self.pos is not None, deadline - time.time()):
            self._fail("no position")

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def write(self, buf: bytes) -> None:
        """Called by pymavlink for every message it packs: queue it and send what fits."""
        if self.sock is not None:
            self.out += buf
            self._flush()

    def _flush(self) -> None:
        """Send queued bytes; a non-blocking send may take only part of them."""
        try:
            sent = self.sock.send(self.out)
        except BlockingIOError:
            return
        except OSError:               # connection reset: the autopilot is gone
            self.close()
            return
        self.out = self.out[sent:]

    def poll(self) -> None:
        """Send the heartbeat when due, read everything waiting. Never blocks."""
        now = time.time()
        if self.sock is not None and now - self.last_heartbeat_s >= HEARTBEAT_S:
            self.last_heartbeat_s = now
            self.mav.heartbeat_send(mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                    mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        if self.sock is not None and self.out:
            self._flush()
        while self.sock is not None:
            try:
                data = self.sock.recv(65536)
            except BlockingIOError:
                return
            except OSError:
                data = b""
            if not data:              # EOF or reset: the autopilot process is gone
                self.close()
                return
            for msg in self.mav.parse_buffer(data) or []:
                self._handle(msg)

    def _handle(self, msg) -> None:
        self.last_rx_s = time.time()
        kind = msg.get_type()
        if kind == "GLOBAL_POSITION_INT":
            if msg.lat == 0 and msg.lon == 0:   # no position yet (SITL before its GPS fix):
                return                          # it would be 13 500 km away, the same for all
            x, y = config.lla_to_enu(msg.lat * 1e-7, msg.lon * 1e-7)
            self.pos = (x, y, msg.relative_alt * 1e-3)
            self.vel = (msg.vy * 0.01, msg.vx * 0.01, -msg.vz * 0.01)   # NED cm/s -> ENU m/s
            self.yaw = math.radians(msg.hdg * 0.01)
            self.t_pos = self.last_rx_s
        elif kind == "HEARTBEAT":
            self.armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.custom_mode = msg.custom_mode
        elif kind == "PARAM_VALUE":
            self.params[msg.param_id] = msg.param_value
        elif kind == "COMMAND_ACK":
            self.acks[msg.command] = msg.result
        elif kind == "STATUSTEXT":
            self.statustext = (self.statustext + [msg.text])[-5:]

    def state(self) -> Optional[VehicleState]:
        """Latest state, or None before the first position."""
        if self.pos is None:
            return None
        link_ok = self.sock is not None and time.time() - self.last_rx_s <= LINK_TIMEOUT_S
        guided = self.armed and self.custom_mode == GUIDED
        return VehicleState(t=self.t_pos, pos=self.pos, vel=self.vel, yaw=self.yaw,
                            link_ok=link_ok, guided=guided)

    # ---- commands --------------------------------------------------------------------
    def _command(self, command: int, p1: float = 0, p2: float = 0, p7: float = 0) -> None:
        self.mav.command_long_send(self.agent_id, 1, command, 0, p1, p2, 0, 0, 0, 0, p7)

    def _wait(self, condition: Callable[[], bool], timeout_s: float) -> bool:
        """Poll until condition() is true (True) or timeout_s passes or the link dies (False)."""
        deadline = time.time() + timeout_s
        while not condition():
            if time.time() > deadline or self.sock is None:
                return False
            time.sleep(0.05)
            self.poll()
        return True

    def _fail(self, what: str) -> None:
        raise RuntimeError(f"agent {self.agent_id}: {what} (autopilot said: {self.statustext})")

    def set_params(self) -> Dict[str, float]:
        """Set PARAMS, read each one back, raise on a mismatch. Returns the read-back values."""
        for name, value in PARAMS.items():
            self.params.pop(name, None)
            self.mav.param_set_send(self.agent_id, 1, name.encode(), float(value),
                                    mavlink.MAV_PARAM_TYPE_REAL32)
            if not self._wait(lambda: name in self.params, 3.0):
                self._fail(f"parameter {name} not echoed")
            if abs(self.params[name] - value) > 1e-3:
                self._fail(f"parameter {name} is {self.params[name]}, wanted {value}")
        return {name: self.params[name] for name in PARAMS}

    def arm_and_takeoff(self, alt_m: float, timeout_s: float = 180.0) -> Dict[str, float]:
        """Set PARAMS, GUIDED, arm, climb straight up to alt_m. Returns the read-back PARAMS."""
        deadline = time.time() + timeout_s
        readback = self.set_params()
        self._command(mavlink.MAV_CMD_DO_SET_MODE, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                      GUIDED)
        if not self._wait(lambda: self.custom_mode == GUIDED, 5.0):
            self._fail("mode did not change to GUIDED")
        # Arming is refused until the EKF has settled after boot, so keep asking.
        while not self.armed:
            if time.time() > deadline:
                self._fail(f"not armed after {timeout_s} s")
            self._command(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            self._wait(lambda: self.armed, 2.0)
        self.acks.pop(mavlink.MAV_CMD_NAV_TAKEOFF, None)
        self._command(mavlink.MAV_CMD_NAV_TAKEOFF, p7=alt_m)
        if not self._wait(lambda: mavlink.MAV_CMD_NAV_TAKEOFF in self.acks, 5.0):
            self._fail("no COMMAND_ACK for takeoff")
        if self.acks[mavlink.MAV_CMD_NAV_TAKEOFF] != mavlink.MAV_RESULT_ACCEPTED:
            self._fail(f"takeoff refused, result {self.acks[mavlink.MAV_CMD_NAV_TAKEOFF]}")
        if not self._wait(lambda: self.pos[2] >= alt_m - 1.0, deadline - time.time()):
            self._fail(f"did not reach {alt_m} m, at {self.pos[2]:.1f} m")
        return readback

    def send_setpoint(self, vx: float, vy: float, z: float) -> None:
        """Position target lead_s ahead along (vx, vy) ENU m/s, altitude z m, nose along the lane."""
        x_m, y_m, z_m = self.pos
        t_lead_s = lead_s(math.hypot(vx, vy))
        lat, lon = config.enu_to_lla(x_m + vx * t_lead_s, y_m + vy * t_lead_s)
        z_target_m = z_m + max(-ALT_STEP_M, min(ALT_STEP_M, z - z_m))
        self.yaw_cmd = lane_yaw(vx, vy, self.yaw_cmd)
        self.mav.set_position_target_global_int_send(
            0, self.agent_id, 1, mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, POSITION_ONLY,
            round(lat * 1e7), round(lon * 1e7), z_target_m, 0, 0, 0, 0, 0, 0, self.yaw_cmd, 0)
