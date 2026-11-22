"""The only way agent code touches its own autopilot (the wired companion-autopilot link).

Implementations: swarm/mav_vehicle.py (pymavlink, real SITL) and sim/fake_vehicle.py (tests).
"""
import math
from dataclasses import dataclass
from typing import Dict, Optional, Protocol, Tuple


@dataclass
class VehicleState:
    t: float                          # wall time the sample was received
    pos: Tuple[float, float, float]   # area ENU metres, z above the area origin
    vel: Tuple[float, float, float]   # ENU m/s
    yaw: float                        # rad, 0 = north, clockwise
    link_ok: bool                     # False once the autopilot is silent or disconnected
    guided: bool                      # True while the autopilot is armed and in GUIDED mode


YAW_MIN_MS = 2.0   # slower than this: keep the heading (a hover has no direction)


def travel_yaw(vx: float, vy: float, last: float) -> float:
    """Nose where we fly, like ArduPilot's own WP_YAW_BEHAVIOR default; slow: keep the last yaw.
    Yaw in rad, 0 = north, clockwise. On the lanes it is north or south and the footprint is the
    same turned by 180 deg; on the short hops between lanes the footprint turns with the nose."""
    if math.hypot(vx, vy) < YAW_MIN_MS:
        return last
    return math.atan2(vx, vy) % (2 * math.pi)


class Vehicle(Protocol):
    def connect(self) -> None: ...

    def arm_and_takeoff(self, alt_m: float, timeout_s: float = 180.0) -> Dict[str, float]:
        """Set the parameters (speed cap, failsafes), arm, climb straight up to alt_m.
        Returns the parameters as read back from the autopilot."""

    def poll(self) -> None:
        """Read incoming telemetry without blocking. Call at CONTROL_HZ or faster."""

    def state(self) -> Optional[VehicleState]: ...

    def send_setpoint(self, vx: float, vy: float, z: float) -> None:
        """Horizontal velocity (ENU, m/s) and target altitude (m). Call at CONTROL_HZ."""
