"""The only way agent code touches its own autopilot (the wired companion-autopilot link).

Implementations: swarm/mav_vehicle.py (pymavlink, real SITL) and sim/fake_vehicle.py (tests).
"""
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
