"""The only way agent code touches its own autopilot (the wired companion<->autopilot link).

Implementations: swarm/mav_vehicle.py (pymavlink, real SITL) and swarm/fake_vehicle.py (kinematic, tests).
"""
from dataclasses import dataclass
from typing import Optional, Protocol, Tuple

@dataclass
class VehicleState:
    t: float                      # wall time the sample was received
    pos: Tuple[float, float, float]   # area ENU metres (x east, y north, z up above area origin)
    vel: Tuple[float, float, float]   # ENU m/s
    yaw: float                    # rad, 0 = north, clockwise positive (compass)
    armed: bool
    link_ok: bool                 # False if the autopilot has been silent > 1 s

class Vehicle(Protocol):
    def connect(self) -> None: ...
    def arm_and_takeoff(self, alt: float, timeout: float = 180.0) -> None:
        """Blocks until at altitude in GUIDED. Applies the speed-limit parameters first."""
    def poll(self) -> None:
        """Pump incoming telemetry; call at >= 20 Hz."""
    def state(self) -> Optional[VehicleState]: ...
    def send_setpoint(self, vx: float, vy: float, z: float) -> None:
        """Horizontal velocity (ENU, m/s) plus target altitude z (m). Called at CONTROL_HZ. Yaw is held north."""
