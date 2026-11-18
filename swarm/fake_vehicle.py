"""FakeVehicle: kinematic stand-in for the autopilot, implementing the Vehicle protocol.

Model (per poll(), integrated in sub-steps of <= 20 ms):
  horizontal: first-order velocity response to the commanded (vx, vy) with time constant TAU_V,
              acceleration magnitude limited to A_MAX_XY (3 m/s^2), speed limited to V_MAX.
  vertical:   vz_cmd = clamp(KP_Z * (z_cmd - z), +-VZ_MAX) (2.5 m/s), same first-order response.
  guided timeout: if no setpoint for GUIDED_TIMEOUT_S, the commanded velocity becomes zero.
Real time by default; tests pass `clock` (and `sleep`) to run in simulated time.
"""
import math
import threading
import time
from typing import Callable, Optional, Tuple

from swarm import config as C
from swarm.vehicle_api import VehicleState

TAU_V = 0.3
A_MAX_XY = 3.0
VZ_MAX = 2.5
A_MAX_Z = 3.0
KP_Z = 1.0
GUIDED_TIMEOUT_S = 3.0
MAX_SUBSTEP = 0.02


class FakeVehicle:
    def __init__(self, agent_id: int, start: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 clock: Callable[[], float] = time.time, sleep: Callable[[float], None] = time.sleep):
        self.id = agent_id
        self.clock, self.sleep = clock, sleep
        self.pos = list(start)
        self.vel = [0.0, 0.0, 0.0]
        self.cmd = (0.0, 0.0, start[2])
        self.armed = start[2] > 0.5
        self.link_ok = True
        self._t_last: Optional[float] = None
        self._t_cmd: Optional[float] = None
        self._connected = False
        self._lock = threading.Lock()

    # ---- Vehicle protocol -------------------------------------------------------------------
    def connect(self) -> None:
        self._connected = True
        self._t_last = self.clock()
        self._t_cmd = self._t_last

    def arm_and_takeoff(self, alt: float, timeout: float = 180.0) -> None:
        """Blocks until within 0.5 m of `alt`. Holds position horizontally while climbing."""
        t0 = self.clock()
        with self._lock:
            self.armed = True
        while True:
            with self._lock:
                self.cmd = (0.0, 0.0, alt)
                self._t_cmd = self.clock()
            self.poll()
            if abs(self.pos[2] - alt) < 0.5:
                return
            if self.clock() - t0 > timeout:
                raise TimeoutError("takeoff timeout")
            self.sleep(0.05)

    def poll(self) -> None:
        now = self.clock()
        with self._lock:
            if self._t_last is None:
                self._t_last = now
            dt = now - self._t_last
            self._t_last = now
            while dt > 1e-9:
                h = min(dt, MAX_SUBSTEP)
                self._integrate(h, now)
                dt -= h

    def state(self) -> Optional[VehicleState]:
        if not self._connected:
            return None
        with self._lock:
            return VehicleState(t=self._t_last, pos=tuple(self.pos), vel=tuple(self.vel), yaw=0.0,
                                armed=self.armed, link_ok=self.link_ok)

    def send_setpoint(self, vx: float, vy: float, z: float) -> None:
        with self._lock:
            self.cmd = (vx, vy, z)
            self._t_cmd = self.clock()

    # ---- test helpers -----------------------------------------------------------------------
    def kill_link(self) -> None:
        """Simulate the autopilot link dying (state().link_ok becomes False)."""
        self.link_ok = False

    # ---- dynamics ---------------------------------------------------------------------------
    def _integrate(self, h: float, now: float) -> None:
        if not self.armed:
            return
        vx_c, vy_c, z_c = self.cmd
        if self._t_cmd is not None and now - self._t_cmd > GUIDED_TIMEOUT_S:
            vx_c, vy_c = 0.0, 0.0
        s = math.hypot(vx_c, vy_c)
        if s > C.V_MAX:
            vx_c, vy_c = vx_c * C.V_MAX / s, vy_c * C.V_MAX / s
        # horizontal: first-order response, acceleration-limited
        ax = (vx_c - self.vel[0]) / TAU_V
        ay = (vy_c - self.vel[1]) / TAU_V
        a = math.hypot(ax, ay)
        if a > A_MAX_XY:
            ax, ay = ax * A_MAX_XY / a, ay * A_MAX_XY / a
        self.vel[0] += ax * h
        self.vel[1] += ay * h
        # vertical
        vz_c = max(-VZ_MAX, min(VZ_MAX, KP_Z * (z_c - self.pos[2])))
        az = max(-A_MAX_Z, min(A_MAX_Z, (vz_c - self.vel[2]) / TAU_V))
        self.vel[2] += az * h
        for i in range(3):
            self.pos[i] += self.vel[i] * h
        if self.pos[2] < 0.0:
            self.pos[2], self.vel[2] = 0.0, 0.0
