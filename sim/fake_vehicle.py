"""FakeVehicle: a stand-in autopilot for tests, with the same actuator path as MavVehicle.

send_setpoint(vx, vy, z) becomes a position target, position + v * T_LEAD_S, as in MavVehicle.
The autopilot acts on it REACT_S later (the target is aimed from where the vehicle is then) and
flies toward it the way the measured ArduCopter does: it speeds up at A_UP_MS2, slows down at
A_BRAKE_MS2, never exceeds WP_SPD_MS, and climbs or descends at most VZ_MAX_MS. A zero velocity
puts the target on the vehicle, so it brakes as hard as it can, like the real one.
Numbers measured on ArduCopter in SITL and Gazebo. Real time through poll(); tests call
step(dt) or pass a simulated clock.
"""
import math
import threading
import time
from collections import deque

from swarm import config
from swarm.vehicle_api import VehicleState, lane_yaw

T_LEAD_S = 1.4       # lead of the target; MavVehicle's lead_s() is 1.05-1.63 s by speed
REACT_S = 0.25       # measured: speed starts to fall 0.2-0.3 s after a zero setpoint
A_UP_MS2 = 2.6       # measured: 0 -> 95 % of 10 m/s in 3.5-4.2 s
A_BRAKE_MS2 = 4.2    # measured: stop from 10 m/s in 13.9-15.0 m and 2.4-2.9 s
VZ_MAX_MS = 2.9      # measured: max vertical speed 2.91-2.92 m/s
KP_Z = 0.5           # measured: 10 m climb in 5.1 s, 20 m descent in 8.6 s (within 1 m)
SUBSTEP_S = 0.01


class FakeVehicle:
    def __init__(self, agent_id: int, start=(0.0, 0.0, 0.0), clock=time.time):
        self.id = agent_id
        self.clock = clock
        self.t = clock()
        self.pos = list(start)
        self.vel = [0.0, 0.0, 0.0]
        self.yaw = 0.0                 # rad, 0 = north (lane_yaw)
        self.target = tuple(start)
        self.pending = deque()   # (time the autopilot acts on it, (vx, vy, z))
        self.link_ok = True      # tests set these two to simulate failures
        self.guided = start[2] > 1.0
        self.lock = threading.Lock()

    def connect(self) -> None:
        pass

    def arm_and_takeoff(self, alt_m: float, timeout_s: float = 180.0) -> dict:
        """Climb straight up to alt_m. Blocks, polling in real time, like MavVehicle.
        Returns no parameters: there is no autopilot to read them from."""
        self.guided = True
        self.send_setpoint(0.0, 0.0, alt_m)
        deadline = time.monotonic() + timeout_s
        while self.pos[2] < alt_m - 1.0:
            if time.monotonic() > deadline:
                raise RuntimeError(f"agent {self.id}: takeoff timeout")
            self.poll()
            time.sleep(0.002)
        return {}

    def poll(self) -> None:
        with self.lock:
            self._advance(self.clock() - self.t)

    def step(self, dt: float) -> None:
        with self.lock:
            self._advance(dt)

    def state(self) -> VehicleState:
        return VehicleState(self.t, tuple(self.pos), tuple(self.vel), self.yaw, self.link_ok,
                            self.guided)

    def send_setpoint(self, vx: float, vy: float, z: float) -> None:
        with self.lock:
            self.pending.append((self.t + REACT_S, (vx, vy, z)))
            self.yaw = lane_yaw(vx, vy, self.yaw)   # turns at once: the footprint is the same

    def _advance(self, dt: float) -> None:
        while dt > 1e-9:
            h = min(dt, SUBSTEP_S)
            self._integrate(h)
            self.t += h
            dt -= h

    def _integrate(self, h: float) -> None:
        while self.pending and self.pending[0][0] <= self.t:
            vx, vy, z = self.pending.popleft()[1]
            self.target = (self.pos[0] + vx * T_LEAD_S, self.pos[1] + vy * T_LEAD_S, z)
        # horizontal: fly toward the target at the speed that covers the lead in T_LEAD_S
        want_x = (self.target[0] - self.pos[0]) / T_LEAD_S
        want_y = (self.target[1] - self.pos[1]) / T_LEAD_S
        want = math.hypot(want_x, want_y)
        if want > config.WP_SPD_MS:
            want_x, want_y = want_x * config.WP_SPD_MS / want, want_y * config.WP_SPD_MS / want
        speed = math.hypot(self.vel[0], self.vel[1])
        accel = A_BRAKE_MS2 if math.hypot(want_x, want_y) < speed else A_UP_MS2
        dvx, dvy = want_x - self.vel[0], want_y - self.vel[1]
        dv = math.hypot(dvx, dvy)
        if dv > accel * h:
            dvx, dvy = dvx * accel * h / dv, dvy * accel * h / dv
        self.vel[0] += dvx
        self.vel[1] += dvy
        # vertical: proportional climb or descent, speed-limited
        self.vel[2] = max(-VZ_MAX_MS, min(VZ_MAX_MS, KP_Z * (self.target[2] - self.pos[2])))
        for i in range(3):
            self.pos[i] += self.vel[i] * h
