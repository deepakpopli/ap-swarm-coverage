"""MavVehicle's message handling, without an autopilot."""
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from swarm import config
from swarm.mav_vehicle import MavVehicle


def position(lat_deg, lon_deg):
    return mavlink.MAVLink_global_position_int_message(
        0, round(lat_deg * 1e7), round(lon_deg * 1e7), 0, 50000, 0, 0, 0, 0)


def test_no_position_before_the_autopilot_has_one():
    """SITL sends lat = lon = 0 before its GPS fix (seen in the SITL smoke run)."""
    vehicle = MavVehicle(1)
    vehicle._handle(position(0.0, 0.0))
    assert vehicle.state() is None
    vehicle._handle(position(*config.ORIGIN_LLA[:2]))
    assert vehicle.state().pos[:2] == (0.0, 0.0)
