"""sim/demo_world.py (SWARM_DEMO, the live view): with SWARM_DEMO_PHYSICS the body is the iris
Froude scaled; off, the world is the evaluation's. No simulator needed."""
import math
import os
import xml.etree.ElementTree as ET

import pytest

import run_mission
from sim import demo_world, gazebo_world

POSITIONS = [(140.0, 0.0), (150.0, 0.0), (160.0, 0.0)]
STANDOFFS = os.path.join(gazebo_world.AP_GAZEBO, "models", "iris_with_standoffs", "model.sdf")
IRIS = os.path.join(gazebo_world.AP_GAZEBO, "models", "iris_with_ardupilot", "model.sdf")
needs_ap_gazebo = pytest.mark.skipif(not os.path.exists(IRIS),
                                     reason="ardupilot_gazebo not installed")


def test_colours_and_zone_match_the_owner_and_the_runner():
    # drone 1 blue, 2 dark green, 3 black
    assert demo_world.DRONE_COLORS[:3] == ["#1f5fd6", "#0b6b2a", "#111111"]
    # Gazebo takes material colours as linear light: the round trip gives the screen colour back
    dark_blue = demo_world.linear(demo_world.rgb("#0a2a7a"))
    assert [round(255 * (1.055 * v ** (1 / 2.4) - 0.055)) for v in dark_blue[1:]] == [42, 122]
    assert tuple(demo_world.MIDDLE_ZONE) == tuple(run_mission.MIDDLE_ZONE)


def lift_per_rad_s2(plugin):
    """Lift of one blade per (rad/s)^2 with no inflow: 1/2 rho (omega cp)^2 area cla a0."""
    value = {tag: float(plugin.find(tag).text)
             for tag in ("a0", "cla", "area", "air_density")}
    cp = float(plugin.find("cp").text.split()[0])
    return 0.5 * value["air_density"] * cp ** 2 * value["area"] * value["cla"] * value["a0"]


def thrust_to_weight(body_text, plugins_text):
    """Top thrust of all blades over the weight of all links (the iris: about 3.5)."""
    mass = sum(float(m.text) for m in ET.fromstring(body_text).iter("mass"))
    model = ET.fromstring(plugins_text).find("model")
    top = abs(float(model.find("plugin[@name='ArduPilotPlugin']/control/multiplier").text))
    blades = model.findall("plugin[@name='gz::sim::systems::LiftDrag']")
    lift = sum(lift_per_rad_s2(p) for p in blades)
    return lift * top ** 2 / (mass * 9.81)


@needs_ap_gazebo
def test_physical_drone_is_the_iris_froude_scaled():
    with open(STANDOFFS) as f:
        stock_body = f.read()
    with open(IRIS) as f:
        stock_plugins = f.read()
    s = 6.0
    body = demo_world.body_model(stock_body, demo_world.DRONE_COLORS[0], scale=s)
    plugins = demo_world.physical_plugins(stock_plugins, s)
    stock, big = ET.fromstring(stock_body).find("model"), ET.fromstring(body).find("model")
    for link in stock.findall("link"):
        twin = big.find(f"link[@name='{link.get('name')}']")
        assert float(twin.find("inertial/mass").text) == pytest.approx(
            float(link.find("inertial/mass").text) * s ** 3)
        assert float(twin.find("inertial/inertia/izz").text) == pytest.approx(
            float(link.find("inertial/inertia/izz").text) * s ** 5)
    rotor = big.find("link[@name='rotor_0']")
    assert rotor.find("pose").text.split()[:3] == ["0.78", "-1.32", "0.138"]
    # the prop you see is the rotor: no extra links or plugins, a blur disc on the rotor
    assert len(big.findall("link")) == len(stock.findall("link"))
    assert big.find("plugin") is None
    assert [v.get("name") for v in rotor.findall("visual")] == ["visual", "prop_blur"]
    assert all(v.find("cast_shadows").text == "false" for v in big.iter("visual"))
    # same thrust to weight ratio, so the same hover throttle
    assert thrust_to_weight(body, plugins) == pytest.approx(
        thrust_to_weight(stock_body, stock_plugins), rel=1e-4)
    # spin-up: rotor inertia over torque limit goes as s^1/2 like every other time
    control = ET.fromstring(plugins).find(".//control")
    stock_control = ET.fromstring(stock_plugins).find(".//control")
    spin_up = (float(rotor.find("inertial/inertia/izz").text)
               * float(control.find("multiplier").text) / float(control.find("cmd_max").text))
    stock_rotor = stock.find("link[@name='rotor_0']")
    stock_spin_up = (float(stock_rotor.find("inertial/inertia/izz").text)
                     * float(stock_control.find("multiplier").text)
                     / float(stock_control.find("cmd_max").text))
    assert spin_up == pytest.approx(stock_spin_up * s ** 0.5)


def parm(text):
    return {name: float(value) for name, value in
            (line.split() for line in text.splitlines() if not line.startswith("#"))}


def test_physical_params_slow_the_stock_tune_down():
    params = parm(demo_world.physical_params(4.0))
    assert params["ATC_RAT_RLL_P"] == pytest.approx(0.135 * 2)
    assert params["ATC_RAT_RLL_D"] == pytest.approx(0.0036 * 4)
    assert params["ATC_ANG_PIT_P"] == pytest.approx(4.5 / 2)
    assert params["ATC_ACC_R_MAX"] == pytest.approx(1100 / 4)
    assert params["PSC_NE_VEL_P"] == pytest.approx(2.0 / 2)
    assert params["ATC_RAT_YAW_I"] == pytest.approx(0.02)
    # at scale 1 it is the stock tune
    stock = parm(demo_world.physical_params(1.0))
    assert stock == pytest.approx({k: v for k, (v, _) in demo_world.SCALED_GAINS.items()})


def test_physics_needs_demo_and_a_scale(monkeypatch):
    monkeypatch.setattr(demo_world, "PHYSICS", True)
    monkeypatch.setattr(demo_world, "DRONE_SCALE", 6.0)
    assert gazebo_world.physical_demo(demo=True)
    assert gazebo_world.spawn_z_m(True) == pytest.approx(0.195 * 6)
    assert not gazebo_world.physical_demo(demo=False)   # the evaluation
    assert gazebo_world.spawn_z_m() == gazebo_world.SPAWN_Z_M
    monkeypatch.setattr(demo_world, "DRONE_SCALE", 1.0)
    assert not gazebo_world.physical_demo(demo=True)


@needs_ap_gazebo
def test_without_demo_the_world_is_the_evaluations(tmp_path):
    path = gazebo_world.generate(str(tmp_path), POSITIONS, (0.0, 0.0, 0.0))
    text = open(path).read()
    assert "demo_overlays" not in text and "<light" not in text
    assert sorted(os.listdir(tmp_path / "models")) == ["iris_0", "iris_1", "iris_2"]


@needs_ap_gazebo
def test_demo_world_has_border_zone_and_coloured_bodies(tmp_path, monkeypatch):
    monkeypatch.setattr(demo_world, "PHYSICS", True)
    monkeypatch.setattr(demo_world, "DRONE_SCALE", 6.0)
    path = gazebo_world.generate(str(tmp_path), POSITIONS, (0.0, 0.0, 0.0), demo=True,
                                 demo_zone=True)
    root = ET.parse(path).getroot()
    overlay = root.find(".//model[@name='demo_overlays']")
    names = {v.get("name") for v in overlay.iter("visual")}
    assert {"ground", "border_s", "zone_s", "lane_0"} <= names
    assert overlay.find(".//collision") is None and overlay.find(".//inertial") is None
    for i in range(3):
        iris = open(tmp_path / "models" / f"iris_{i}" / "model.sdf").read()
        assert f"<uri>model://iris_{i}_body</uri>" in iris
        assert (tmp_path / "models" / f"iris_{i}_body" / "model.sdf").exists()


def blade_tip_m(dae_path):
    """Largest distance of a prop mesh vertex from the hub axis (z)."""
    root = ET.parse(dae_path).getroot()
    tip = 0.0
    for array in root.iter():
        if array.tag.endswith("float_array") and "positions" in array.get("id", ""):
            v = [float(x) for x in array.text.split()]
            tip = max([tip] + [math.hypot(x, y) for x, y in zip(v[0::3], v[1::3])])
    return tip


@needs_ap_gazebo
def test_blades_stay_inside_their_blur_disc_on_the_hub():
    with open(STANDOFFS) as f:
        stock = f.read()
    meshes = os.path.join(os.path.dirname(STANDOFFS), "meshes")
    tip = max(blade_tip_m(os.path.join(meshes, f"iris_prop_{d}.dae")) for d in ("cw", "ccw"))
    model = ET.fromstring(demo_world.body_model(stock, demo_world.DRONE_COLORS[0], scale=6))
    for disc in model.iter("visual"):
        if disc.get("name") != "prop_blur":
            continue
        x, y, z = map(float, disc.find("pose").text.split()[:3])
        assert (x, y) == (0, 0)                          # on the hub, the joint axis
        assert z == pytest.approx(6 * demo_world.PROP_LIFT_M)   # level with the blades
        assert float(disc.find("geometry/cylinder/radius").text) >= 6 * tip
