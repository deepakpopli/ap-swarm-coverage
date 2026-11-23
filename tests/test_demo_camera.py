"""sim/demo_camera.py (SWARM_DEMO_CAMERA, the live view's drone cameras) and the pose helper of
tools/cam_bridge.py. No simulator needed."""
import hashlib
import math
import os
import xml.etree.ElementTree as ET

import pytest

from sim import demo_camera, demo_world, gazebo_world

POSITIONS = [(140.0, 0.0), (150.0, 0.0), (160.0, 0.0)]
IRIS = os.path.join(gazebo_world.AP_GAZEBO, "models", "iris_with_ardupilot", "model.sdf")
needs_ap_gazebo = pytest.mark.skipif(not os.path.exists(IRIS),
                                     reason="ardupilot_gazebo not installed")
# swarm.sdf of the evaluation for POSITIONS at origin (0, 0, 0), before the cameras existed
EVALUATION_WORLD_SHA = "5f329239f388a953"


def world_files(out_dir, **kwargs):
    path = gazebo_world.generate(str(out_dir), POSITIONS, (0.0, 0.0, 0.0), **kwargs)
    files = {"swarm.sdf": open(path, "rb").read()}
    for i in range(3):
        files[f"iris_{i}"] = open(out_dir / "models" / f"iris_{i}" / "model.sdf", "rb").read()
    return files


@needs_ap_gazebo
def test_the_evaluation_world_is_unchanged_with_or_without_the_flag(tmp_path, monkeypatch):
    plain = world_files(tmp_path / "plain")
    assert hashlib.sha256(plain["swarm.sdf"]).hexdigest()[:16] == EVALUATION_WORLD_SHA
    monkeypatch.setattr(demo_camera, "CAMERA", True)      # the camera needs SWARM_DEMO too
    assert world_files(tmp_path / "flag") == plain
    for text in plain.values():
        assert b"nadir_cam" not in text and b"gz::sim::systems::Sensors" not in text


@needs_ap_gazebo
def test_demo_without_the_flag_has_no_camera(tmp_path, monkeypatch):
    monkeypatch.setattr(demo_camera, "CAMERA", False)
    files = world_files(tmp_path, demo=True)
    assert all(b"nadir_cam" not in t and b"gz::sim::systems::Sensors" not in t
               for t in files.values())


@needs_ap_gazebo
@pytest.mark.parametrize("physics", [False, True])
def test_demo_camera_is_nadir_81_deg_on_each_drone(tmp_path, monkeypatch, physics):
    monkeypatch.setattr(demo_camera, "CAMERA", True)
    monkeypatch.setattr(demo_world, "PHYSICS", physics)
    monkeypatch.setattr(demo_world, "DRONE_SCALE", 6.0)
    files = world_files(tmp_path, demo=True)
    assert files["swarm.sdf"].count(b"gz::sim::systems::Sensors") == 1
    assert b"<render_engine>ogre2</render_engine>" in files["swarm.sdf"]
    for i in range(3):
        model = ET.fromstring(files[f"iris_{i}"]).find("model")
        cam = model.find("link[@name='cam_link']/sensor[@name='nadir_cam']")
        assert cam.findtext("topic") == f"/swarm/cam/{i + 1}"
        assert cam.findtext("update_rate") == "10" and cam.findtext("always_on") == "1"
        assert float(cam.findtext("camera/horizontal_fov")) == pytest.approx(math.radians(81))
        assert (cam.findtext("camera/image/width"), cam.findtext("camera/image/height")) == (
            "640", "360")
        # pitched 90 deg down: looks along the drone's -z, image up = the nose (+x)
        assert [float(v) for v in cam.findtext("pose").split()] == pytest.approx(
            [0, 0, 0, 0, math.pi / 2, 0], abs=1e-6)
        joint = model.find("joint[@name='cam_joint']")
        assert joint.get("type") == "fixed"
        assert joint.findtext("parent") == "iris_with_standoffs::base_link"
        assert float(cam.findtext("camera/clip/near")) == pytest.approx(0.3 * (6 if physics else 1))


def quat_from_yaw(enu_yaw):
    return math.cos(enu_yaw / 2), 0.0, 0.0, math.sin(enu_yaw / 2)


@pytest.mark.parametrize("enu_deg, compass_deg", [(90, 0), (0, 90), (180, -90), (-90, -180),
                                                  (45, 45), (135, -45)])
def test_compass_yaw_is_north_zero_clockwise(enu_deg, compass_deg):
    # the drone spawns at ENU yaw 90 deg (nose north): the agent csv says yaw 0
    x, y, z, yaw = demo_camera.compass_pose(1.0, 2.0, 3.0, *quat_from_yaw(math.radians(enu_deg)))
    assert (x, y, z) == (1.0, 2.0, 3.0)
    assert math.degrees(yaw) == pytest.approx(compass_deg)


def test_compass_yaw_ignores_roll_and_pitch():
    # a small roll about the nose does not change the heading (nose east: compass 90 deg)
    r = math.radians(10) / 2
    qw, qx = math.cos(r), math.sin(r)
    _, _, _, yaw = demo_camera.compass_pose(0, 0, 0, qw, qx, 0.0, 0.0)
    assert math.degrees(yaw) == pytest.approx(90.0)
