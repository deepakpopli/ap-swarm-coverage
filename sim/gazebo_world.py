#!/usr/bin/env python3
"""Generate a headless-friendly Gazebo Harmonic world with N iris copters.

Each copter is a copy of ardupilot_gazebo's iris_with_ardupilot model with its
own ArduPilotPlugin fdm_port_in (9002 + 10*instance), matching
`arducopter -I instance --model JSON`.

World frame: Gazebo's world frame is ENU (x east, y north, z up) anchored at
<spherical_coordinates>. By default that anchor is the stock ArduPilot test site
and the models are spaced 10 m apart along +x (east), as before.

CLI (standalone, unchanged default behaviour):
    gen_world.py N OUT_DIR [--no-lock-step] [--swarm-layout] [--base-instance K]
  --swarm-layout   anchor the world at config.ORIGIN_LLA (the coverage area origin)
                   and put model i at config.start_xy(i, N)
  --base-instance  first SITL instance number (fdm ports 9002+10*(K+i))

Library use (sim/launch.py):
    generate(out_dir, positions_xy, origin_lla, instances, lock_step)
"""
import argparse
import os
import re
import shutil
import sys

AG = os.path.expanduser(os.environ.get("ARDUPILOT_GAZEBO", "~/ardupilot_gazebo"))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Stock ArduPilot SITL site (CMAC); also equal to config.ORIGIN_LLA at the time of writing.
DEFAULT_ORIGIN = (-35.363262, 149.165237, 584.0)
SPAWN_Z = 0.195   # m, iris landing-gear height so it rests on the ground plane


def write_model(out_dir, name, fdm_port, lock_step):
    """Copy iris_with_ardupilot to out_dir/models/<name> with its own FDM port and lock-step setting."""
    src = os.path.join(AG, "models", "iris_with_ardupilot")
    dst = os.path.join(out_dir, "models", name)
    shutil.rmtree(dst, ignore_errors=True)
    shutil.copytree(src, dst)
    p = os.path.join(dst, "model.sdf")
    s = open(p).read()
    s = s.replace('<model name="iris_with_ardupilot">', f'<model name="{name}">')
    s = re.sub(r"<fdm_port_in>\d+</fdm_port_in>", f"<fdm_port_in>{fdm_port}</fdm_port_in>", s)
    if not lock_step:
        # lock_step=1 blocks the whole world while one autopilot is silent, so a
        # SIGKILLed SITL would freeze every other vehicle. Free-running avoids that.
        s = s.replace("<lock_step>1</lock_step>", "<lock_step>0</lock_step>")
    open(p, "w").write(s)
    c = os.path.join(dst, "model.config")
    if os.path.exists(c):
        txt = re.sub(r"<name>.*?</name>", f"<name>{name}</name>", open(c).read(), count=1)
        open(c, "w").write(txt)


def generate(out_dir, positions_xy, origin_lla=DEFAULT_ORIGIN, instances=None, lock_step=True):
    """Write out_dir/swarm.sdf plus one model per vehicle; return the .sdf path.

    positions_xy: list of (x east, y north) in metres in the world (= area ENU) frame.
    origin_lla:   (lat deg, lon deg, alt m AMSL) of the world origin.
    instances:    SITL instance number per vehicle (default 0..n-1); model k is named iris_<instance>.
    """
    n = len(positions_xy)
    instances = list(range(n)) if instances is None else list(instances)
    os.makedirs(os.path.join(out_dir, "models"), exist_ok=True)
    includes = []
    for (x, y), inst in zip(positions_xy, instances):
        name = f"iris_{inst}"
        write_model(out_dir, name, 9002 + 10 * inst, lock_step)
        # yaw 90 deg: the iris model's nose is its +x; this points it north, matching SITL heading 0
        includes.append(f'    <include><uri>model://{name}</uri><name>{name}</name>'
                        f'<pose degrees="true">{x:.3f} {y:.3f} {SPAWN_Z} 0 0 90</pose></include>')
    lat, lon, alt = origin_lla
    world = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="swarm">
    <physics name="1ms" type="ignore"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <scene><ambient>1 1 1</ambient><background>0.8 0.8 0.8</background></scene>
    <spherical_coordinates><latitude_deg>{lat}</latitude_deg><longitude_deg>{lon}</longitude_deg>
      <elevation>{alt}</elevation><heading_deg>0</heading_deg><surface_model>EARTH_WGS84</surface_model></spherical_coordinates>
    <light type="directional" name="sun"><pose>0 0 10 0 0 0</pose><direction>-0.5 0.1 -0.9</direction></light>
    <model name="ground"><static>1</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>2000 2000</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>2000 2000</size></plane></geometry>
        <material><ambient>0.5 0.6 0.5 1</ambient><diffuse>0.5 0.6 0.5 1</diffuse></material></visual>
    </link></model>
{chr(10).join(includes)}
  </world>
</sdf>
"""
    path = os.path.join(out_dir, "swarm.sdf")
    open(path, "w").write(world)
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("n", type=int)
    ap.add_argument("out_dir")
    ap.add_argument("--no-lock-step", action="store_true")
    ap.add_argument("--swarm-layout", action="store_true")
    ap.add_argument("--base-instance", type=int, default=0)
    a = ap.parse_args()
    instances = [a.base_instance + i for i in range(a.n)]
    if a.swarm_layout:
        sys.path.insert(0, ROOT)
        from swarm import config
        positions = [config.start_xy(i, a.n) for i in range(a.n)]
        origin = config.ORIGIN_LLA
    else:
        positions = [(10.0 * i, 0.0) for i in range(a.n)]
        origin = DEFAULT_ORIGIN
    print(generate(a.out_dir, positions, origin, instances, lock_step=not a.no_lock_step))


if __name__ == "__main__":
    main()
