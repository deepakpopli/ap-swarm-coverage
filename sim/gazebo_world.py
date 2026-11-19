"""Write a Gazebo Harmonic world with one ardupilot_gazebo iris per vehicle.

Model iris_<i> talks to `arducopter -I <i> --model JSON` on UDP port 9002 + 10 * i.
The world frame is ENU (x east, y north) anchored at `origin` (lat, lon, alt).
Lock-step is always off: with lock_step=1 the world waits for every autopilot on every step,
so one killed autopilot would freeze the world and every other vehicle with it.
"""
import os
import shutil

AP_GAZEBO = os.path.expanduser(os.environ.get("ARDUPILOT_GAZEBO", "~/ardupilot_gazebo"))
SPAWN_Z_M = 0.195        # iris landing-gear height, so it rests on the ground plane

WORLD = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="swarm">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <spherical_coordinates>
      <latitude_deg>{lat}</latitude_deg><longitude_deg>{lon}</longitude_deg>
      <elevation>{alt}</elevation><heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>
    <model name="ground"><static>1</static><link name="link"><collision name="c">
      <geometry><plane><normal>0 0 1</normal><size>2000 2000</size></plane></geometry>
    </collision></link></model>
{includes}
  </world>
</sdf>
"""

# yaw 90 deg: the iris nose is its +x axis, this points it north like SITL's heading 0
INCLUDE = ('    <include><uri>model://{name}</uri><name>{name}</name>'
           '<pose degrees="true">{x:.3f} {y:.3f} {z} 0 0 90</pose></include>')


def write_model(out_dir: str, name: str, fdm_port: int) -> None:
    """Copy iris_with_ardupilot to out_dir/models/<name> with its own FDM port, no lock-step."""
    target_dir = os.path.join(out_dir, "models", name)
    shutil.rmtree(target_dir, ignore_errors=True)
    shutil.copytree(os.path.join(AP_GAZEBO, "models", "iris_with_ardupilot"), target_dir)
    sdf_path = os.path.join(target_dir, "model.sdf")
    with open(sdf_path) as f:
        sdf = f.read()
    sdf = sdf.replace('<model name="iris_with_ardupilot">', f'<model name="{name}">')
    sdf = sdf.replace("<fdm_port_in>9002</fdm_port_in>", f"<fdm_port_in>{fdm_port}</fdm_port_in>")
    sdf = sdf.replace("<lock_step>1</lock_step>", "<lock_step>0</lock_step>")
    with open(sdf_path, "w") as f:
        f.write(sdf)


def generate(out_dir: str, positions: list, origin: tuple) -> str:
    """Write out_dir/swarm.sdf and its models; return the .sdf path.

    positions: (x east, y north) in metres of vehicle i = 0, 1, ...; origin: (lat, lon, alt).
    """
    includes = []
    for i, (x, y) in enumerate(positions):
        name = f"iris_{i}"
        write_model(out_dir, name, 9002 + 10 * i)
        includes.append(INCLUDE.format(name=name, x=x, y=y, z=SPAWN_Z_M))
    lat, lon, alt = origin
    path = os.path.join(out_dir, "swarm.sdf")
    with open(path, "w") as f:
        f.write(WORLD.format(lat=lat, lon=lon, alt=alt, includes="\n".join(includes)))
    return path
