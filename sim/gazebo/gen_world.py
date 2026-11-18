#!/usr/bin/env python3
"""Generate a headless-friendly Gazebo Harmonic world with N iris copters.

Each copter is a copy of ardupilot_gazebo's iris_with_ardupilot model with its
own ArduPilotPlugin fdm_port_in (9002 + 10*i), matching `arducopter -I i --model JSON`.
Models are spaced 10 m apart along Gazebo +X (east).
Usage: gen_world.py N OUT_DIR [--no-lock-step]
"""
import os, re, sys, shutil

AG = os.path.expanduser(os.environ.get("ARDUPILOT_GAZEBO", "~/ardupilot_gazebo"))

def main():
    n = int(sys.argv[1]); out = sys.argv[2]
    src = os.path.join(AG, "models", "iris_with_ardupilot")
    os.makedirs(os.path.join(out, "models"), exist_ok=True)
    includes = []
    for i in range(n):
        dst = os.path.join(out, "models", f"iris_{i}")
        shutil.rmtree(dst, ignore_errors=True); shutil.copytree(src, dst)
        p = os.path.join(dst, "model.sdf"); s = open(p).read()
        s = s.replace('<model name="iris_with_ardupilot">', f'<model name="iris_{i}">')
        s = re.sub(r"<fdm_port_in>\d+</fdm_port_in>", f"<fdm_port_in>{9002 + 10*i}</fdm_port_in>", s)
        if "--no-lock-step" in sys.argv:
            # lock_step=1 blocks the whole world while one autopilot is silent, so a
            # SIGKILLed SITL would freeze every other vehicle. Free-running avoids that.
            s = s.replace("<lock_step>1</lock_step>", "<lock_step>0</lock_step>")
        open(p, "w").write(s)
        c = os.path.join(dst, "model.config")
        if os.path.exists(c):
            txt = re.sub(r"<name>.*?</name>", f"<name>iris_{i}</name>", open(c).read(), count=1)
            open(c, "w").write(txt)
        includes.append(f'    <include><uri>model://iris_{i}</uri><name>iris_{i}</name>'
                        f'<pose degrees="true">{10*i} 0 0.195 0 0 90</pose></include>')
    world = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="swarm">
    <physics name="1ms" type="ignore"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <scene><ambient>1 1 1</ambient><background>0.8 0.8 0.8</background></scene>
    <spherical_coordinates><latitude_deg>-35.363262</latitude_deg><longitude_deg>149.165237</longitude_deg>
      <elevation>584</elevation><heading_deg>0</heading_deg><surface_model>EARTH_WGS84</surface_model></spherical_coordinates>
    <light type="directional" name="sun"><pose>0 0 10 0 0 0</pose><direction>-0.5 0.1 -0.9</direction></light>
    <model name="ground"><static>1</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>1000 1000</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>1000 1000</size></plane></geometry>
        <material><ambient>0.5 0.6 0.5 1</ambient><diffuse>0.5 0.6 0.5 1</diffuse></material></visual>
    </link></model>
{chr(10).join(includes)}
  </world>
</sdf>
"""
    open(os.path.join(out, "swarm.sdf"), "w").write(world)
    print(os.path.join(out, "swarm.sdf"))

if __name__ == "__main__":
    main()
