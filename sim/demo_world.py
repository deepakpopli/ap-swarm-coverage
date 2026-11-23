"""The live view's world extras (SWARM_DEMO=1, sim/gazebo_world.py, tools/show_gazebo.sh).
Off by default: without SWARM_DEMO the world and model files are the evaluation's.

- body_model(): a per-drone copy of iris_with_standoffs, DRONE_SCALE x the iris in the drone's
  colour (DRONE_COLORS: blue, dark green, black), with a faint blur disc on each prop.
- overlays_sdf(): one static model with visuals only (no collision, no inertial): a white
  ground visual, the yellow 300 x 300 m border, subtle lane lines and, with SWARM_DEMO_ZONE=1,
  the Mission B priority zone border (red). Part of the world file: it cannot lag.

SWARM_DEMO_PHYSICS=1 with SWARM_DEMO_SCALE=s: the drone really is s x the iris, so the props
you see are its rotors, turning as fast as the autopilot commands. Froude scaling (same shape,
same density, same g): lengths x s, mass x s^3, inertia x s^5, rotor speed x s^-1/2, every
time x s^1/2. The thrust to weight ratio and the hover throttle stay the iris's; the drone is
the iris in slow motion. body_model() scales the body, physical_plugins() the blades and
motors, physical_params() the autopilot. Without SWARM_DEMO_PHYSICS (or with scale 1) the
drones are the stock iris.
"""
import os
import shutil
import xml.etree.ElementTree as ET

from swarm import config

DRONE_SCALE = float(os.environ.get("SWARM_DEMO_SCALE", "1"))   # 1: the stock iris
PHYSICS = os.environ.get("SWARM_DEMO_PHYSICS") == "1"   # off: the physics stays the stock iris
PROP_RADIUS_M = 0.135     # iris prop blade tip (0.133 m): the blur disc is this x DRONE_SCALE
BLUR_TRANSPARENCY = 0.75  # a screen recording strobes the blades, the faint disc reads as spinning
# The stock iris puts the prop (rotor z 0.023 m, blade +-0.005 m) 2.5 mm into the motor bell
# (top at z 0.0205 m in iris.dae). Unseen at 1x, 1.5 cm into the motor at 6x: lift it onto the bell.
PROP_LIFT_M = 0.0205 + 0.005 - 0.023
# iris.dae has its motor shafts at x 0.1251 / -0.1383, y -0.2191 / 0.2225 front, 0.2061 / -0.2027
# rear: the mesh sits 6.6 mm aft and 1.7 mm left of the model origin, and the stock rotors
# (x +-0.13, y +-0.22 front, +-0.20 rear) are up to 10 mm off their shafts. The prop meshes have
# their hub off their origin too. Unseen at 1x, a prop beside its motor at 6x: centre the mesh,
# put each rotor on its shaft (symmetric about the origin, so the centre of mass stays the
# rotors' centre) and each prop's hub on its rotor axis.
BODY_MESH_SHIFT_M = (0.0066, -0.0017)
MOTOR_XY_M = {"rotor_0": (0.1317, -0.2208), "rotor_1": (-0.1317, 0.2044),
              "rotor_2": (0.1317, 0.2208), "rotor_3": (-0.1317, -0.2044)}
PROP_HUB_XY_M = {"iris_prop_ccw.dae": (-0.0044, -0.0004), "iris_prop_cw.dae": (-0.0007, 0.0001)}
# autopilot gains as (stock value, power of s). Angular acceleration per unit of motor output
# goes as 1/s and the loops must be t = s^1/2 slower: a rate P x t, its I x 1, its D x s; gains
# in 1/s and filter corners x 1/t; angular acceleration limits x 1/s; time constants x t.
# The stock values are the ArduCopter defaults (the yaw rate ones from copter.parm).
SCALED_GAINS = {
    "ATC_RAT_RLL_P": (0.135, 0.5), "ATC_RAT_RLL_I": (0.135, 0.0), "ATC_RAT_RLL_D": (0.0036, 1.0),
    "ATC_RAT_PIT_P": (0.135, 0.5), "ATC_RAT_PIT_I": (0.135, 0.0), "ATC_RAT_PIT_D": (0.0036, 1.0),
    "ATC_RAT_YAW_P": (0.3, 0.5), "ATC_RAT_YAW_I": (0.02, 0.0),
    "ATC_RAT_RLL_FLTD": (20.0, -0.5), "ATC_RAT_RLL_FLTT": (20.0, -0.5),
    "ATC_RAT_PIT_FLTD": (20.0, -0.5), "ATC_RAT_PIT_FLTT": (20.0, -0.5),
    "ATC_RAT_YAW_FLTT": (20.0, -0.5), "ATC_RAT_YAW_FLTE": (2.5, -0.5),
    "INS_GYRO_FILTER": (20.0, -0.5),
    "ATC_ANG_RLL_P": (4.5, -0.5), "ATC_ANG_PIT_P": (4.5, -0.5), "ATC_ANG_YAW_P": (4.5, -0.5),
    "ATC_ACC_R_MAX": (1100.0, -1.0), "ATC_ACC_P_MAX": (1100.0, -1.0),
    "ATC_ACC_Y_MAX": (270.0, -1.0),
    "ATC_INPUT_TC": (0.1, 0.5),
    "PSC_NE_POS_P": (1.0, -0.5), "PSC_NE_VEL_P": (2.0, -0.5), "PSC_NE_VEL_I": (1.0, -1.0),
    "PSC_D_POS_P": (1.0, -0.5), "PSC_D_VEL_P": (5.0, -0.5), "PSC_D_ACC_I": (0.1, -0.5),
}
# drone 1 blue, 2 dark green, 3 black on the white field; nothing else is coloured but the
# yellow border and the priority zone (red). tools/gz_markers.py draws each footprint in the same colour.
DRONE_COLORS = ["#1f5fd6", "#0b6b2a", "#111111", "#e06000", "#5a1a8a"]
BORDER_RGB = (1.0, 0.85, 0.0)           # yellow
ZONE_RGB = (1.0, 0.0, 0.0)              # red
LANE_RGB = (0.70, 0.70, 0.70)
GROUND_RGB = (0.97, 0.97, 0.97)         # white field, dark drones stand out
BORDER_W_M = 1.0
ZONE_W_M = 0.8
LANE_W_M = 0.2
MIDDLE_ZONE = (120.0, 120.0, 180.0, 180.0)   # run_mission.py MIDDLE_ZONE (Mission B default)

# Only rendering uses these (the window); the physics ignores them.
SUN_AND_SKY = """    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 100 0 0 0</pose>
      <diffuse>1.0 0.97 0.9 1</diffuse><specular>0.1 0.1 0.1 1</specular>
      <direction>-0.75 0.25 -0.6</direction>
    </light>
    <scene>
      <ambient>0.35 0.37 0.4 1</ambient><background>0.62 0.75 0.9 1</background>
      <sky></sky><shadows>true</shadows><grid>false</grid>
    </scene>"""


def physical() -> bool:
    """SWARM_DEMO_PHYSICS=1 and a scale: the demo drones are Froude scaled."""
    return PHYSICS and DRONE_SCALE != 1


def rgb(hex_colour: str) -> tuple:
    h = hex_colour.lstrip("#")
    return tuple(int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4))


def linear(colour: tuple) -> tuple:
    """A screen colour (sRGB, 0..1) as the material value that Gazebo shows as that colour: ogre2
    takes material colours as linear light, so #0a2a7a given as it is came out light blue
    (58, 120, 200 on screen). The drone bodies and their footprints go through this."""
    return tuple(v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4 for v in colour)


def _material(colour: tuple, emissive: float = 0.35, alpha: float = 1.0,
              specular: float = 0.1) -> str:
    r, g, b = colour
    e = " ".join(f"{v * emissive:.4f}" for v in colour)
    return (f"<material><ambient>{r:.4f} {g:.4f} {b:.4f} {alpha}</ambient>"
            f"<diffuse>{r:.4f} {g:.4f} {b:.4f} {alpha}</diffuse>"
            f"<specular>{specular:g} {specular:g} {specular:g} 1</specular>"
            f"<emissive>{e} 1</emissive></material>")


def _set_material(visual: ET.Element, colour: tuple) -> None:
    for old in visual.findall("material"):
        visual.remove(old)
    visual.append(ET.fromstring(_material(colour, specular=0.0)))   # no grey sheen on black


def _scale_pose(element: ET.Element, scale: float) -> None:
    pose = element.find("pose")
    if pose is None:
        return
    v = [float(s) for s in pose.text.split()]
    pose.text = " ".join(f"{x * scale:g}" for x in v[:3]) + " " + " ".join(f"{x:g}" for x in v[3:])


def _scale_geometry(geometry: ET.Element, scale: float) -> None:
    mesh, cylinder = geometry.find("mesh"), geometry.find("cylinder")
    if mesh is not None:
        for old in mesh.findall("scale"):
            mesh.remove(old)
        ET.SubElement(mesh, "scale").text = f"{scale:g} {scale:g} {scale:g}"
    if cylinder is not None:
        for tag in ("radius", "length"):
            cylinder.find(tag).text = f"{float(cylinder.find(tag).text) * scale:g}"


def _scale_text(element: ET.Element, tag: str, factor: float) -> None:
    found = element.find(tag)
    if found is not None:
        found.text = f"{float(found.text) * factor:.6g}"


def _scale_physics(model: ET.Element, scale: float) -> None:
    """Every link, collision and joint of the body s x larger, Froude scaled: mass x s^3,
    inertia x s^5, joint damping (torque per rad/s) x s^4.5."""
    for link in model.findall("link"):
        _scale_pose(link, scale)
        inertial = link.find("inertial")
        _scale_pose(inertial, scale)
        _scale_text(inertial, "mass", scale ** 3)
        for tag in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"):
            _scale_text(inertial, f"inertia/{tag}", scale ** 5)
        for collision in link.findall("collision"):
            _scale_pose(collision, scale)
            _scale_geometry(collision.find("geometry"), scale)
    for joint in model.findall("joint"):
        _scale_text(joint, "axis/dynamics/damping", scale ** 4.5)


def _blur_disc(link: ET.Element, colour: tuple, scale: float) -> None:
    blur = ET.SubElement(link, "visual", name="prop_blur")
    ET.SubElement(blur, "pose").text = f"0 0 {PROP_LIFT_M * scale:g} 0 0 0"   # with the blades
    disc = ET.SubElement(ET.SubElement(blur, "geometry"), "cylinder")
    ET.SubElement(disc, "radius").text = f"{PROP_RADIUS_M * scale:g}"
    ET.SubElement(disc, "length").text = f"{0.002 * scale:g}"
    _set_material(blur, colour)
    ET.SubElement(blur, "transparency").text = f"{BLUR_TRANSPARENCY:g}"


def body_model(sdf_text: str, colour_hex: str, scale: float = DRONE_SCALE) -> str:
    """iris_with_standoffs model.sdf text -> the same body scale x larger in one colour; the
    rotor links keep their props (they are what flies it), each with a blur disc."""
    root = ET.fromstring(sdf_text)
    model = root.find("model")
    colour = linear(rgb(colour_hex))
    base = model.find("link[@name='base_link']")
    for visual in base.findall("visual"):
        _scale_pose(visual, scale)
        _scale_geometry(visual.find("geometry"), scale)
        _set_material(visual, colour)
        if visual.findtext("geometry/mesh/uri", "").endswith("/iris.dae"):
            if visual.find("pose") is None:    # the stock mesh visual has none (0 0 0)
                ET.SubElement(visual, "pose")
            x, y = BODY_MESH_SHIFT_M
            visual.find("pose").text = f"{x * scale:g} {y * scale:g} 0 0 0 0"
    _scale_physics(model, scale)
    for link in model.findall("link"):
        if not link.get("name", "").startswith("rotor_"):
            continue
        pose = link.find("pose").text.split()
        pose[:2] = (f"{v * scale:g}" for v in MOTOR_XY_M[link.get("name")])
        link.find("pose").text = " ".join(pose)
        # the prop you see is the rotor: it turns as fast as the motor does
        prop = link.find("visual")
        hx, hy = PROP_HUB_XY_M[prop.findtext("geometry/mesh/uri").rsplit("/", 1)[-1]]
        prop.find("pose").text = f"{-hx * scale:g} {-hy * scale:g} {PROP_LIFT_M * scale:g} 0 0 0"
        _scale_geometry(prop.find("geometry"), scale)
        _set_material(prop, colour)
        _blur_disc(link, colour, scale)
    for visual in root.iter("visual"):        # no drone shadows on the field
        shadow = visual.find("cast_shadows")
        if shadow is None:
            shadow = ET.SubElement(visual, "cast_shadows")
        shadow.text = "false"
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode")


def physical_plugins(sdf_text: str, scale: float) -> str:
    """iris_with_ardupilot model.sdf text -> the same for the scale x drone. Each blade is a
    lift-drag surface, thrust ~ omega^2 * cp^2 * area, so with cp x s, area x s^2 and the top
    rotor speed (multiplier) x s^-1/2 the thrust goes as s^3, like the weight. The motor is a
    torque loop on the rotor joint: rotor inertia is x s^5 and its spin-up time x s^1/2, so the
    torque limits go as s^4 and the P gain (torque per rad/s) as s^4.5."""
    root = ET.fromstring(sdf_text)
    model = root.find("model")
    for plugin in model.findall("plugin"):
        if plugin.get("name") == "gz::sim::systems::LiftDrag":
            _scale_text(plugin, "area", scale ** 2)
            cp = plugin.find("cp")
            cp.text = " ".join(f"{float(v) * scale:g}" for v in cp.text.split())
        if plugin.get("name") == "ArduPilotPlugin":
            for control in plugin.findall("control"):
                for tag, power in (("multiplier", -0.5), ("p_gain", 4.5), ("i_gain", 4.0),
                                   ("d_gain", 5.0), ("cmd_max", 4.0), ("cmd_min", 4.0),
                                   ("i_max", 4.0), ("i_min", 4.0)):
                    _scale_text(control, tag, scale ** power)
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode")


def physical_params(scale: float) -> str:
    """An ArduPilot defaults file for the scale x drone: the stock gains slowed down by s^1/2
    (SCALED_GAINS). Hover throttle, lean angle and speeds need nothing: they do not scale."""
    lines = [f"# the stock iris tune for a {scale:g} x Froude-scaled iris (sim/demo_world.py)"]
    for name, (stock, power) in SCALED_GAINS.items():
        lines.append(f"{name} {stock * scale ** power:.6g}")
    return "\n".join(lines) + "\n"


def write_body(out_dir: str, ap_gazebo: str, name: str, index: int) -> str:
    """Write out_dir/models/<name>_body (iris_with_standoffs, the scaled coloured body) and
    return its model:// uri. The inner model name stays iris_with_standoffs, so the plugin's
    joint and link names (iris_with_standoffs::rotor_0 ...) are unchanged; the meshes still
    load from the installed model."""
    source = os.path.join(ap_gazebo, "models", "iris_with_standoffs")
    target = os.path.join(out_dir, "models", f"{name}_body")
    shutil.rmtree(target, ignore_errors=True)
    os.makedirs(target)
    shutil.copy(os.path.join(source, "model.config"), target)
    with open(os.path.join(source, "model.sdf")) as f:
        text = f.read()
    text = body_model(text, DRONE_COLORS[index % len(DRONE_COLORS)])
    with open(os.path.join(target, "model.sdf"), "w") as f:
        f.write(text)
    return f"model://{name}_body"


def _bar(name: str, x0: float, y0: float, x1: float, y1: float, width: float, z: float,
         colour: tuple, emissive: float = 0.6) -> str:
    """A flat box visual from (x0, y0) to (x1, y1), `width` wide (axis-aligned segments)."""
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    sx = abs(x1 - x0) + width if x1 != x0 else width
    sy = abs(y1 - y0) + width if y1 != y0 else width
    return (f'<visual name="{name}"><pose>{cx:.3f} {cy:.3f} {z:.3f} 0 0 0</pose>'
            f"<geometry><box><size>{sx:.3f} {sy:.3f} 0.05</size></box></geometry>"
            f"{_material(colour, emissive)}<cast_shadows>false</cast_shadows></visual>")


def _rectangle(prefix: str, x0, y0, x1, y1, width, z, colour) -> list:
    return [_bar(f"{prefix}_s", x0, y0, x1, y0, width, z, colour),
            _bar(f"{prefix}_n", x0, y1, x1, y1, width, z, colour),
            _bar(f"{prefix}_w", x0, y0, x0, y1, width, z, colour),
            _bar(f"{prefix}_e", x1, y0, x1, y1, width, z, colour)]


def overlays_sdf(lane_xs_m: list, zone: bool = False) -> str:
    """One static, visual-only model: ground visual, yellow border, lanes, optional priority zone."""
    xmin, xmax, ymin, ymax = config.AREA
    visuals = ['<visual name="ground"><pose>150 150 0.01 0 0 0</pose><geometry><plane>'
               "<normal>0 0 1</normal><size>4000 4000</size></plane></geometry>"
               f"{_material(GROUND_RGB, 0.0)}</visual>"]
    visuals += _rectangle("border", xmin, ymin, xmax, ymax, BORDER_W_M, 0.05, BORDER_RGB)
    for k, x in enumerate(lane_xs_m):
        visuals.append(_bar(f"lane_{k}", x, ymin, x, ymax, LANE_W_M, 0.03, LANE_RGB, 0.2))
    if zone:
        zx0, zy0, zx1, zy1 = MIDDLE_ZONE
        visuals += _rectangle("zone", zx0, zy0, zx1, zy1, ZONE_W_M, 0.07, ZONE_RGB)
    return ('    <model name="demo_overlays"><static>1</static><link name="link">\n      '
            + "\n      ".join(visuals) + "\n    </link></model>")
