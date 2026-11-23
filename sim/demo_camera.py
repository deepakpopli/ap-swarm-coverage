"""A simulated downward camera on each live-view drone (SWARM_DEMO=1 with SWARM_DEMO_CAMERA=1).

Off by default: without both the world and model files are the evaluation's.

The camera matches swarm/sensor.py: nadir, 81 deg HFOV, 16:9 (640 x 360), the wide side across
the track. A Gazebo camera looks along its +x with image up = its +z; pitched 90 deg down it
looks along the drone's -z with image up = the drone's +x (the nose) and image right = the
drone's right. Drone N (iris_<N-1>) publishes on /swarm/cam/N; tools/cam_bridge.py streams and
saves the frames.
"""
import math
import os

from swarm import sensor

CAMERA = os.environ.get("SWARM_DEMO_CAMERA") == "1"
WIDTH, HEIGHT = 640, 360
RATE_HZ = 10
TOPIC = "/swarm/cam/{n}"

SENSORS_PLUGIN = ('    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
                  '<render_engine>ogre2</render_engine></plugin>')

# a tiny link on a fixed joint to the body; near clip past the body so it never sees itself
CAMERA_LINK = """    <link name="cam_link">
      <inertial><mass>0.001</mass>
        <inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <sensor name="nadir_cam" type="camera">
        <pose>0 0 0 0 {pitch:.6f} 0</pose>
        <always_on>1</always_on><update_rate>{rate}</update_rate><topic>{topic}</topic>
        <camera>
          <horizontal_fov>{hfov:.6f}</horizontal_fov>
          <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
          <clip><near>{near:g}</near><far>3000</far></clip>
        </camera>
      </sensor>
    </link>
    <joint name="cam_joint" type="fixed">
      <parent>iris_with_standoffs::base_link</parent><child>cam_link</child>
    </joint>
"""


def topic(n: int) -> str:
    return TOPIC.format(n=n)


def add_camera(model_sdf: str, n: int, scale: float = 1.0) -> str:
    """iris model.sdf text -> the same with the nadir camera of drone n (1, 2, ...)."""
    link = CAMERA_LINK.format(pitch=math.pi / 2, rate=RATE_HZ, topic=topic(n),
                              hfov=math.radians(sensor.HFOV_DEG), w=WIDTH, h=HEIGHT,
                              near=0.3 * scale)
    head, tail = model_sdf.rsplit("</model>", 1)
    return head + link + "  </model>" + tail


def add_sensors_plugin(world_sdf: str) -> str:
    """The world with the Sensors system (it renders the cameras), once."""
    if "gz::sim::systems::Sensors" in world_sdf:
        return world_sdf
    imu = '<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>\n'
    return world_sdf.replace(imu, imu + SENSORS_PLUGIN + "\n", 1)


def compass_pose(x: float, y: float, z: float, qw: float, qx: float, qy: float, qz: float):
    """A Gazebo world pose (ENU, = the area frame) -> (x, y, z, yaw) with yaw as in the agent
    csv: 0 = north, clockwise, radians in [-pi, pi)."""
    enu_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    yaw = (math.pi / 2.0 - enu_yaw + math.pi) % (2.0 * math.pi) - math.pi
    return x, y, z, yaw
