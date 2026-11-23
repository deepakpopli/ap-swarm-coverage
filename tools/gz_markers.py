#!/usr/bin/env python3
"""Draw helper markers in a running Gazebo window so a zoomed-out view shows the mission.

Viewer only: it reads model poses from Gazebo and draws on the GUI. It sends nothing to the
swarm and changes nothing in the simulation.

Draws: the 300 m area outline, the four fixed lanes, and for each drone
its camera footprint on the ground and two thin dotted rings of RING_RADIUS_M around it, one
horizontal and one vertical (no solid ball: it hid the drone). With SWARM_DEMO=1 the world
already has the ground, the border and the lanes (sim/demo_world.py): only the drones' marks.

    GZ_PARTITION=swarm<runner pid> PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python \
        python3 tools/gz_markers.py [--no-rings]

--no-rings (or SWARM_MARKER_RINGS=0): no safety rings; they trail the drone in a live view.
SWARM_FOOTPRINT_W=<m>: each footprint edge a flat bar this wide (Gazebo lines are 1 px).
"""
import argparse
import math
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.marker_v_pb2 import Marker_V
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

from sim import demo_world
from swarm import config, sensor
from swarm.allocator import area_lanes

# exactly the colours of the drone bodies (sim/demo_world.py: blue, dark green, black)
COLOURS = [demo_world.linear(demo_world.rgb(h)) for h in demo_world.DRONE_COLORS]
DEMO = os.environ.get("SWARM_DEMO", "0") == "1"   # the world already has ground, border, lanes
FOOTPRINT_W_M = float(os.environ.get("SWARM_FOOTPRINT_W", "0"))   # > 0: footprint edges as bars
LINE_Z_M = 0.5             # lines drawn just above the ground
RING_RADIUS_M = 8.0         # dotted ring around each drone: the required separation
RING_DASHES = 24            # dashes per ring, each with a gap of the same length
UPDATE_HZ = 20.0           # boxes follow the drones smoothly
# no footprint below this height above the ground: on the pad the footprint is about 2 m wide,
# so its 1.5 m bars merged into a filled square on the border ("what are these boxes?")
FOOTPRINT_MIN_HEIGHT_M = 10.0
QUIET_EXIT_S = 20.0         # no poses for this long: the simulation has ended
DRONE_NAME = re.compile(r"iris_\d+")

node = Node()
poses = {}                 # model name -> (x, y, z), filled by the subscriber
footprints = set()         # indices of the drones whose footprint is drawn now


def on_poses(msg: Pose_V) -> None:
    for pose in msg.pose:
        if DRONE_NAME.fullmatch(pose.name):     # iris_0, iris_1, ...; not their nested links
            poses[pose.name] = (pose.position.x, pose.position.y, pose.position.z)


PENDING = []               # markers of one update, sent together in one request


def send(marker: Marker) -> None:
    PENDING.append(marker)


def flush() -> None:
    """All markers of this update in one /marker_array request (one per marker was too slow)."""
    if not PENDING:
        return
    batch = Marker_V()
    batch.marker.extend(PENDING)
    PENDING.clear()
    node.request("/marker_array", batch, Marker_V, Boolean, 200)


def new_marker(ns: str, marker_id: int, kind: int, colour: tuple, flat: bool = False) -> Marker:
    """marker_id counts from 0 here; Gazebo reads id 0 as "create a new marker", so send id + 1.
    flat: the colour only glows (ambient and diffuse black), so the sun and the sky do not
    lighten it: a drone's footprint and rings then show exactly the drone's colour."""
    m = Marker()
    m.ns, m.id, m.action, m.type = ns, marker_id + 1, Marker.ADD_MODIFY, kind
    for part in (m.material.ambient, m.material.diffuse, m.material.emissive):
        lit = 0.0 if flat and part is not m.material.emissive else 1.0
        part.r, part.g, part.b, part.a = colour[0] * lit, colour[1] * lit, colour[2] * lit, 1.0
    return m


def line_strip(ns: str, marker_id: int, points: list, colour: tuple, flat: bool = False) -> None:
    m = new_marker(ns, marker_id, Marker.LINE_STRIP, colour, flat)
    for x, y in points:
        p = m.point.add()
        p.x, p.y, p.z = x, y, LINE_Z_M
    send(m)


def clear_all() -> None:
    m = Marker()
    m.action = Marker.DELETE_ALL
    send(m)


def draw_ground(x0: float, x1: float, y0: float, y1: float) -> None:
    """A flat green slab over the search area. Drawn in the window only, not in the physics."""
    slab = new_marker("ground", 0, Marker.BOX, (0.42, 0.55, 0.36))
    slab.pose.position.x, slab.pose.position.y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    slab.pose.position.z = 0.05
    slab.scale.x, slab.scale.y, slab.scale.z = x1 - x0, y1 - y0, 0.1
    send(slab)


def draw_static() -> None:
    x0, x1, y0, y1 = config.AREA
    clear_all()
    if DEMO:
        return
    draw_ground(x0, x1, y0, y1)
    line_strip("area", 0, [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)], (0.1, 0.1, 0.1))
    for k, x in enumerate(area_lanes()):
        line_strip("lane", k, [(x, y0), (x, y1)], (0.55, 0.55, 0.55))


RINGS = os.environ.get("SWARM_MARKER_RINGS", "1") != "0"


def draw_drone(index: int, x: float, y: float, z: float) -> None:
    colour = COLOURS[index % len(COLOURS)]
    for k, plane in enumerate(("xy", "xz") if RINGS else ()):    # horizontal and vertical dotted ring
        ring = new_marker("ring", 2 * index + k, Marker.LINE_LIST, colour, flat=True)
        for j in range(2 * RING_DASHES):
            a = 2 * math.pi * j / (2 * RING_DASHES)
            p = ring.point.add()
            c, d = RING_RADIUS_M * math.cos(a), RING_RADIUS_M * math.sin(a)
            p.x, p.y, p.z = (x + c, y + d, z) if plane == "xy" else (x + c, y, z + d)
        send(ring)
    height_m = max(0.0, z)                     # flat ground at z = 0
    if height_m < FOOTPRINT_MIN_HEIGHT_M:      # on the pad, taking off or landing: no footprint
        if index in footprints:
            footprints.discard(index)
            remove_footprint(index)
        return
    footprints.add(index)
    across_m, along_m = sensor.footprint_size(height_m)
    hx, hy = across_m / 2.0, along_m / 2.0
    corners = [(x - hx, y - hy), (x + hx, y - hy), (x + hx, y + hy), (x - hx, y + hy)]
    if FOOTPRINT_W_M > 0:
        footprint_bars(index, x, y, hx, hy, colour)
    else:
        line_strip("footprint", index, corners + corners[:1], colour, flat=True)


def footprint_bars(index: int, x: float, y: float, hx: float, hy: float, colour: tuple) -> None:
    """The footprint as four flat bars FOOTPRINT_W_M wide (lines are 1 px in Gazebo)."""
    w = FOOTPRINT_W_M
    edges = [(x, y - hy, 2 * hx + w, w), (x, y + hy, 2 * hx + w, w),
             (x - hx, y, w, 2 * hy + w), (x + hx, y, w, 2 * hy + w)]
    for k, (cx, cy, sx, sy) in enumerate(edges):
        bar = new_marker("fbar", 4 * index + k, Marker.BOX, colour, flat=True)
        bar.pose.position.x, bar.pose.position.y, bar.pose.position.z = cx, cy, 0.3
        bar.scale.x, bar.scale.y, bar.scale.z = sx, sy, 0.2
        send(bar)


def remove_markers(ids: list) -> None:
    for ns, marker_id in ids:
        m = Marker()
        m.ns, m.id, m.action = ns, marker_id + 1, Marker.DELETE_MARKER
        send(m)


def remove_footprint(index: int) -> None:
    remove_markers([("footprint", index)] + [("fbar", 4 * index + k) for k in range(4)])


def remove_drone(index: int) -> None:
    footprints.discard(index)
    remove_footprint(index)
    remove_markers([("ring", 2 * index), ("ring", 2 * index + 1)])


def main() -> None:
    global RINGS
    parser = argparse.ArgumentParser(description="helper markers in a running Gazebo window")
    parser.add_argument("--no-rings", action="store_true", help="no 8 m safety rings")
    if parser.parse_args().no_rings:
        RINGS = False
    node.subscribe(Pose_V, "/world/swarm/dynamic_pose/info", on_poses)
    draw_static()
    flush()
    shown = set()
    last_pose_time = time.time()
    while time.time() - last_pose_time < QUIET_EXIT_S:
        poses.clear()
        time.sleep(1.0 / UPDATE_HZ)
        if poses:
            last_pose_time = time.time()
        for name, (x, y, z) in sorted(poses.items()):
            draw_drone(int(name.split("_")[1]), x, y, z)
        current = {int(name.split("_")[1]) for name in poses}
        for index in shown - current:      # a killed drone's model is removed from the world
            remove_drone(index)
        shown = current or shown
        flush()


if __name__ == "__main__":
    main()
