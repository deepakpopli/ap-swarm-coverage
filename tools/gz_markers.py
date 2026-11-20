#!/usr/bin/env python3
"""Draw helper markers in a running Gazebo window so a zoomed-out view shows the mission.

Viewer only: it reads model poses from Gazebo and draws on the GUI. It sends nothing to the
swarm and changes nothing in the simulation.

Draws: the 300 m area outline, the four fixed lanes, and for each drone a large coloured ball
plus its camera footprint on the ground.

    GZ_PARTITION=swarm<runner pid> PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python \
        python3 tools/gz_markers.py
"""
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

from swarm import config, sensor

COLOURS = [(0.9, 0.3, 0.1), (0.1, 0.6, 0.3), (0.2, 0.4, 0.9), (0.8, 0.7, 0.1), (0.6, 0.2, 0.7)]
BALL_DIAMETER_M = 6.0      # the iris is 0.3 m wide: invisible from 300 m away without this
LINE_Z_M = 0.5             # lines drawn just above the ground plane
UPDATE_HZ = 5.0
QUIET_EXIT_S = 20.0         # no poses for this long: the simulation has ended
DRONE_NAME = re.compile(r"iris_\d+")

node = Node()
poses = {}                 # model name -> (x, y, z), filled by the subscriber


def on_poses(msg: Pose_V) -> None:
    for pose in msg.pose:
        if DRONE_NAME.fullmatch(pose.name):     # iris_0, iris_1, ...; not their nested links
            poses[pose.name] = (pose.position.x, pose.position.y, pose.position.z)


def send(marker: Marker) -> None:
    node.request("/marker", marker, Marker, Empty, 200)


def new_marker(ns: str, marker_id: int, kind: int, colour: tuple) -> Marker:
    """marker_id counts from 0 here; Gazebo reads id 0 as "create a new marker", so send id + 1."""
    m = Marker()
    m.ns, m.id, m.action, m.type = ns, marker_id + 1, Marker.ADD_MODIFY, kind
    for part in (m.material.ambient, m.material.diffuse, m.material.emissive):
        part.r, part.g, part.b, part.a = colour[0], colour[1], colour[2], 1.0
    return m


def line_strip(ns: str, marker_id: int, points: list, colour: tuple) -> None:
    m = new_marker(ns, marker_id, Marker.LINE_STRIP, colour)
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
    draw_ground(x0, x1, y0, y1)
    line_strip("area", 0, [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)], (0.1, 0.1, 0.1))
    across_m, along_m = sensor.lane_footprint(config.ALT_CRUISE_M)
    n_lanes = -(-int(x1 - x0) // int(across_m))            # ceil
    for k in range(n_lanes):
        x = x0 + (x1 - x0) * (k + 0.5) / n_lanes
        line_strip("lane", k, [(x, y0), (x, y1)], (0.55, 0.55, 0.55))


def draw_drone(index: int, x: float, y: float, z: float) -> None:
    colour = COLOURS[index % len(COLOURS)]
    ball = new_marker("drone", index, Marker.SPHERE, colour)
    ball.pose.position.x, ball.pose.position.y, ball.pose.position.z = x, y, z
    ball.scale.x = ball.scale.y = ball.scale.z = BALL_DIAMETER_M
    send(ball)
    across_m, along_m = sensor.footprint_size(max(z, 1.0))
    hx, hy = across_m / 2.0, along_m / 2.0
    corners = [(x - hx, y - hy), (x + hx, y - hy), (x + hx, y + hy), (x - hx, y + hy)]
    line_strip("footprint", index, corners + corners[:1], colour)


def remove_drone(index: int) -> None:
    for ns in ("drone", "footprint"):
        m = Marker()
        m.ns, m.id, m.action = ns, index + 1, Marker.DELETE_MARKER
        send(m)


def main() -> None:
    node.subscribe(Pose_V, "/world/swarm/dynamic_pose/info", on_poses)
    draw_static()
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


if __name__ == "__main__":
    main()
