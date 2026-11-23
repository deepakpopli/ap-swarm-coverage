"""Ground footprint of the SIYI A8 mini camera, pointing straight down.

A rectangle on flat ground: the wide side (horizontal field of view) lies across the track and
the rectangle turns with the logged yaw.
"""
import math

import numpy as np

# https://siyi.biz/en/product/tri-axis-single-camera-gimbal/a8-mini/spec/
HFOV_DEG = 81.0                       # horizontal field of view
# https://res.siyi.biz/oss/other/2026/06/15/A8_mini_User_Manual_v1_10_563cde30.pdf (p.15)
IMAGE_WIDTH_OVER_HEIGHT = 16.0 / 9.0  # every recording mode is 16:9

TAN_HALF_ACROSS = math.tan(math.radians(HFOV_DEG) / 2.0)
TAN_HALF_ALONG = TAN_HALF_ACROSS / IMAGE_WIDTH_OVER_HEIGHT   # vertical FOV 51.3 deg, derived


def footprint_size(alt_m):
    """(across_m, along_m) of the footprint at alt_m above flat ground. 50 m: 85.4 x 48.0 m."""
    return (2.0 * alt_m * TAN_HALF_ACROSS, 2.0 * alt_m * TAN_HALF_ALONG)


def lane_footprint(alt_m):
    """footprint_size rounded down to whole metres, for the allocator.

    Rounding down puts lanes slightly closer together, so the plan never relies on ground
    the camera does not quite see. 50 m: (85, 48).
    """
    across_m, along_m = footprint_size(alt_m)
    return (float(math.floor(across_m)), float(math.floor(along_m)))


def heading_axes(yaw_rad):
    """Unit vectors (east, north) along the heading and across it to the right.

    Yaw is a compass angle: 0 = north, clockwise positive.
    """
    along = (math.sin(yaw_rad), math.cos(yaw_rad))
    across = (math.cos(yaw_rad), -math.sin(yaw_rad))
    return along, across


def footprint_polygon(x, y, alt_m, yaw_rad):
    """The 4 ground corners [(x, y), ...] of the footprint, for drawing."""
    across_m, along_m = footprint_size(alt_m)
    (fx, fy), (rx, ry) = heading_axes(yaw_rad)
    corners = []
    for front, right in ((1, -1), (-1, -1), (-1, 1), (1, 1)):
        a = front * along_m / 2.0
        c = right * across_m / 2.0
        corners.append((x + a * fx + c * rx, y + a * fy + c * ry))
    return corners


def seen_mask(points_x, points_y, x, y, alt_m, yaw_rad):
    """True for each ground point (arrays, metres) inside the footprint of a camera at x, y."""
    across_m, along_m = footprint_size(alt_m)
    (fx, fy), (rx, ry) = heading_axes(yaw_rad)
    dx = points_x - x
    dy = points_y - y
    along_offset = dx * fx + dy * fy
    across_offset = dx * rx + dy * ry
    return (np.abs(along_offset) <= along_m / 2.0) & (np.abs(across_offset) <= across_m / 2.0)
