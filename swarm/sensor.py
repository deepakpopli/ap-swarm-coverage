"""Sensor footprint model: SIYI A8 mini gimbal camera, pointing straight down (nadir).

Sources (checked 2026-11-19):
  [S1] SIYI official spec page
       https://siyi.biz/en/product/tri-axis-single-camera-gimbal/a8-mini/spec/
       "Equivalent focal length 21 mm", "Sony 1/1.7-inch, 8 MP effective resolution",
       "FOV: Diagonal: 93°, Horizontal: 81°",
       "4K (4096 × 2160) @ 25 fps", "2K (2560 × 1440) @ 25 fps", "1080p (1920 × 1080) @ 25 fps",
       "720p (1280 × 720) @ 25 fps", "Weight: 95 g", "Angular vibration range ± 0.01°".
  [S2] SIYI A8 mini User Manual v1.10 (official PDF)
       https://res.siyi.biz/oss/other/2026/06/15/A8_mini_User_Manual_v1_10_563cde30.pdf
       p.15: "FOV  Diagonal: 93°  Horizontal: 81°", "4K (3840 x 2160) @ 25 fps" (other modes as S1),
       "Still Photo Resolution 4K (3840 x 2160) Follow video recording resolution",
       "Weight 95 g", "Zoom is not supported in 4K mode."
  [S3] SIYI official A8 mini FAQ
       https://siyi.biz/en/product/tri-axis-single-camera-gimbal/a8-mini/faq/
       Q6: "DFOV: 93°, HFOV: 81°, VFOV: 65°"; Q9: HDMI outputs "1080P".

Where official sources disagree:
  * 4K width: S1 says 4096 x 2160, S2 (manual) says 3840 x 2160. We use 3840 x 2160 (the manual,
    which also states the photo resolution follows it). Both are 2160 rows; the GSD differs by 6 %.
  * Vertical FOV: S3 says 65°. But 81°/65°/93° is the geometry of a 4:3 rectilinear frame
    (tan 40.5° x 3/4 -> VFOV 65.3°, diagonal 93.7°), while every recording mode is 16:9.
    We therefore DERIVE the VFOV of the 16:9 recorded/streamed frame from the HFOV and the
    16:9 aspect ratio, assuming a rectilinear (pinhole) lens and that 81° spans the full frame width:
        VFOV = 2 * atan(tan(HFOV / 2) * 9 / 16) = 51.3°
    This is the smaller (conservative) of the two choices: it never over-claims coverage.
    If the 65° figure is right for the recorded frame, the along-track footprint is 27 % longer
    (76.5 m instead of 57.7 m at 60 m); across-track (the axis that matters for lane spacing)
    is identical. Lens distortion is not modelled (no official figure).

Model
  * Nadir pointing: the 3-axis gimbal (+-0.01° stabilisation, S1) holds the camera straight down,
    so the footprint does not tilt with vehicle roll/pitch.
  * Flat ground at z = 0 (the area origin height); `alt` is the logged z (m above the origin).
  * Rectangular footprint, wide axis (HFOV) ACROSS track, i.e. perpendicular to the vehicle's
    heading. With yaw = 0 (north) the wide axis runs east-west. The rectangle rotates with yaw.
  * Yaw is compass convention: radians, 0 = north, clockwise positive (as in the agent logs).

Footprint (across x along, metres):  50 m: 85.4 x 48.0   60 m: 102.5 x 57.7   70 m: 119.6 x 67.3
(The allocator's 102 m x 57 m at 60 m agrees with this to within 1 m.)

Range (max useful altitude)
  Requirement used: a person seen from above (~0.5 m shoulder width) must span >= 10 pixels,
  i.e. ground sample distance (GSD) <= 5 cm/pixel. GSD = across_m / image_width_px.
    at 60 m: 4K recording (3840 px): 2.67 cm/px (18.7 px on a person)
             1080p live stream/HDMI (1920 px): 5.34 cm/px (9.4 px on a person)
    max altitude for 5 cm/px: 112 m on the 4K recording, 56 m on the 1080p live stream.
  So the 50-70 m band meets the requirement on the onboard recording; the 1080p downlink is
  marginal (just above 5 cm/px) at 60 m. MAX_USEFUL_ALT_M below uses the 4K recording.

Update rate
  The camera records at 25 fps (all modes, S1/S2). The coverage metric does NOT sample at 25 Hz:
  it uses the agents' 10 Hz state log (config.CONTROL_HZ), one footprint per log row. At 10 m/s
  the vehicle moves 1 m between rows, far less than the 48 m along-track footprint, so the
  10 Hz sampling misses no ground.
"""
import math

import numpy as np

# ---- verified product figures (see docstring for sources)
PRODUCT = "SIYI A8 mini"
HFOV_DEG = 81.0            # [S1][S2][S3]
DFOV_DEG = 93.0            # [S1][S2][S3] (not used by the model; kept for reference)
VFOV_FAQ_DEG = 65.0        # [S3] (not used: describes a 4:3 frame, see docstring)
IMAGE_W_PX = 3840          # 4K recording [S2]
IMAGE_H_PX = 2160
STREAM_W_PX = 1920         # HDMI / live 1080p [S3 Q9]
FRAME_RATE_HZ = 25.0       # [S1][S2]
WEIGHT_G = 95.0            # [S1][S2]

# ---- derived
ASPECT = IMAGE_W_PX / IMAGE_H_PX                     # 16:9
TAN_H = math.tan(math.radians(HFOV_DEG) / 2.0)
TAN_V = TAN_H / ASPECT
VFOV_DEG = math.degrees(2.0 * math.atan(TAN_V))      # 51.3 deg

# ---- range requirement
TARGET_SIZE_M = 0.5        # person, seen from above
TARGET_MIN_PX = 10         # pixels across that target
GSD_REQ_M = TARGET_SIZE_M / TARGET_MIN_PX            # 0.05 m/px
MAX_USEFUL_ALT_M = GSD_REQ_M * IMAGE_W_PX / (2.0 * TAN_H)   # 112 m

# ---- the rate the coverage metric samples at
METRIC_SAMPLE_HZ = 10.0    # = config.CONTROL_HZ, the agent log rate


def footprint_size(alt):
    """Ground footprint (across_m, along_m) at altitude `alt` metres above flat ground.

    across = 2 * alt * tan(HFOV/2), along = 2 * alt * tan(VFOV/2). Zero for alt <= 0.
    """
    a = max(float(alt), 0.0)
    return (2.0 * a * TAN_H, 2.0 * a * TAN_V)


def lane_footprint(alt):
    """Footprint (across_m, along_m) the allocator plans lanes with: footprint_size(alt) rounded
    DOWN to whole metres (conservative: lanes a little closer, ends a little nearer the edges).
    60 m -> (102, 57), 50 m -> (85, 48)."""
    w, l = footprint_size(alt)
    return (float(math.floor(w + 1e-9)), float(math.floor(l + 1e-9)))


def gsd(alt, width_px=IMAGE_W_PX):
    """Ground sample distance, metres per pixel, across track."""
    return footprint_size(alt)[0] / width_px


def _axes(yaw):
    """Unit vectors (east, north) of the heading (along) and right-hand (across) axes."""
    s, c = math.sin(yaw), math.cos(yaw)
    along = (s, c)        # compass yaw: 0 -> north (0, 1), pi/2 -> east (1, 0)
    across = (c, -s)      # 90 deg clockwise of the heading
    return along, across


def footprint_polygon(x, y, alt, yaw):
    """The 4 ground corners [(x, y), ...] of the footprint, counter-clockwise from front-left.

    x, y: camera position (area ENU metres); alt: metres; yaw: rad, compass (0 north, CW).
    """
    w, l = footprint_size(alt)
    (fx, fy), (rx, ry) = _axes(yaw)
    hw, hl = w / 2.0, l / 2.0
    corners = []
    for sa, sr in ((1, -1), (-1, -1), (-1, 1), (1, 1)):   # front-left, back-left, back-right, front-right
        corners.append((x + sa * hl * fx + sr * hw * rx, y + sa * hl * fy + sr * hw * ry))
    return corners


def seen_mask(grid_x, grid_y, x, y, alt, yaw):
    """Boolean array, True where the point (grid_x, grid_y) lies inside the footprint.

    grid_x, grid_y: arrays of the same shape (e.g. cell centres from np.meshgrid), metres.
    A point is seen when its along-track and across-track offsets from (x, y) are within half
    the footprint length / width (boundary inclusive). Altitudes above MAX_USEFUL_ALT_M see
    nothing usable (GSD requirement not met).
    """
    gx = np.asarray(grid_x, dtype=float)
    gy = np.asarray(grid_y, dtype=float)
    if alt <= 0.0 or alt > MAX_USEFUL_ALT_M:
        return np.zeros(np.broadcast(gx, gy).shape, dtype=bool)
    w, l = footprint_size(alt)
    (fx, fy), (rx, ry) = _axes(yaw)
    dx, dy = gx - x, gy - y
    d_along = dx * fx + dy * fy
    d_across = dx * rx + dy * ry
    return (np.abs(d_along) <= l / 2.0) & (np.abs(d_across) <= w / 2.0)


def bounding_half_extent(alt, yaw):
    """Half-size (hx, hy) in metres of the axis-aligned box enclosing the rotated footprint."""
    w, l = footprint_size(alt)
    (fx, fy), (rx, ry) = _axes(yaw)
    hx = abs(fx) * l / 2.0 + abs(rx) * w / 2.0
    hy = abs(fy) * l / 2.0 + abs(ry) * w / 2.0
    return hx, hy


if __name__ == "__main__":
    print(f"{PRODUCT}: HFOV {HFOV_DEG:.1f} deg, VFOV(16:9, derived) {VFOV_DEG:.1f} deg")
    for h in (50.0, 60.0, 70.0):
        a, b = footprint_size(h)
        print(f"  {h:4.0f} m: {a:6.1f} m across x {b:5.1f} m along, area {a*b:7.0f} m^2, "
              f"GSD 4K {gsd(h)*100:.2f} cm/px, 1080p {gsd(h, STREAM_W_PX)*100:.2f} cm/px")
    print(f"  max useful altitude (GSD <= {GSD_REQ_M*100:.0f} cm/px, 4K): {MAX_USEFUL_ALT_M:.0f} m")
