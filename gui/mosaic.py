"""Photo mosaic of a run: each camera frame (cam/frames.jsonl) laid on its ground footprint,
later frames over earlier ones, north up. Built incrementally as the browser's time moves.

The camera is fixed to the body, so it looks straight down only in steady flight: while the
drone speeds up, brakes or turns it leans by atan(a / g) and the picture slides z * a / g on
the ground (26 m at 5 m/s^2 from 50 m). frames.jsonl has no roll or pitch, so the lean comes
from the horizontal acceleration in the drone's agent_N.csv (vx, vy), and the frame is laid
down through that tilted view. A frame that leans more than MAX_TILT_DEG is left out.
"""
import bisect
import csv
import io
import json
import math
import os
import time

from PIL import Image

from swarm import sensor

RES_M = 0.5                      # metres per mosaic pixel
EXTENT_M = (-10.0, 310.0)        # x and y range of the mosaic, metres (the field and a margin)
SIZE_PX = int(round((EXTENT_M[1] - EXTENT_M[0]) / RES_M))
BACKGROUND = (247, 247, 247)     # never photographed: the map's ground colour
SNAPSHOT_EVERY = 60              # frames between saved canvases, for seeking back
JPEG_QUALITY = 80
G_MS2 = 9.81
ACCEL_HALF_S = 0.3               # acceleration = the change of velocity over +- this
MAX_TILT_DEG = 25.0              # leaning more than this: stretched and smeared, left out
FEATHER = 0.06                   # the outer part of a frame that fades into what is under it
WAIT_S = 3.0                     # live: a new frame waits this long at most for its csv rows


def footprint_m(frame):
    """(across_m, along_m) on the ground: as sensor.footprint_size, from the frame's own fov."""
    hfov = math.radians(frame.get("hfov_deg", sensor.HFOV_DEG))
    across = 2.0 * frame["z"] * math.tan(hfov / 2.0)
    return across, across * frame.get("h", 9) / frame.get("w", 16)


def tilt_deg(frame):
    """How far the camera leans off straight down, from the frame's acceleration (ax, ay)."""
    return math.degrees(math.atan(math.hypot(frame.get("ax", 0.0), frame.get("ay", 0.0)) / G_MS2))


def camera_axes(frame):
    """Unit vectors (east, north, up) of the camera: image up (the nose), image right, the view.

    Level they are sensor.heading_axes and straight down. Thrust points along the body's up, so
    with horizontal acceleration (ax, ay) the body's up is along (ax, ay, g); the nose keeps
    the logged heading.
    """
    (fx, fy), _ = sensor.heading_axes(frame["yaw"])
    ax, ay = frame.get("ax", 0.0), frame.get("ay", 0.0)
    norm = math.sqrt(ax * ax + ay * ay + G_MS2 * G_MS2)
    up = (ax / norm, ay / norm, G_MS2 / norm)
    fz = -(fx * up[0] + fy * up[1]) / up[2]          # the nose, square to the body's up
    fn = math.sqrt(1.0 + fz * fz)
    nose = (fx / fn, fy / fn, fz / fn)
    view = (-up[0], -up[1], -up[2])
    right = (view[1] * nose[2] - view[2] * nose[1], view[2] * nose[0] - view[0] * nose[2],
             view[0] * nose[1] - view[1] * nose[0])
    return nose, right, view


def projection(frame, img_w, img_h):
    """Rows (cu, cv, cw) with u = cu . p / cw . p and v = cv . p / cw . p for p = (dx, dy, 1),
    the ground point's offset from the drone in metres."""
    nose, right, view = camera_axes(frame)
    tan_across = math.tan(math.radians(frame.get("hfov_deg", sensor.HFOV_DEG)) / 2.0)
    tan_along = tan_across * frame.get("h", 9) / frame.get("w", 16)
    z = frame["z"]
    row = lambda axis: (axis[0], axis[1], -z * axis[2])     # axis . (dx, dy, -z)
    cw, r, n = row(view), row(right), row(nose)
    cu = tuple(img_w * (0.5 * w + a / (2.0 * tan_across)) for w, a in zip(cw, r))
    cv = tuple(img_h * (0.5 * w - a / (2.0 * tan_along)) for w, a in zip(cw, n))
    return cu, cv, cw


def ground_to_image(x, y, frame, img_w, img_h):
    """Image pixel (u, v) that sees ground point (x, y): top = nose, right = right of the nose."""
    cu, cv, cw = projection(frame, img_w, img_h)
    p = (x - frame["x"], y - frame["y"], 1.0)
    dot = lambda c: c[0] * p[0] + c[1] * p[1] + c[2] * p[2]
    return dot(cu) / dot(cw), dot(cv) / dot(cw)


def perspective(frame, img_w, img_h, ox, oy):
    """PIL PERSPECTIVE coefficients: mosaic pixel (ox + i, oy + j) -> image pixel (u, v)."""
    dx0 = EXTENT_M[0] + ox * RES_M - frame["x"]
    dy0 = EXTENT_M[1] - oy * RES_M - frame["y"]
    # p = (dx0 + i * RES_M, dy0 - j * RES_M, 1): each row c . p as (per i, per j, constant)
    rows = [(c[0] * RES_M, -c[1] * RES_M, c[0] * dx0 + c[1] * dy0 + c[2])
            for c in projection(frame, img_w, img_h)]
    k = rows[2][2]
    (a, b, c), (d, e, f), (g, h, _) = [tuple(value / k for value in row) for row in rows]
    return (a, b, c, d, e, f, g, h)


FEATHER_MASKS = {}


def feather_mask(size):
    """Opaque in the middle, fading to clear over the outer FEATHER of the frame's short side."""
    if size not in FEATHER_MASKS:
        w, h = size
        edge = max(1, int(round(FEATHER * min(w, h))))
        mask = Image.new("L", size, 0)
        for i in range(edge):
            mask.paste(int(round(255 * (i + 1) / edge)), (i, i, w - i, h - i))
        FEATHER_MASKS[size] = mask
    return FEATHER_MASKS[size]


def paste_frame(canvas, frame, image):
    """Warp one frame onto the canvas (an RGB Image of SIZE_PX), inside its footprint only."""
    if frame["z"] <= 5.0:          # parked or taking off: the ground is inside the near clip
        return
    if tilt_deg(frame) > MAX_TILT_DEG:
        return
    corners = sensor_corners(frame)
    cols = [(x - EXTENT_M[0]) / RES_M for x, _ in corners]
    rows = [(EXTENT_M[1] - y) / RES_M for _, y in corners]
    x0, y0 = max(0, math.floor(min(cols))), max(0, math.floor(min(rows)))
    x1, y1 = min(SIZE_PX, math.ceil(max(cols))), min(SIZE_PX, math.ceil(max(rows)))
    if x1 <= x0 or y1 <= y0:
        return
    coeffs = perspective(frame, image.width, image.height, x0, y0)
    size = (x1 - x0, y1 - y0)
    warped = image.transform(size, Image.PERSPECTIVE, coeffs, Image.BILINEAR)
    mask = feather_mask(image.size).transform(size, Image.PERSPECTIVE, coeffs, Image.BILINEAR,
                                              fillcolor=0)
    canvas.paste(warped, (x0, y0), mask)


def sensor_corners(frame):
    """Where the 4 image corners meet the ground (the sensor footprint when level)."""
    nose, right, view = camera_axes(frame)
    tan_across = math.tan(math.radians(frame.get("hfov_deg", sensor.HFOV_DEG)) / 2.0)
    tan_along = tan_across * frame.get("h", 9) / frame.get("w", 16)
    corners = []
    for f, c in ((1, -1), (-1, -1), (-1, 1), (1, 1)):
        ray = [view[i] + f * tan_along * nose[i] + c * tan_across * right[i] for i in range(3)]
        reach = frame["z"] / -ray[2]         # MAX_TILT_DEG keeps every corner ray going down
        corners.append((frame["x"] + reach * ray[0], frame["y"] + reach * ray[1]))
    return corners


class Motion:
    """A drone's horizontal velocity over time, from the growing agent_N.csv next to cam/."""

    def __init__(self, path):
        self.path = path
        self.offset = 0
        self.columns = None
        self.t, self.vx, self.vy = [], [], []

    def read(self):
        try:
            with open(self.path, "rb") as f:
                f.seek(self.offset)
                data = f.read()
        except OSError:
            return
        end = data.rfind(b"\n") + 1
        self.offset += end
        for row in csv.reader(data[:end].decode(errors="replace").splitlines()):
            if self.columns is None:
                try:
                    self.columns = [row.index(name) for name in ("t_wall", "vx", "vy")]
                except ValueError:
                    self.columns = ()        # not an agent csv: no velocities
                continue
            try:
                t, vx, vy = (float(row[i]) for i in self.columns)
            except (ValueError, IndexError):
                continue
            if not self.t or t > self.t[-1]:
                self.t.append(t), self.vx.append(vx), self.vy.append(vy)

    def velocity(self, t):
        i = bisect.bisect(self.t, t)
        if i == 0 or i == len(self.t):
            j = min(i, len(self.t) - 1)
            return self.vx[j], self.vy[j]
        w = (t - self.t[i - 1]) / (self.t[i] - self.t[i - 1])
        return (self.vx[i - 1] + w * (self.vx[i] - self.vx[i - 1]),
                self.vy[i - 1] + w * (self.vy[i] - self.vy[i - 1]))

    def accel(self, t):
        """(ax, ay) at t, m/s^2; None before the csv has reached t + ACCEL_HALF_S."""
        if not self.t or self.t[-1] < t + ACCEL_HALF_S:
            return None
        (ax0, ay0), (ax1, ay1) = self.velocity(t - ACCEL_HALF_S), self.velocity(t + ACCEL_HALF_S)
        return (ax1 - ax0) / (2 * ACCEL_HALF_S), (ay1 - ay0) / (2 * ACCEL_HALF_S)


class Mosaic:
    """Frames sorted by time; the canvas holds frames[:applied]."""

    def __init__(self, cam_dir, skip=lambda frame: False):
        self.cam_dir = cam_dir
        self.skip = skip             # frames to leave out (a dead drone's camera)
        self.offset = 0              # bytes of frames.jsonl read
        self.keys = []               # sorted (t_wall, line number)
        self.frames = []             # the frame dicts, in the same order
        self.motions = {}            # drone -> Motion
        self.snapshots = [(0, Image.new("RGB", (SIZE_PX, SIZE_PX), BACKGROUND))]
        self.rewind(0)

    def exists(self):
        return os.path.exists(os.path.join(self.cam_dir, "frames.jsonl"))

    def rewind(self, n):
        """Back to the newest saved canvas with at most n frames."""
        self.snapshots = [s for s in self.snapshots if s[0] <= n] or self.snapshots[:1]
        self.applied, canvas = self.snapshots[-1]
        self.canvas = canvas.copy()
        self.encoded = None

    def read(self):
        path = os.path.join(self.cam_dir, "frames.jsonl")
        if not os.path.exists(path):
            return
        with open(path, "rb") as f:
            f.seek(self.offset)
            data = f.read()
        end = data.rfind(b"\n") + 1          # a half-written last line waits
        self.offset += end
        for line in data[:end].decode(errors="replace").splitlines():
            try:
                frame = json.loads(line)
                key = (float(frame["t_wall"]), len(self.keys))
            except (ValueError, KeyError, TypeError):
                continue
            if self.skip(frame):
                continue
            i = bisect.bisect(self.keys, key)
            self.keys.insert(i, key)
            self.frames.insert(i, frame)
            if i < self.applied:             # late for a time already drawn: paint again
                self.rewind(i)

    def advance(self, t_wall):
        """The canvas with every frame up to t_wall; seeking back restores a saved canvas."""
        n = bisect.bisect(self.keys, (t_wall, math.inf))
        if n < self.applied:
            self.rewind(n)
        while self.applied < n:
            if not self.lean(self.frames[self.applied]):
                break                        # live: its csv rows are not written yet
            self.paint(self.frames[self.applied])
            self.applied += 1
            self.encoded = None
            if self.applied % SNAPSHOT_EVERY == 0 and self.applied > self.snapshots[-1][0]:
                self.snapshots.append((self.applied, self.canvas.copy()))

    def lean(self, frame):
        """Give the frame its acceleration (ax, ay); False while a new frame's is still to come."""
        if "ax" in frame:
            return True
        drone = frame.get("drone")
        if drone not in self.motions:
            self.motions[drone] = Motion(os.path.join(os.path.dirname(self.cam_dir),
                                                      f"agent_{drone}.csv"))
        motion = self.motions[drone]
        accel = motion.accel(frame["t_wall"])
        if accel is None:
            motion.read()
            accel = motion.accel(frame["t_wall"])
        if accel is None:
            if motion.t and time.time() - frame["t_wall"] < WAIT_S:
                return False
            accel = (0.0, 0.0)               # no csv, or it ended before the frame: level
        frame["ax"], frame["ay"] = accel
        return True

    def paint(self, frame):
        try:
            with Image.open(os.path.join(self.cam_dir, os.path.basename(frame["file"]))) as im:
                im.draft("RGB", (im.width // 2, im.height // 2))   # 0.5 m/px needs no more
                image = im.convert("RGB")
            paste_frame(self.canvas, frame, image)
        except (OSError, KeyError, ValueError):
            pass                             # a missing or broken frame leaves a gap

    def jpeg(self, t_wall):
        self.read()
        self.advance(t_wall)
        if self.encoded is None:
            out = io.BytesIO()
            self.canvas.save(out, "JPEG", quality=JPEG_QUALITY)
            self.encoded = out.getvalue()
        return self.encoded
