"""Photo mosaic of a run: each camera frame (cam/frames.jsonl) laid on its ground footprint,
later frames over earlier ones, north up. Built incrementally as the browser's time moves.
"""
import bisect
import io
import json
import math
import os

from PIL import Image

from swarm import sensor

RES_M = 0.5                      # metres per mosaic pixel
EXTENT_M = (-10.0, 310.0)        # x and y range of the mosaic, metres (the field and a margin)
SIZE_PX = int(round((EXTENT_M[1] - EXTENT_M[0]) / RES_M))
BACKGROUND = (247, 247, 247)     # never photographed: the map's ground colour
SNAPSHOT_EVERY = 60              # frames between saved canvases, for seeking back
JPEG_QUALITY = 80


def footprint_m(frame):
    """(across_m, along_m) on the ground: as sensor.footprint_size, from the frame's own fov."""
    hfov = math.radians(frame.get("hfov_deg", sensor.HFOV_DEG))
    across = 2.0 * frame["z"] * math.tan(hfov / 2.0)
    return across, across * frame.get("h", 9) / frame.get("w", 16)


def ground_to_image(x, y, frame, img_w, img_h):
    """Image pixel (u, v) that sees ground point (x, y): top = nose, right = right of the nose."""
    across, along = footprint_m(frame)
    (fx, fy), (rx, ry) = sensor.heading_axes(frame["yaw"])
    dx, dy = x - frame["x"], y - frame["y"]
    u = ((dx * rx + dy * ry) / across + 0.5) * img_w
    v = (0.5 - (dx * fx + dy * fy) / along) * img_h
    return u, v


def affine(frame, img_w, img_h, ox, oy):
    """PIL AFFINE coefficients: mosaic pixel (ox + i, oy + j) -> image pixel (u, v)."""
    u0, v0 = ground_to_image(EXTENT_M[0] + ox * RES_M, EXTENT_M[1] - oy * RES_M, frame, img_w, img_h)
    u1, v1 = ground_to_image(EXTENT_M[0] + (ox + 1) * RES_M, EXTENT_M[1] - oy * RES_M, frame, img_w, img_h)
    u2, v2 = ground_to_image(EXTENT_M[0] + ox * RES_M, EXTENT_M[1] - (oy + 1) * RES_M, frame, img_w, img_h)
    return (u1 - u0, u2 - u0, u0, v1 - v0, v2 - v0, v0)


def paste_frame(canvas, frame, image):
    """Warp one frame onto the canvas (an RGB Image of SIZE_PX), inside its footprint only."""
    if frame["z"] <= 5.0:          # parked or taking off: the ground is inside the near clip
        return
    across, along = footprint_m(frame)
    corners = sensor_corners(frame, across, along)
    cols = [(x - EXTENT_M[0]) / RES_M for x, _ in corners]
    rows = [(EXTENT_M[1] - y) / RES_M for _, y in corners]
    x0, y0 = max(0, math.floor(min(cols))), max(0, math.floor(min(rows)))
    x1, y1 = min(SIZE_PX, math.ceil(max(cols))), min(SIZE_PX, math.ceil(max(rows)))
    if x1 <= x0 or y1 <= y0:
        return
    coeffs = affine(frame, image.width, image.height, x0, y0)
    size = (x1 - x0, y1 - y0)
    warped = image.transform(size, Image.AFFINE, coeffs, Image.BILINEAR)
    mask = Image.new("L", image.size, 255).transform(size, Image.AFFINE, coeffs, Image.NEAREST,
                                                     fillcolor=0)
    canvas.paste(warped, (x0, y0), mask)


def sensor_corners(frame, across, along):
    (fx, fy), (rx, ry) = sensor.heading_axes(frame["yaw"])
    return [(frame["x"] + f * along / 2 * fx + c * across / 2 * rx,
             frame["y"] + f * along / 2 * fy + c * across / 2 * ry)
            for f, c in ((1, -1), (-1, -1), (-1, 1), (1, 1))]


class Mosaic:
    """Frames sorted by time; the canvas holds frames[:applied]."""

    def __init__(self, cam_dir, skip=lambda frame: False):
        self.cam_dir = cam_dir
        self.skip = skip             # frames to leave out (a dead drone's camera)
        self.offset = 0              # bytes of frames.jsonl read
        self.keys = []               # sorted (t_wall, line number)
        self.frames = []             # the frame dicts, in the same order
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
            self.paint(self.frames[self.applied])
            self.applied += 1
            self.encoded = None
            if self.applied % SNAPSHOT_EVERY == 0 and self.applied > self.snapshots[-1][0]:
                self.snapshots.append((self.applied, self.canvas.copy()))

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
