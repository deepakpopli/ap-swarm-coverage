#!/usr/bin/env python3
"""Stream the live-view drone cameras (SWARM_DEMO_CAMERA=1, sim/demo_camera.py) and save
geotagged frames.

Viewer only: it reads the camera images and model poses from Gazebo and changes nothing.
For each drone N (topic /swarm/cam/N, model iris_<N-1>):
- every frame goes to MediaMTX as rtsp://127.0.0.1:8554/droneN (H.264 through ffmpeg); when
  MediaMTX is not up it tries again every few seconds, quietly;
- once a second a 480 x 270 JPEG goes to <run_dir>/cam/dN_<seq>.jpg (top = the nose) with one
  line in <run_dir>/cam/frames.jsonl:
  {"drone": N, "t_wall": s, "x": m, "y": m, "z": m, "yaw": rad, "file": "dN_000000.jpg",
   "w": 480, "h": 270, "hfov_deg": 81.0}
  x, y in the area frame (the Gazebo world frame, as in agent_N.csv), z the height above the
  ground, yaw as in agent_N.csv (0 = north, clockwise); the pose is the one nearest the
  frame's sim time.

    GZ_PARTITION=swarm<runner pid> PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python \
        python3 tools/cam_bridge.py --out <run_dir>
"""
import argparse
import collections
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time

import numpy as np
from PIL import Image as Picture

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

from sim import demo_camera
from swarm import sensor

RTSP_URL = "rtsp://127.0.0.1:8554/drone{n}"
RETRY_S = 5.0              # MediaMTX not up (or the stream broke): try again after this
SAVE_EVERY_S = 1.0
JPEG_W, JPEG_H, JPEG_QUALITY = 480, 270, 70
POSES_KEPT = 400           # per drone, about a second of poses to match a frame's sim time
QUIET_EXIT_S = 20.0        # no frames for this long, once a camera was found: the run has ended
CAM_TOPIC = re.compile(r"/swarm/cam/(\d+)")
DRONE_NAME = re.compile(r"iris_(\d+)")

node = Node()
lock = threading.Lock()
poses = collections.defaultdict(lambda: collections.deque(maxlen=POSES_KEPT))  # N -> (t, pose)
last_frame_wall = [time.time()]


def stamp_s(msg) -> float:
    return msg.header.stamp.sec + 1e-9 * msg.header.stamp.nsec


def on_poses(msg: Pose_V) -> None:
    t = stamp_s(msg)
    for p in msg.pose:
        match = DRONE_NAME.fullmatch(p.name)
        if match:
            o = p.orientation
            pose = demo_camera.compass_pose(p.position.x, p.position.y, p.position.z,
                                            o.w, o.x, o.y, o.z)
            with lock:
                poses[int(match.group(1)) + 1].append((t, pose))


def pose_at(n: int, t: float):
    """Drone n's (x, y, z, yaw) nearest sim time t, or None before the first pose."""
    with lock:
        kept = list(poses[n])
    if not kept:
        return None
    return min(kept, key=lambda tp: abs(tp[0] - t))[1]


def rgb_array(msg: Image) -> np.ndarray:
    """The image as rows x columns x RGB; row 0 is the top of the image (the nose)."""
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    return rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)


class Stream(threading.Thread):
    """One drone's frames to MediaMTX through ffmpeg; only the newest frame is kept, so a slow
    or stuck ffmpeg never holds up Gazebo's callbacks."""

    def __init__(self, n: int):
        super().__init__(daemon=True)
        self.n = n
        self.frame = None
        self.ready = threading.Event()
        self.ffmpeg = None
        self.failed_at = 0.0

    def push(self, frame: np.ndarray) -> None:
        self.frame = frame
        self.ready.set()

    def start_ffmpeg(self, w: int, h: int) -> None:
        cmd = ["ffmpeg", "-loglevel", "quiet", "-nostdin", "-f", "rawvideo",
               "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(demo_camera.RATE_HZ), "-i", "-",
               "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
               "-pix_fmt", "yuv420p", "-g", "20", "-f", "rtsp", "-rtsp_transport", "tcp",
               RTSP_URL.format(n=self.n)]
        self.ffmpeg = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                                       stderr=subprocess.DEVNULL)

    def stop(self) -> None:
        if self.ffmpeg is not None:
            self.ffmpeg.kill()
            self.ffmpeg.wait()
            self.ffmpeg = None

    def run(self) -> None:
        while True:
            self.ready.wait()
            self.ready.clear()
            frame = self.frame
            if self.ffmpeg is not None and self.ffmpeg.poll() is not None:
                self.stop()                        # MediaMTX went away or refused
                self.failed_at = time.time()
            if self.ffmpeg is None:
                if time.time() - self.failed_at < RETRY_S:
                    continue
                self.start_ffmpeg(frame.shape[1], frame.shape[0])
            try:
                self.ffmpeg.stdin.write(frame.tobytes())
                self.ffmpeg.stdin.flush()
            except OSError:                        # broken pipe: ffmpeg exited
                self.stop()
                self.failed_at = time.time()


class Camera:
    """One drone: every frame to its stream, one a second to disk."""

    def __init__(self, n: int, cam_dir: str, log):
        self.n, self.cam_dir, self.log = n, cam_dir, log
        self.seq = 0
        self.saved_at = 0.0
        self.stream = Stream(n)
        self.stream.start()

    def on_image(self, msg: Image) -> None:
        t_wall = time.time()
        last_frame_wall[0] = t_wall
        frame = rgb_array(msg)
        self.stream.push(frame)
        if t_wall - self.saved_at >= SAVE_EVERY_S:
            pose = pose_at(self.n, stamp_s(msg))
            if pose is not None:
                self.saved_at = t_wall
                self.save(frame, t_wall, pose)

    def save(self, frame: np.ndarray, t_wall: float, pose) -> None:
        name = f"d{self.n}_{self.seq:06d}.jpg"
        self.seq += 1
        path = os.path.join(self.cam_dir, name)
        small = Picture.fromarray(frame).resize((JPEG_W, JPEG_H), Picture.BILINEAR)
        small.save(path + ".tmp", "JPEG", quality=JPEG_QUALITY)
        os.replace(path + ".tmp", path)            # the line below never names a partial file
        x, y, z, yaw = pose
        line = json.dumps({"drone": self.n, "t_wall": round(t_wall, 3), "x": round(x, 3),
                           "y": round(y, 3), "z": round(z, 3), "yaw": round(yaw, 5),
                           "file": name, "w": JPEG_W, "h": JPEG_H,
                           "hfov_deg": float(sensor.HFOV_DEG)})
        with lock:
            self.log.write(line + "\n")             # one write, then flush: whole lines only
            self.log.flush()


def main() -> None:
    parser = argparse.ArgumentParser(description="stream and save the live-view drone cameras")
    parser.add_argument("--out", required=True, help="the run directory; frames go to <out>/cam")
    cam_dir = os.path.join(parser.parse_args().out, "cam")
    os.makedirs(cam_dir, exist_ok=True)
    log = open(os.path.join(cam_dir, "frames.jsonl"), "a")
    node.subscribe(Pose_V, "/world/swarm/dynamic_pose/info", on_poses)   # moving drones
    node.subscribe(Pose_V, "/world/swarm/pose/info", on_poses)           # also a drone at rest
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))                # stop the ffmpegs too
    cameras = {}
    try:
        while not cameras or time.time() - last_frame_wall[0] < QUIET_EXIT_S:   # a slow world load waits
            for topic in node.topic_list():        # drones whose camera has come up
                match = CAM_TOPIC.fullmatch(topic)
                if match and int(match.group(1)) not in cameras:
                    n = int(match.group(1))
                    cameras[n] = Camera(n, cam_dir, log)
                    last_frame_wall[0] = time.time()
                    node.subscribe(Image, topic, cameras[n].on_image)
                    print(f"drone {n}: {topic} -> {RTSP_URL.format(n=n)}", flush=True)
            time.sleep(2.0)
    finally:
        for camera in cameras.values():
            camera.stream.stop()


if __name__ == "__main__":
    main()
