#!/usr/bin/env python3
"""Keep the world of a live show run running: tools/show_gazebo.sh starts this next to the window.

The Gazebo window can pause the world: its play / pause button and, window-wide, the Space key
(gz-gui WorldControl.qml: Shortcut "Space"). The window takes the keyboard focus when it opens,
so one stray Space pauses the world. A paused world ends the mission: the autopilots get no
sensor frames (lock_step 0: they then advance 2.5 ms per 100 ms), their telemetry stops, every
agent sees its autopilot link silent and exits ("autopilot_link_lost", runner: "agent_N died
unexpectedly (exit 2)").

This guard reads /world/swarm/stats (GZ_PARTITION from the environment) and, when the world is
paused, un-pauses it at once through /world/swarm/control and prints one line with the time.
A pause then lasts a fraction of a second, shorter than the agents' link timeout.
Only for watching: the evaluation is headless, has no window and never starts this.
"""
import os
import sys
import time

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node

WORLD = "swarm"
QUIET_EXIT_S = 20.0        # no stats for this long: the simulation has ended
RESUME_EVERY_S = 0.2       # while paused: ask again this often

node = Node()
state = {"paused": False, "sim_s": 0.0, "t_msg": time.time()}


def on_stats(msg: WorldStatistics) -> None:
    state["paused"] = msg.paused
    state["sim_s"] = msg.sim_time.sec + msg.sim_time.nsec * 1e-9
    state["t_msg"] = time.time()


def say(text: str) -> None:
    now = time.time()
    stamp = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int(now % 1 * 1000):03d}"
    print(f"[gz_pause_guard {stamp}] {text}", flush=True)


def resume() -> None:
    request = WorldControl()
    request.pause = False
    node.request(f"/world/{WORLD}/control", request, WorldControl, Boolean, 1000)


def main() -> int:
    if not node.subscribe(WorldStatistics, f"/world/{WORLD}/stats", on_stats):
        say("could not subscribe to the world stats")
        return 1
    say("watching: a paused world is resumed at once")
    paused_since = None
    while time.time() - state["t_msg"] < QUIET_EXIT_S:
        if state["paused"]:
            if paused_since is None:
                paused_since = time.time()
                say(f"WORLD PAUSED at sim time {state['sim_s']:.3f} s (Space or the play / "
                    "pause button in the window?): resuming")
            resume()
            time.sleep(RESUME_EVERY_S)
        else:
            if paused_since is not None:
                say(f"running again after {time.time() - paused_since:.2f} s")
                paused_since = None
            time.sleep(0.02)
    return 0


if __name__ == "__main__":
    sys.exit(main())
