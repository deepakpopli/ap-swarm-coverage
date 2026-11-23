#!/usr/bin/env bash
# Run one mission on Gazebo WITH the window open and the marker overlay, for watching.
# Usage: tools/show_gazebo.sh [run_mission.py arguments]   (default: Mission A, seed 3)
# SWARM_DEMO=1 ... (sim/demo_world.py): the live view's drones and field.
# SWARM_DEMO_CAMERA=1 with it: each drone's nadir camera, streamed through MediaMTX (tools/mediamtx.yml,
# $MEDIAMTX) to the web view and saved once a second to $OUT/cam (tools/cam_bridge.py).
# The evaluation never uses this script: it stays headless.
cd "$(dirname "$0")/.." || exit 1
ARGS=("$@"); [ ${#ARGS[@]} -eq 0 ] && ARGS=(--mission A --n 3 --seed 3)
OUT=runs/show/$(date +%H%M%S)
mkdir -p run
# the drone cameras publish to MediaMTX (tools/mediamtx.yml); it stops with this script
if [ "${SWARM_DEMO_CAMERA:-}" = 1 ]; then
  if command -v "${MEDIAMTX:-mediamtx}" > /dev/null; then
    "${MEDIAMTX:-mediamtx}" tools/mediamtx.yml > run/show_mediamtx.out 2>&1 &
    MEDIAMTX_PID=$!
    trap 'kill "$MEDIAMTX_PID" 2>/dev/null' EXIT
  else
    echo "mediamtx not found (set MEDIAMTX=/path/to/mediamtx): no live video"
  fi
fi
# SWARM_LIVE=1: a drone removed in the window (right click, Remove) is a loss the swarm handles
SWARM_LIVE=1 nice -n 19 python3 run_mission.py --backend gazebo "${ARGS[@]}" --out "$OUT" > run/show.out 2>&1 &
MISSION_PID=$!
echo "$MISSION_PID" > run/show.pid
export GZ_PARTITION="swarm$MISSION_PID" PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
until pgrep -f "gz sim -s .*$OUT" > /dev/null; do sleep 1; kill -0 "$MISSION_PID" || exit 1; done
# this run's own logs: when the window, the overlay and the camera move came, and their output
step() { echo "[show_gazebo $(date +%H:%M:%S.%3N)] $*" >> "$OUT/show.log"; }
step "server up (mission pid $MISSION_PID, GZ_PARTITION $GZ_PARTITION)"
# The window can pause the world (Space, window-wide, or its play / pause button) and it takes
# the keyboard focus when it opens. A paused world ends the mission: every agent loses its
# autopilot link. So a guard resumes a paused world at once; it runs before the window opens.
nice -n 19 python3 tools/gz_pause_guard.py > "$OUT/show_pause_guard.out" 2>&1 &
GUARD_PID=$!
sleep 3
step "starting the window"
# the stock Gazebo window
gz sim -g > "$OUT/show_gui.out" 2>&1 &
GUI_PID=$!
sleep 8
step "starting the marker overlay"
nice -n 19 python3 tools/gz_markers.py > "$OUT/show_markers.out" 2>&1 &
MARKERS_PID=$!
CAM_PID=
if [ "$SWARM_DEMO" = 1 ] && [ "$SWARM_DEMO_CAMERA" = 1 ]; then
  step "starting the camera bridge"
  nice -n 19 python3 tools/cam_bridge.py --out "$OUT" > "$OUT/show_cam_bridge.out" 2>&1 &
  CAM_PID=$!
fi
# overview camera:
# south of the area looking north, 46 deg down; the whole yellow square fills the window
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'pose: {position: {x: 150, y: -160, z: 305},
  orientation: {x: -0.2754, y: 0.2754, z: 0.6513, w: 0.6513}}' > "$OUT/show_camera.out" 2>&1
step "camera moved to the overview: $(tr -d '\n' < "$OUT/show_camera.out")"
# a close-up while it runs (same GZ_PARTITION): follow drone 2, then back to the overview
#   gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 2000 --req 'data: "iris_1"'
#   gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 2000 --req 'x: -25, y: 0, z: 12'
#   gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 2000 --req 'data: ""'  (stop), then move_to/pose
echo "watching $OUT (mission pid $MISSION_PID)"
wait "$MISSION_PID"
step "mission ended: $(tail -n 1 run/show.out)"
kill "$MARKERS_PID" "$GUI_PID" "$GUARD_PID" $CAM_PID 2>/dev/null
grep -h "WORLD PAUSED" "$OUT/show_pause_guard.out" 2>/dev/null   # say so: the run had a glitch
echo "finished: $OUT"
