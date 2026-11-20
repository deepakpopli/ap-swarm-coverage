#!/usr/bin/env bash
# Run one mission on Gazebo WITH the window open and the marker overlay, for watching.
# Usage: tools/show_gazebo.sh [run_mission.py arguments]   (default: Mission A, seed 3)
# The evaluation never uses this script: it stays headless.
cd "$(dirname "$0")/.." || exit 1
ARGS=("$@"); [ ${#ARGS[@]} -eq 0 ] && ARGS=(--mission A --n 3 --seed 3)
OUT=runs/show/$(date +%H%M%S)
nice -n 19 python3 run_mission.py --backend gazebo "${ARGS[@]}" --out "$OUT" > run/show.out 2>&1 &
MISSION_PID=$!
echo "$MISSION_PID" > run/show.pid
export GZ_PARTITION="swarm$MISSION_PID" PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
until pgrep -f "gz sim -s .*$OUT" > /dev/null; do sleep 1; kill -0 "$MISSION_PID" || exit 1; done
sleep 3
gz sim -g > run/show_gui.out 2>&1 &
GUI_PID=$!
sleep 8
nice -n 19 python3 tools/gz_markers.py > run/show_markers.out 2>&1 &
MARKERS_PID=$!
# overview camera: south of the area, looking north and down
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'pose: {position: {x: 150, y: -230, z: 330},
  orientation: {x: -0.2706, y: 0.2706, z: 0.6533, w: 0.6533}}' > /dev/null 2>&1
echo "watching $OUT (mission pid $MISSION_PID)"
wait "$MISSION_PID"
kill "$MARKERS_PID" "$GUI_PID" 2>/dev/null
echo "finished: $OUT"
