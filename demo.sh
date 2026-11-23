#!/usr/bin/env bash
# The live demo in one command: the web view, then one mission in the Gazebo window.
#
#   ./demo.sh [A|B|C] [--seed N] [--duration S] [--forever] [--no-camera] [--port P] [--host H]
#
#   A|B|C        the mission (default B, the priority zone)
#   --seed N     which drone fails, and when (default: a random seed)
#   --duration S mission length in seconds (default 180). The fault comes at 60-90 s; under
#                120 s it is moved to half the duration so it is still seen
#   --forever    the mission does not end on its own: only Ctrl+C stops it
#   --no-camera  no drone cameras (they need MediaMTX: $MEDIAMTX or mediamtx on the PATH)
#   --port P     web view port (default 8801), --host H its address (default localhost)
#
# Ctrl+C, or the end of the mission, stops everything this script started, and only that.
# One simulator at a time: if run/sim.lock is held, it says so and waits (Ctrl+C gives up).
cd "$(dirname "$0")" || exit 1

MISSION=B SEED=$((RANDOM % 1000 + 1)) DURATION=180 FOREVER=0 CAMERA=1 PORT=8801 HOST=localhost
usage() { sed -n '4,13p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
while [ $# -gt 0 ]; do
  case $1 in
    A|B|C) MISSION=$1 ;;
    --seed) SEED=$2; shift ;;
    --duration) DURATION=$2; shift ;;
    --forever) FOREVER=1 ;;
    --no-camera) CAMERA=0 ;;
    --port) PORT=$2; shift ;;
    --host) HOST=$2; shift ;;
    -h|--help) usage ;;
    *) echo "unknown argument: $1"; usage 2 ;;
  esac
  shift
done
for value in "$SEED" "$DURATION" "$PORT"; do
  [[ $value =~ ^[0-9]+$ ]] || { echo "not a whole number: '$value'"; usage 2; }
done
[ "$DURATION" -ge 30 ] || { echo "--duration must be at least 30 s"; exit 2; }

# one simulator at a time (the lock is shared by every run on this machine)
mkdir -p run
exec 9>> run/sim.lock
if ! flock -n 9; then
  echo "another simulation is running (run/sim.lock is held): waiting for it to end (Ctrl+C gives up)"
  flock 9 || exit 1
fi

MISSION_ARGS=(--mission "$MISSION" --n 3 --seed "$SEED")
if [ "$FOREVER" = 1 ]; then
  MISSION_ARGS+=(--mission-s 1000000)
  echo "mission $MISSION, seed $SEED, no end: Ctrl+C stops it"
else
  MISSION_ARGS+=(--mission-s "$DURATION")
  echo "mission $MISSION, seed $SEED, $DURATION s"
  if [ "$DURATION" -lt 120 ]; then
    MISSION_ARGS+=(--kill-at $((DURATION / 2)))
    echo "short mission: the fault comes at $((DURATION / 2)) s"
  fi
fi

# the demo look: big drones with spinning props, the field around the area, the zone (B)
export SWARM_DEMO=1 SWARM_DEMO_SCALE=6 SWARM_DEMO_PHYSICS=1 SWARM_DEMO_GROUND=field \
  SWARM_MARKER_RINGS=0 SWARM_FOOTPRINT_W=${SWARM_FOOTPRINT_W:-1.2} SWARM_DEMO_ZONE=0
[ "$MISSION" = B ] && SWARM_DEMO_ZONE=1
if command -v nvidia-smi > /dev/null; then      # draw the window on the NVIDIA card
  export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
    __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
fi
export SWARM_DEMO_CAMERA=0
if [ "$CAMERA" = 1 ]; then
  if MEDIAMTX=$(command -v "${MEDIAMTX:-mediamtx}"); then
    export MEDIAMTX SWARM_DEMO_CAMERA=1
  else
    echo "MediaMTX not found (set MEDIAMTX=/path/to/mediamtx): running without the cameras"
  fi
fi

# --- start and stop. Each part runs in its own process group; the stop signals only those.
SHOW_PID='' GUI_PID=''
alive() {   # a process group of ours still has a live (not zombie) process
  ps -eo pgid=,stat= | awk -v g="$1" '$1 == g && $2 !~ /^Z/ {found = 1} END {exit !found}'
}
stop_group() {   # $1 = process group, $2 = seconds it gets after SIGTERM
  local i
  alive "$1" || return 0
  kill -TERM -- "-$1" 2>/dev/null
  for ((i = 0; i < $2; i++)); do alive "$1" || return 0; sleep 1; done
  echo "process group $1 ignored SIGTERM: SIGKILL"
  kill -KILL -- "-$1" 2>/dev/null
  sleep 1
}
cleanup() {
  trap '' INT TERM HUP                       # a second Ctrl+C must not stop the stop
  trap - EXIT
  local mission i group port left='' busy=()
  if [ -n "$SHOW_PID" ] && alive "$SHOW_PID"; then
    echo "stopping the simulation..."
    # the mission first: it stops its agents, autopilots and Gazebo in order, and says why it ended
    mission=$(pgrep -g "$SHOW_PID" -f run_mission.py | head -n 1)
    [ -n "$mission" ] && kill -INT "$mission" 2>/dev/null
    for ((i = 0; i < 40; i++)); do alive "$SHOW_PID" || break; sleep 1; done
  fi
  [ -n "$SHOW_PID" ] && stop_group "$SHOW_PID" 10
  [ -n "$GUI_PID" ] && stop_group "$GUI_PID" 5
  wait 2>/dev/null
  # check: nothing of this run is left, and its ports are free
  for group in $SHOW_PID $GUI_PID; do
    left+=$(ps -eo pid=,pgid=,stat=,args= | awk -v g="$group" '$2 == g && $3 !~ /^Z/')
  done
  [ -n "$left" ] && echo "still running:" && echo "$left"
  # autopilots (TCP 5760 + 10 i, + 2), radio, ground and agents (UDP), MediaMTX, the web view
  for port in 5760 5762 5770 5772 5780 5782 14000 14099 14101 14102 14103 8554 8889 8189; do
    [ -n "$(ss -Htuln "sport = :$port")" ] && busy+=("$port")
  done
  [ -n "$GUI_PID" ] && [ -n "$(ss -Htln "sport = :$PORT")" ] && busy+=("$PORT")
  if [ ${#busy[@]} -gt 0 ]; then
    echo "ports still in use: ${busy[*]}"
  elif [ -z "$left" ]; then
    echo "stopped: nothing left running, ports free"
  fi
  [ -f run/show.out ] && echo "mission: $(tail -n 1 run/show.out)"
  exit "${1:-0}"
}
trap 'cleanup 130' INT
trap 'cleanup 143' TERM HUP
trap 'cleanup' EXIT

# the web view, unless something already listens on the port (then that one is used)
if [ -n "$(ss -Htln "sport = :$PORT")" ]; then
  echo "port $PORT is in use: not starting the web view (one may be running there already)"
else
  setsid python3 -m gui --host "$HOST" --port "$PORT" --runs runs/show > run/demo_gui.out 2>&1 9>&- &
  GUI_PID=$!
  sleep 1
  if alive "$GUI_PID"; then
    echo "web view: http://$HOST:$PORT"
  else
    echo "the web view did not start (run/demo_gui.out):"; cat run/demo_gui.out; GUI_PID=
  fi
fi

echo "starting Gazebo (the window opens in about 20 s)..."
setsid nice -n 19 tools/show_gazebo.sh "${MISSION_ARGS[@]}" &
SHOW_PID=$!
wait "$SHOW_PID"
