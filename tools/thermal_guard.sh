#!/usr/bin/env bash
# CPU temperature guard for simulation runs. Two modes:
#
#   tools/thermal_guard.sh          guard (runs forever; its pid goes in run/guard.pid).
#     At >= PAUSE_C for 2 samples: SIGSTOP every PID in run/pids/*.pid, touch run/HOT and
#     run/TAINTED (a paused run has broken wall-clock timing and must be redone).
#     At <= RESUME_C: SIGCONT them and remove run/HOT. Only registered PIDs are signalled.
#   tools/thermal_guard.sh --wait   block until a new run may start: no run/HOT,
#     temperature <= START_C and 1-minute load <= MAX_LOAD.
#
# Without a temperature sensor both modes fail: guessing "cool" could cook the machine.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PAUSE_C=${PAUSE_C:-90}
RESUME_C=${RESUME_C:-75}
START_C=${START_C:-70}
MAX_LOAD=${MAX_LOAD:-10}
PERIOD_S=${PERIOD_S:-3}
LOG="$ROOT/run/thermal.log"
mkdir -p "$ROOT/run/pids"

# CPU temperature in whole degrees C: AMD k10temp, else the first thermal zone.
read_temp() {
  local f
  for f in /sys/class/hwmon/hwmon*; do
    if [ "$(cat "$f/name" 2>/dev/null)" = "k10temp" ]; then
      echo $(( $(cat "$f/temp1_input") / 1000 )); return
    fi
  done
  if [ -r /sys/class/thermal/thermal_zone0/temp ]; then
    echo $(( $(cat /sys/class/thermal/thermal_zone0/temp) / 1000 )); return
  fi
  echo "thermal_guard: no temperature sensor found" >&2
  exit 2
}

signal_all() {
  local pid_file pid
  for pid_file in "$ROOT"/run/pids/*.pid; do
    [ -e "$pid_file" ] || continue
    for pid in $(cat "$pid_file"); do kill "-$1" "$pid" 2>/dev/null; done
  done
}

wait_cool() {
  local temp load
  while true; do
    temp=$(read_temp) || exit 2
    load=$(cut -d' ' -f1 /proc/loadavg)
    if [ ! -e "$ROOT/run/HOT" ] && [ "$temp" -le "$START_C" ] \
        && awk "BEGIN{exit !($load <= $MAX_LOAD)}"; then
      exit 0
    fi
    echo "waiting: temp=${temp}C load=$load" >&2
    sleep 10
  done
}

guard() {
  local hot=0 over=0 n=0 temp load
  trap 'signal_all CONT; rm -f "$ROOT/run/HOT"; exit 0' INT TERM
  while true; do
    temp=$(read_temp) || exit 2
    load=$(cut -d' ' -f1 /proc/loadavg)
    if [ "$hot" = 0 ]; then
      if [ "$temp" -ge "$PAUSE_C" ]; then over=$((over + 1)); else over=0; fi
      if [ "$over" -ge 2 ]; then
        hot=1
        touch "$ROOT/run/HOT" "$ROOT/run/TAINTED"
        signal_all STOP
        echo "$(date '+%F %T') temp=$temp load=$load PAUSED sims" >> "$LOG"
      fi
    else
      signal_all STOP   # also catches PIDs registered after the pause
      if [ "$temp" -le "$RESUME_C" ]; then
        hot=0; over=0
        rm -f "$ROOT/run/HOT"
        signal_all CONT
        echo "$(date '+%F %T') temp=$temp load=$load RESUMED sims" >> "$LOG"
      fi
    fi
    n=$((n + 1))
    if [ $((n % 10)) = 0 ]; then
      echo "$(date '+%F %T') temp=$temp load=$load hot=$hot" >> "$LOG"
    fi
    sleep "$PERIOD_S"
  done
}

if [ "$1" = "--wait" ]; then wait_cool; else guard; fi
