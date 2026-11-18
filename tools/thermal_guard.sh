#!/usr/bin/env bash
# Thermal + load guard for long simulation sessions.
#
# Reads the CPU temperature (k10temp Tctl) every few seconds and logs it.
#  - temp >= PAUSE_C for 2 samples: SIGSTOP every PID listed in run/pids/*.pid,
#    touch run/TAINTED (a paused run has broken wall-clock timing and must be redone)
#    and touch run/HOT.
#  - temp <= RESUME_C: SIGCONT the same PIDs and remove run/HOT.
# Launchers must wait while run/HOT exists (see tools/wait_cool.sh).
# Only PIDs registered in run/pids are ever signalled.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PAUSE_C=${PAUSE_C:-90}
RESUME_C=${RESUME_C:-75}
PERIOD=${PERIOD:-3}
LOG="$ROOT/run/thermal.log"
mkdir -p "$ROOT/run/pids"

read_temp() {
  local f
  for f in /sys/class/hwmon/hwmon*; do
    if [ "$(cat "$f/name" 2>/dev/null)" = "k10temp" ]; then
      echo $(( $(cat "$f/temp1_input") / 1000 )); return
    fi
  done
  echo $(( $(cat /sys/class/thermal/thermal_zone0/temp) / 1000 ))
}
signal_all() {
  local p
  for pf in "$ROOT"/run/pids/*.pid; do
    [ -e "$pf" ] || continue
    for p in $(cat "$pf"); do kill "-$1" "$p" 2>/dev/null; done
  done
}

hot=0; over=0; n=0
trap 'signal_all CONT; rm -f "$ROOT/run/HOT"; exit 0' INT TERM
while true; do
  t=$(read_temp); load=$(cut -d' ' -f1 /proc/loadavg)
  if [ "$hot" = 0 ]; then
    if [ "$t" -ge "$PAUSE_C" ]; then over=$((over+1)); else over=0; fi
    if [ "$over" -ge 2 ]; then
      hot=1; touch "$ROOT/run/HOT" "$ROOT/run/TAINTED"; signal_all STOP
      echo "$(date +%F\ %T) temp=$t load=$load PAUSED sims" >> "$LOG"
    fi
  else
    signal_all STOP   # catch PIDs registered after the pause
    if [ "$t" -le "$RESUME_C" ]; then
      hot=0; over=0; rm -f "$ROOT/run/HOT"; signal_all CONT
      echo "$(date +%F\ %T) temp=$t load=$load RESUMED sims" >> "$LOG"
    fi
  fi
  n=$((n+1))
  if [ $((n % 10)) = 0 ]; then echo "$(date +%F\ %T) temp=$t load=$load hot=$hot" >> "$LOG"; fi
  sleep "$PERIOD"
done
