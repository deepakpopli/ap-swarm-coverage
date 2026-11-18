#!/usr/bin/env bash
# Block until the machine is safe to start a simulation set:
# no run/HOT flag, CPU temp <= START_C and 1-min load <= MAX_LOAD.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
START_C=${START_C:-70}; MAX_LOAD=${MAX_LOAD:-10}
temp() { for f in /sys/class/hwmon/hwmon*; do [ "$(cat $f/name 2>/dev/null)" = k10temp ] && { echo $(( $(cat $f/temp1_input)/1000 )); return; }; done; echo 0; }
while true; do
  t=$(temp); l=$(cut -d' ' -f1 /proc/loadavg)
  if [ ! -e "$ROOT/run/HOT" ] && [ "$t" -le "$START_C" ] && awk "BEGIN{exit !($l <= $MAX_LOAD)}"; then exit 0; fi
  echo "waiting: temp=${t}C load=$l" >&2; sleep 10
done
