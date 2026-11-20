# Sourced by run_eval.sh and run_scenarios.sh: simulator runs one at a time, each through the
# machine lock (run/sim.lock: the simulator is shared) and the thermal guard. A finished run is
# skipped, so re-running continues; a failed or tainted attempt is kept in <runs>/failed/ and
# retried up to RETRIES times. Callers set BACKEND and FORCE and read SUMMARY.
LOCK=run/sim.lock
RETRIES=2
SUMMARY=()

say() { echo "[$(basename "$0") $(date +%H:%M:%S)] $*"; }

outcome() {   # $1 = run directory
  python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["outcome"])' \
    "$1/meta.json" 2> /dev/null || echo "no meta.json"
}

is_done() { [ "$(outcome "$1")" = ok ]; }

ensure_guard() {   # the guard must run during every simulation; start it if it died
  local pid
  pid=$(cat run/guard.pid 2> /dev/null)
  if [ -n "$pid" ] && grep -q thermal_guard "/proc/$pid/cmdline" 2> /dev/null; then return; fi
  mkdir -p run
  setsid nohup bash tools/thermal_guard.sh > /dev/null 2>&1 < /dev/null &
  echo $! > run/guard.pid
  say "started the thermal guard (pid $!)"
}

no_leftovers() {   # nothing of a simulator may outlive its run; never kill by pattern
  local left
  left=$(ps -eo pid,args | grep -E '[a]rducopter|[g]z sim')
  [ -z "$left" ] && return 0
  say "simulator processes left after a run, stopping: $left"
  exit 1
}

run_one() {   # $1 = runs directory, $2 = run name, then run_mission.py options
  local runs=$1 name=$2 dir="$1/$2" attempt failed=()
  shift 2
  if is_done "$dir" && [ "$FORCE" != 1 ]; then
    failed=("$runs"/failed/"$name"_*_attempt*)
    [ -e "${failed[0]}" ] || failed=()
    SUMMARY+=("$name: done before, skipped (${#failed[@]} failed attempts in $runs/failed/)")
    return
  fi
  mkdir -p "$runs/failed"
  [ -e "$dir" ] && mv "$dir" "$runs/failed/${name}_$(date +%Y%m%d-%H%M%S)_replaced"
  for attempt in $(seq 1 $((RETRIES + 1))); do
    ensure_guard
    bash tools/thermal_guard.sh --wait
    say "$name: attempt $attempt, waiting for the simulator lock"
    flock "$LOCK" nice -n 19 python3 run_mission.py --backend "$BACKEND" --out "$dir" "$@"
    no_leftovers
    if is_done "$dir"; then
      SUMMARY+=("$name: ok on attempt $attempt${failed:+ (failed before: ${failed[*]})}")
      return
    fi
    failed+=("$(outcome "$dir")")
    mv "$dir" "$runs/failed/${name}_$(date +%Y%m%d-%H%M%S)_attempt$attempt"
  done
  SUMMARY+=("$name: FAILED $attempt attempts: ${failed[*]} (kept in $runs/failed/)")
}
