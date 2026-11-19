#!/usr/bin/env bash
# The evaluation in one command: Mission A runs, one N-1 reference run and one no-loss baseline
# run, one after the other and headless, then metrics, report, plots and one animation.
#
#   ./run_eval.sh                      # gazebo, seeds 1..5, 50 m, 180 s
#   BACKEND=sitl SEEDS="1 2 3" ./run_eval.sh
#
# Environment: BACKEND (gazebo | sitl | fake, default gazebo), SEEDS (default "1 2 3 4 5"),
# SWARM_ALT (cruise altitude in m, default 50), MISSION_S (default 180), FORCE=1 (re-run runs
# that are already done). Set name: ${BACKEND}_${SWARM_ALT}m. Raw runs go to runs/<set>/
# (git-ignored), tables and figures to results/<set>/. The fake backend is not a measurement:
# it writes under run/ instead.
#
# A run directory whose meta.json says outcome "ok" is done and skipped. A failed or tainted
# attempt is moved to runs/<set>/failed/ and retried, at most 3 attempts per run.
set -u
cd "$(dirname "$0")" || exit 1

BACKEND=${BACKEND:-gazebo}
SEEDS=${SEEDS:-"1 2 3 4 5"}
export SWARM_ALT=${SWARM_ALT:-50}
MISSION_S=${MISSION_S:-180}
FORCE=${FORCE:-0}
SET="${BACKEND}_${SWARM_ALT}m"
RUNS="runs/$SET"
RESULTS="results/$SET"
if [ "$BACKEND" = fake ]; then RUNS="run/runs/$SET"; RESULTS="run/results/$SET"; fi
N=3
ATTEMPTS=3
SUMMARY=()
mkdir -p "$RUNS"

say() { echo "[run_eval $(date +%H:%M:%S)] $*"; }

is_done() {   # $1 = run directory
  [ -f "$1/meta.json" ] && python3 -c 'import json, sys
sys.exit(json.load(open(sys.argv[1]))["outcome"] != "ok")' "$1/meta.json"
}

move_to_failed() {   # $1 = run directory, $2 = label
  mkdir -p "$RUNS/failed"
  mv "$1" "$RUNS/failed/$(basename "$1")_$2_$(date +%Y%m%d-%H%M%S)"
}

run_one() {   # $1 = run name, then run_mission.py options
  local name=$1 dir="$RUNS/$1" attempt
  shift
  if is_done "$dir" && [ "$FORCE" != 1 ]; then
    SUMMARY+=("$name: done before, skipped")
    return
  fi
  [ -e "$dir" ] && move_to_failed "$dir" replaced
  for attempt in $(seq 1 $ATTEMPTS); do
    say "$name: attempt $attempt of $ATTEMPTS"
    python3 run_mission.py --backend "$BACKEND" --mission-s "$MISSION_S" --out "$dir" "$@"
    if is_done "$dir"; then
      SUMMARY+=("$name: ok on attempt $attempt")
      return
    fi
    say "$name: $(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["outcome"])' \
      "$dir/meta.json" 2> /dev/null)"
    move_to_failed "$dir" "attempt$attempt"
  done
  SUMMARY+=("$name: FAILED $ATTEMPTS attempts (see $RUNS/failed/)")
}

# ---- thermal guard: start it if needed, and stop it again only if we started it
guard_running() {
  local pid
  pid=$(cat run/guard.pid 2> /dev/null) || return 1
  grep -q thermal_guard "/proc/$pid/cmdline" 2> /dev/null
}
OUR_GUARD=""
if ! guard_running; then
  mkdir -p run
  setsid nohup bash tools/thermal_guard.sh > /dev/null 2>&1 < /dev/null &
  OUR_GUARD=$!
  echo "$OUR_GUARD" > run/guard.pid
  say "started the thermal guard (pid $OUR_GUARD)"
fi
stop_guard() { [ -n "$OUR_GUARD" ] && kill "$OUR_GUARD" && rm -f run/guard.pid; }
trap 'stop_guard; exit 130' INT TERM HUP

# ---- the runs, sequential: never two simulations at once
for s in $SEEDS; do
  run_one "seed_$s" --mission A --n $N --seed "$s"
done
FIRST_SEED=${SEEDS%% *}
run_one ref_n2 --mission ref --n $((N - 1)) --seed "$FIRST_SEED"
run_one baseline_n3 --mission ref --n $N --seed "$FIRST_SEED"
stop_guard

# ---- analysis of the valid runs
GOOD=(); REF=(); BASE=()
for s in $SEEDS; do is_done "$RUNS/seed_$s" && GOOD+=("$RUNS/seed_$s"); done
is_done "$RUNS/ref_n2" && REF+=("$RUNS/ref_n2")
is_done "$RUNS/baseline_n3" && BASE+=("$RUNS/baseline_n3")
ANALYSIS="not run: no valid Mission A run"
if [ ${#GOOD[@]} -gt 0 ]; then
  mkdir -p "$RESULTS"
  ANALYSIS=ok
  for dir in "${GOOD[@]}" "${REF[@]}" "${BASE[@]}"; do
    nice -n 19 python3 -m analysis.metrics "$dir" || ANALYSIS="FAILED (metrics $dir)"
  done
  nice -n 19 python3 -m analysis.report --out "$RESULTS" --name "$SET" "${GOOD[@]}" \
    --ref "${REF[@]}" --baseline "${BASE[@]}" || ANALYSIS="FAILED (report)"
  nice -n 19 python3 -m analysis.plots --out "$RESULTS" "${GOOD[@]}" \
    --ref "${REF[@]}" --baseline "${BASE[@]}" || ANALYSIS="FAILED (plots)"
  nice -n 19 python3 -m analysis.animate "${GOOD[0]}" \
    --out "$RESULTS/replay_$(basename "${GOOD[0]}").mp4" || ANALYSIS="FAILED (animation)"
fi

{
  echo "evaluation $SET, $(date '+%F %T'), mission $MISSION_S s, seeds: $SEEDS"
  printf '  %s\n' "${SUMMARY[@]}"
  echo "  analysis: $ANALYSIS -> $RESULTS/"
} | tee "$RUNS/summary.txt"
[ "$ANALYSIS" = ok ] && [ ${#GOOD[@]} -eq "$(echo "$SEEDS" | wc -w)" ]
