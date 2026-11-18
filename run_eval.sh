#!/usr/bin/env bash
# One-command headless evaluation: N Mission A runs, one after the other, then the analysis.
#
#   ./run_eval.sh                          # 5 runs, seeds 1..5, 180 s, backend gazebo
#   BACKEND=sitl SEEDS="1 2 3" ./run_eval.sh
#   SKIP_RUNS=1 ./run_eval.sh              # analysis only, on the runs already there
#
# Environment (all optional):
#   BACKEND     gazebo | sitl | fake           (default gazebo)
#   SEEDS       "1 2 3 4 5"
#   MISSION     A (B is not implemented; C works)                (default A)
#   MISSION_S   mission length, s                                (default 180)
#   N           number of drones                                 (default 3)
#   OUT         run root  (default runs/eval_<backend>; run/eval_fake for the fake backend,
#               because fake runs are not measurements and never go under runs/ or results/)
#   ANALYSIS_OUT  aggregate table + plots                       (default $OUT/analysis)
#   ANIM_SEED   seed whose run is animated                       (default: first of SEEDS)
#   RETRIES     extra attempts for a tainted/failed run          (default 2)
#   RUN_ARGS    extra run_mission.py flags, e.g. "--kill-window 20 30 --port-base 500"
#   SKIP_RUNS=1 analysis only
#
# Runs are SEQUENTIAL: never two simulators at once. A seed whose run already finished OK
# (meta.json outcome "ok") is not run again; delete its directory to redo it. A failed or
# tainted attempt is moved to $OUT/failed/ and retried after tools/wait_cool.sh. A seed that
# still fails is listed as FAILED in the summary and left out of the analysis.
set -u
ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT" || exit 1

BACKEND=${BACKEND:-gazebo}
SEEDS=${SEEDS:-"1 2 3 4 5"}
MISSION=${MISSION:-A}
MISSION_S=${MISSION_S:-180}
N=${N:-3}
RETRIES=${RETRIES:-2}
RUN_ARGS=${RUN_ARGS:-}
SKIP_RUNS=${SKIP_RUNS:-0}
if [ "$BACKEND" = fake ]; then OUT=${OUT:-run/eval_fake}; else OUT=${OUT:-runs/eval_$BACKEND}; fi
ANALYSIS_OUT=${ANALYSIS_OUT:-$OUT/analysis}
ANIM_SEED=${ANIM_SEED:-$(echo $SEEDS | awk '{print $1}')}
PY=${PYTHON:-python3}
GUARD_PID_FILE="$ROOT/run/guard.pid"
SUMMARY="$OUT/summary.txt"

say() { echo "[run_eval $(date +%H:%M:%S)] $*"; }

# ---------------------------------------------------------------- thermal guard
guard_running() {
  # run/guard.pid holds the pid as its last word ("12345" or "guard pid 12345")
  [ -f "$GUARD_PID_FILE" ] || return 1
  local pid; pid=$(awk '{print $NF}' "$GUARD_PID_FILE")
  [ -n "$pid" ] && grep -q thermal_guard "/proc/$pid/cmdline" 2>/dev/null
}

ensure_guard() {
  if guard_running; then
    say "thermal guard already running (pid $(awk '{print $NF}' "$GUARD_PID_FILE"))"
    return
  fi
  mkdir -p "$ROOT/run"
  setsid nohup bash "$ROOT/tools/thermal_guard.sh" > /dev/null 2>&1 < /dev/null &
  echo "guard pid $!" > "$GUARD_PID_FILE"
  say "started thermal guard (pid $!)"
}

wait_cool() {
  [ "$BACKEND" = fake ] && return 0
  bash "$ROOT/tools/wait_cool.sh"
}

# ---------------------------------------------------------------- runs
run_ok() {  # $1 = run dir; true if its meta.json says outcome ok and not tainted
  [ -f "$1/meta.json" ] && $PY - "$1/meta.json" <<'EOF'
import json, sys
m = json.load(open(sys.argv[1]))
sys.exit(0 if m.get("outcome") == "ok" and not m.get("tainted") else 1)
EOF
}

move_aside() {  # $1 = run dir, $2 = attempt number
  [ -e "$1" ] || return 0
  mkdir -p "$OUT/failed"
  local dest
  dest="$OUT/failed/$(basename "$1")_attempt$2_$(date +%Y%m%d-%H%M%S)"
  mv "$1" "$dest"
  say "moved failed attempt to $dest"
}

run_seed() {  # $1 = seed; sets STATUS[seed]
  local s=$1 dir="$OUT/seed_$1" attempt rc
  if run_ok "$dir"; then
    say "seed $s: already done ($dir), skipping"
    STATUS[$s]="ok (existing)"
    return
  fi
  for attempt in $(seq 0 "$RETRIES"); do
    move_aside "$dir" "$attempt"
    say "seed $s: attempt $((attempt + 1)) of $((RETRIES + 1))"
    # shellcheck disable=SC2086
    $PY run_mission.py --mission "$MISSION" --backend "$BACKEND" --n "$N" --seed "$s" \
        --mission-s "$MISSION_S" --out "$dir" $RUN_ARGS
    rc=$?
    case $rc in
      0)   STATUS[$s]="ok"; [ "$attempt" -gt 0 ] && STATUS[$s]="ok (after $attempt retries)"; return ;;
      2)   STATUS[$s]="FAILED (usage error / not implemented, exit 2)"; return ;;
      3)   STATUS[$s]="FAILED (tainted by thermal pause)" ;;
      130) STATUS[$s]="INTERRUPTED"; return ;;
      *)   STATUS[$s]="FAILED (exit $rc, see $dir/meta.json outcome)" ;;
    esac
    say "seed $s: ${STATUS[$s]}"
    [ "$attempt" -lt "$RETRIES" ] && wait_cool
  done
  # keep the last failed attempt in place for inspection
}

# ---------------------------------------------------------------- analysis
analyse() {
  local good=() s dir
  for s in $SEEDS; do
    dir="$OUT/seed_$s"
    if run_ok "$dir"; then good+=("$dir"); else say "seed $s: no valid run, left out of the analysis"; fi
  done
  if [ ${#good[@]} -eq 0 ]; then say "no valid runs to analyse"; return 1; fi
  local fail=0
  for dir in "${good[@]}"; do
    say "metrics: $dir"
    nice -n 19 $PY -m analysis.metrics "$dir" || { say "metrics FAILED on $dir"; fail=1; }
  done
  mkdir -p "$ANALYSIS_OUT"
  say "aggregate -> $ANALYSIS_OUT"
  nice -n 19 $PY -m analysis.aggregate "${good[@]}" --out "$ANALYSIS_OUT" || { say "aggregate FAILED"; fail=1; }
  say "plots -> $ANALYSIS_OUT"
  nice -n 19 $PY -m analysis.plots "${good[@]}" --out "$ANALYSIS_OUT" || { say "plots FAILED"; fail=1; }
  dir="$OUT/seed_$ANIM_SEED"
  if run_ok "$dir"; then
    say "animation of seed $ANIM_SEED -> $dir/replay.mp4"
    nice -n 19 $PY -m analysis.animate "$dir" --out "$dir/replay.mp4" || { say "animate FAILED"; fail=1; }
  else
    say "seed $ANIM_SEED has no valid run: no animation"
  fi
  return $fail
}

# ---------------------------------------------------------------- main
declare -A STATUS
mkdir -p "$OUT"
interrupted=0
if [ "$SKIP_RUNS" != 1 ]; then
  ensure_guard
  for s in $SEEDS; do
    run_seed "$s"
    if [ "${STATUS[$s]}" = INTERRUPTED ]; then interrupted=1; break; fi
  done
else
  for s in $SEEDS; do
    if run_ok "$OUT/seed_$s"; then STATUS[$s]="ok (existing)"; else STATUS[$s]="no valid run"; fi
  done
fi

analysis_rc=1
[ "$interrupted" = 0 ] && { analyse; analysis_rc=$?; }

{
  echo "evaluation $(date '+%F %T')  backend=$BACKEND mission=$MISSION mission_s=$MISSION_S n=$N"
  for s in $SEEDS; do printf '  seed %-4s %s\n' "$s" "${STATUS[$s]:-not run}"; done
  if [ "$analysis_rc" = 0 ]; then echo "  analysis: ok"; else echo "  analysis: INCOMPLETE (see output above)"; fi
  echo "  runs:     $OUT/seed_<s>/   (failed attempts: $OUT/failed/)"
  echo "  table:    $ANALYSIS_OUT/results.md"
  echo "  plots:    $ANALYSIS_OUT/*.png"
  if [ -f "$OUT/seed_$ANIM_SEED/replay.mp4" ]; then echo "  video:    $OUT/seed_$ANIM_SEED/replay.mp4"
  else echo "  video:    none (seed $ANIM_SEED has no valid run or animate failed)"; fi
} | tee "$SUMMARY"

n_bad=0
for s in $SEEDS; do case "${STATUS[$s]:-}" in ok*) ;; *) n_bad=$((n_bad + 1)) ;; esac; done
[ "$n_bad" = 0 ] && [ "$analysis_rc" = 0 ] && exit 0
exit 1
