#!/usr/bin/env bash
# The evaluation in one command, headless: Mission A on seeds 1..12 (N = 3, 180 s), one no-loss
# N = 3 baseline and one no-loss N = 2 reference, one run at a time; then metrics, report,
# plots and one animation into results/<set>/.
#
#   ./run_eval.sh                          # gazebo, seeds 1..12, 50 m, 180 s: set gazebo_50m
#   BACKEND=sitl SEEDS="1 2 3 4 5" ./run_eval.sh
#
# Environment: BACKEND (gazebo | sitl), SEEDS, SWARM_ALT (cruise altitude, m), MISSION_S (s),
# FORCE=1 (fly finished runs again). Set name: <backend>_<alt>m. Raw runs go to runs/<set>/
# (git-ignored). Locking, retries and the thermal guard: tools/run_lib.sh.
set -u
cd "$(dirname "$0")" || exit 1
source tools/run_lib.sh

BACKEND=${BACKEND:-gazebo}
SEEDS=${SEEDS:-$(seq -s ' ' 1 12)}
export SWARM_ALT=${SWARM_ALT:-50}
MISSION_S=${MISSION_S:-180}
FORCE=${FORCE:-0}
case $BACKEND in gazebo | sitl) ;; *) echo "BACKEND must be gazebo or sitl"; exit 2 ;; esac
SET="${BACKEND}_${SWARM_ALT}m"
RUNS=runs/$SET
RESULTS=results/$SET
FIRST_SEED=${SEEDS%% *}

for s in $SEEDS; do
  run_one "$RUNS" "seed_$s" --mission A --n 3 --seed "$s" --mission-s "$MISSION_S"
done
run_one "$RUNS" baseline_n3 --mission ref --n 3 --seed "$FIRST_SEED" --mission-s "$MISSION_S"
run_one "$RUNS" ref_n2 --mission ref --n 2 --seed "$FIRST_SEED" --mission-s "$MISSION_S"

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
[ -d "$RESULTS" ] && cp "$RUNS/summary.txt" "$RESULTS/attempts.txt"
[ "$ANALYSIS" = ok ] && [ ${#GOOD[@]} -eq "$(echo "$SEEDS" | wc -w)" ]
