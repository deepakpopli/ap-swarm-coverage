#!/usr/bin/env bash
# The scenario runs in results/scenarios/, one each, headless, one run at a time; each run's
# metrics.json is copied there as <name>.metrics.json.
#
#   ./run_scenarios.sh                     # gazebo, 50 m
#
# Environment: FORCE=1 (fly finished runs again).
# Raw runs: runs/scenarios/ (git-ignored). Locking and retries: tools/run_lib.sh.
# Other scenarios and Mission B: python3 run_mission.py --help.
set -u
cd "$(dirname "$0")" || exit 1
source tools/run_lib.sh

BACKEND=gazebo
FORCE=${FORCE:-0}
RUNS=runs/scenarios
RESULTS=results/scenarios

for scenario in swap obstacle agentdeath payloadfail groundloss radiooutage; do
  run_one $RUNS $scenario --mission A --scenario $scenario --n 3 --seed 1
done
run_one $RUNS C_s1 --mission C --n 3 --seed 1

mkdir -p $RESULTS
for dir in $RUNS/*/; do
  name=$(basename "$dir")
  is_done "$dir" && nice -n 19 python3 -m analysis.metrics "$dir" > /dev/null \
    && cp "$dir/metrics.json" "$RESULTS/$name.metrics.json"
done
printf '%s\n' "${SUMMARY[@]}" | tee $RESULTS/attempts.txt
