#!/usr/bin/env bash
# Every scenario once, headless, one run at a time, then a one-table summary of what each
# should show and what was measured: results/scenarios/summary.md (analysis/scenarios.py).
#
#   ./run_scenarios.sh                     # gazebo, 50 m
#
# Environment: BACKEND (gazebo | sitl), SWARM_ALT (m), FORCE=1 (fly finished runs again).
# Raw runs: runs/scenarios/ (git-ignored). Locking, retries, thermal guard: tools/run_lib.sh.
# Seeds are chosen for their victim (run_mission.plan): in Mission B the zone owner is agent 2
# (middle) or 1 (corner); seed 1 kills agent 1, seed 3 agent 3, seed 8 agent 2. N = 4 and 5:
# seed 3 kills agent 2 and 5, never the N = 5 spare (agent 3).
set -u
cd "$(dirname "$0")" || exit 1
source tools/run_lib.sh

BACKEND=${BACKEND:-gazebo}
export SWARM_ALT=${SWARM_ALT:-50}
FORCE=${FORCE:-0}
case $BACKEND in gazebo | sitl) ;; *) echo "BACKEND must be gazebo or sitl"; exit 2 ;; esac
RUNS=runs/scenarios
RESULTS=results/scenarios
CORNER=0,0,60,60

for scenario in swap obstacle; do
  run_one $RUNS $scenario --mission A --scenario $scenario --n 3 --seed 1
done
run_one $RUNS B_middle --mission B --n 3 --seed 1
run_one $RUNS B_middle_owner --mission B --n 3 --seed 8
run_one $RUNS B_corner --mission B --n 3 --seed 3 --zone $CORNER
run_one $RUNS B_corner_owner --mission B --n 3 --seed 1 --zone $CORNER
for s in 1 2 3; do run_one $RUNS "C_s$s" --mission C --n 3 --seed "$s"; done
run_one $RUNS blackout13 --mission C --scenario blackout13 --n 3 --seed 1
for scenario in mute agentdeath groundrestart groundloss radiooutage; do
  run_one $RUNS $scenario --mission A --scenario $scenario --n 3 --seed 1
done
run_one $RUNS A_n5 --mission A --n 5 --seed 3
run_one $RUNS A_n4 --mission A --n 4 --seed 3

for dir in $RUNS/*/; do
  is_done "$dir" && nice -n 19 python3 -m analysis.metrics "$dir" > /dev/null
done
mkdir -p $RESULTS
nice -n 19 python3 -m analysis.scenarios $RUNS $RESULTS
printf '%s\n' "${SUMMARY[@]}" | tee $RESULTS/attempts.txt
