# Provenance of results/gazebo_50m

## What was flown

The runs were flown from the frozen evaluation copy `~/Documents/Projects/ap-swarm-eval-snapshot`
(runs in its `runs/gazebo_50m/`: seed_1..12, baseline_n3, ref_n2).

- Snapshot HEAD: `3c154f8d8baa783116b3ac34d51794c4f954770e` (plus the uncommitted changes in its
  `EVAL-SNAPSHOT.diff`, which equals `git diff` there today).
- EVAL-SNAPSHOT.txt records: frozen 2026-11-22 03:36:42, `code sha256 (tracked+untracked, sorted):
  dd608d20505c4692b8b3077497fad2a25e16b0775d866124808708a6d30bbeb8`. It records the hash, not the
  command that made it.

## The code hash

Five scripts were added to the snapshot after the freeze (03:36:42) and are not part of the
evaluated code: `run_experiments.sh` (03:38:00), `run_fleet.sh` (12:51:35), `run_40m.sh`
(13:32:38), `run_ablation.sh` (13:43:47) and `run_scen.sh` (13:43:55). These are all the
`run_*.sh` files newer than EVAL-SNAPSHOT.txt, and no other hashed file is newer.

VERIFIED: the recorded hash dd608d... reproduces exactly with this command, run in the snapshot
with every post-freeze script left out (checked on 22 Nov):

```bash
cd ~/Documents/Projects/ap-swarm-eval-snapshot
git ls-files -co --exclude-standard | grep -E '\.(py|sh|xml|sdf|parm|toml)$' \
  | grep -v -E '^run_(experiments|ablation|fleet|40m|scen)\.sh$' | sort | xargs sha256sum | sha256sum
# dd608d20505c4692b8b3077497fad2a25e16b0775d866124808708a6d30bbeb8  -
```

The same, without naming the scripts (it leaves out any `run_*.sh` newer than the freeze file,
so it keeps working if more are added):

```bash
cd ~/Documents/Projects/ap-swarm-eval-snapshot
git ls-files -co --exclude-standard | grep -E '\.(py|sh|xml|sdf|parm|toml)$' \
  | grep -vxF -f <(find . -maxdepth 1 -name 'run_*.sh' -newer EVAL-SNAPSHOT.txt -printf '%P\n') \
  | sort | xargs sha256sum | sha256sum
# dd608d20505c4692b8b3077497fad2a25e16b0775d866124808708a6d30bbeb8  -
```

Also, no tracked or untracked file in the snapshot, other than these scripts, the logs, the `run` symlink,
`results/` and `EVAL-SNAPSHOT.diff` (written 14 ms after the .txt), has a modification time
after the freeze. I don't give a hash of the snapshot "as it is now": it changes every time a
run script is added there after the freeze, so a number would go stale. Use the command above.

## Untracked code in the snapshot

The snapshot's untracked files (43: code, models, textures, notes, the two post-freeze scripts that existed then,
EVAL-SNAPSHOT.txt/.diff; not `results/`, logs or the `run` symlink) are saved in
`~/Documents/Projects/ap-swarm-coverage/run/eval_snapshot_untracked.tar.gz`:

```bash
cd ~/Documents/Projects/ap-swarm-eval-snapshot
git ls-files -o --exclude-standard | grep -Ev '^(results/|run$)|\.log$' | LC_ALL=C sort > /tmp/snap_untracked.txt
tar --sort=name --owner=0 --group=0 --numeric-owner -cf - -T /tmp/snap_untracked.txt | gzip -n \
  > ~/Documents/Projects/ap-swarm-coverage/run/eval_snapshot_untracked.tar.gz
```

sha256 `a598716ca81f5c38571b2b0a821a11d0856683bd11e8d344956bf21ea31b5515`.

## How these results were made

results.md, results.json, runs/*.json and the PNGs here were regenerated on 2026-11-22 with a newer
`analysis/report.py` and `analysis/plots.py` from the main repo (ap-swarm-coverage). Analysis code
only: the flown code is unchanged, the runs' metrics.json files are the snapshot's own, and every
other module (analysis/metrics.py, analysis/logs.py, swarm/) comes from the snapshot, in a scratch
copy. Nothing is written into the snapshot. attempts.txt is the snapshot's runs/gazebo_50m/summary.txt.
The replay video is not regenerated (see the snapshot's results/gazebo_50m/replay_seed_1.mp4).

References: only baseline_n3 and ref_n2 existed at regeneration time; the extra reference runs
(baseline_n3_s2, baseline_n3_s3, ref_n2_s2..s4) were not in the snapshot yet. The command below
picks up every finished `baseline_n3*` / `ref_n2*` run, so running it again includes them.

Re-runnable command:

```bash
#!/usr/bin/env bash
# Regenerate results/gazebo_50m from the frozen snapshot's runs with the main repo's report.py and
# plots.py. The snapshot's own analysis code (metrics.py, logs.py, swarm/) is used for everything
# else, in a scratch copy; nothing is written into the snapshot.
set -eu
SNAP=$HOME/Documents/Projects/ap-swarm-eval-snapshot
MAIN=$HOME/Documents/Projects/ap-swarm-coverage
R=$SNAP/runs/gazebo_50m
OUT=$MAIN/results/gazebo_50m
W=$(mktemp -d)
trap 'rm -rf "$W"' EXIT
rsync -a --exclude .git --exclude runs --exclude results --exclude run "$SNAP"/ "$W"/
cp "$MAIN"/analysis/report.py "$MAIN"/analysis/plots.py "$W"/analysis/
done_ok() {   # finished runs only (meta outcome ok); one without metrics.json is analysed in a copy
  for d in "$@"; do
    [ -d "$d" ] || continue
    python3 -c 'import json,sys; sys.exit(json.load(open(sys.argv[1]+"/meta.json")).get("outcome") != "ok")' "$d" 2> /dev/null || continue
    if [ -f "$d/metrics.json" ]; then echo "$d"; continue; fi
    mkdir -p "$W/copies" && cp -r "$d" "$W/copies/" && (cd "$W" && python3 -m analysis.metrics "$W/copies/$(basename "$d")" > /dev/null) && echo "$W/copies/$(basename "$d")"
  done
}
mapfile -t RUNS < <(done_ok $(for s in $(seq 1 12); do echo "$R/seed_$s"; done))
mapfile -t REF < <(done_ok "$R"/ref_n2*)
mapfile -t BASE < <(done_ok "$R"/baseline_n3*)
echo "runs ${#RUNS[@]}, ref: ${REF[*]##*/}, baseline: ${BASE[*]##*/}"
mkdir -p "$OUT"
cd "$W"
python3 -m analysis.report --out "$OUT" --name gazebo_50m "${RUNS[@]}" --ref "${REF[@]}" --baseline "${BASE[@]}"
python3 -m analysis.plots --out "$OUT" "${RUNS[@]}" --ref "${REF[@]}" --baseline "${BASE[@]}"
cp "$R"/summary.txt "$OUT"/attempts.txt
```
