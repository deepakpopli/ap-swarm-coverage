"""Figures for a set of runs, sized to be read inside a README.

python -m analysis.plots --out results/<set> RUN_DIR... [--ref RUN_DIR...] [--baseline RUN_DIR...]
Set figures: coverage_vs_time, before_after, recovery_steps, by_drone.
One-run figures (from the first RUN_DIR): age_maps, assignments, separation.
"""
import argparse
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from analysis import metrics, report  # noqa: E402
from analysis.logs import load_run  # noqa: E402
from swarm import config  # noqa: E402

INK = "#0b0b0b"
INK_2 = "#52514e"
RUN_GREY = "#9a9993"       # one line per run: recessive
ACCENT = "#2a78d6"         # the median, the main series
ACCENT_2 = "#eb6834"       # the second reference line
CRITICAL = "#e34948"       # the loss and the 8 m limit
STALE = "#f2c14e"          # cells not seen for 60 s or more
AGENT_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#6250d6", "#e87ba4"]
DRONE_COLORS = ["#2a78d6", "#1baf7a", "#eb6834"]   # drones 1, 2 as in Gazebo (blue, green); 3 is
                                                   # black there, too grey for a series: orange

plt.rcParams.update({
    "figure.dpi": 110, "savefig.dpi": 150, "font.size": 10, "axes.edgecolor": INK_2,
    "axes.labelcolor": INK, "xtick.color": INK_2, "ytick.color": INK_2,
    "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True,
    "grid.color": "#e4e3df", "axes.titlesize": 11, "axes.titleweight": "bold",
    "axes.titlelocation": "left", "legend.frameon": False})


def load(run_dir):
    ticks, coverage = metrics.read_coverage_csv(run_dir)
    return {"m": report.add_extras(report.load_metrics(run_dir)), "t": ticks, "cov": coverage}


def median_of(runs, key):
    values = [report.lookup(r["m"], key) for r in runs]
    values = [v for v in values if v is not None]
    return float(np.median(values)) if values else None


def save(fig, out_dir, name):
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, name))
    plt.close(fig)


# ------------------------------------------------------------------- set figures

def plot_coverage(runs, refs, baselines, out_dir):
    """Coverage in absolute %, aligned on the loss, with the levels it is judged against.
    Reference lines are named in a legend below the axes, so no label sits on the curves."""
    fig, ax = plt.subplots(figsize=(8.5, 5.0))
    since_loss = np.arange(-60.0, min(r["m"]["end_s"] - r["m"]["loss"]["loss_s"] for r in runs))
    aligned = []
    for k, r in enumerate(runs):
        loss_s = r["m"]["loss"]["loss_s"]
        y = np.interp(since_loss + loss_s, r["t"], r["cov"], left=np.nan)
        ax.plot(since_loss, y, color=RUN_GREY, lw=0.9, label="one run" if k == 0 else None)
        aligned.append(y)
    ax.plot(since_loss, np.nanmedian(aligned, axis=0), color=ACCENT, lw=2.5, label="median")
    if baselines:
        ax.axhline(median_of(baselines, "coverage.mean_pct"), color=AGENT_COLORS[2], lw=1.2,
                   ls="-.", label=f"no-loss N baseline ({len(baselines)} run"
                                  f"{'s' if len(baselines) > 1 else ''})")
    before = median_of(runs, "loss.before_pct")
    ax.axhline(before, color=INK, lw=1.2, label="P (before), median")
    ax.axhline(0.9 * before, color=INK_2, lw=1.2, ls=":", label="0.9 P")
    if refs:
        ax.axhline(median_of(refs, "coverage.mean_pct"), color=ACCENT, lw=1.2, ls="--",
                   label=f"attainable with N-1 ({len(refs)} no-loss run"
                         f"{'s' if len(refs) > 1 else ''})")
    ax.axhline(median_of(runs, "loss.share_pct"), color=ACCENT_2, lw=1.2, ls="--",
               label="(N-1)/N x P")
    ax.axvline(0.0, color=CRITICAL, lw=1.2)
    ax.text(1, 56.5, "drone lost", color=CRITICAL, fontsize=8)
    declared = median_of(runs, "loss.milestones_s.lost_declared")
    if declared is not None:
        ax.axvline(declared, color=INK_2, lw=1, ls="--")
        ax.text(declared + 1, 56.5, "declared lost (median)", color=INK_2, fontsize=8)
    ax.set_xlim(since_loss[0], since_loss[-1])
    ax.set_ylim(55, 102)
    ax.set_xlabel("time since the loss (s)")
    ax.set_ylabel("area seen in the last 60 s (%)")
    ax.set_title(f"Coverage around the loss: {len(runs)} runs")
    ax.legend(fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=4)
    save(fig, out_dir, "coverage_vs_time.png")


COVERAGE_STAGES = [("before\n[L-60, L)", "loss.before_pct"), ("dip\n(min from L)", "loss.dip_pct"),
                   ("from\nL+60 s", "loss.after_mean_pct"),
                   ("after\nrecovery", "extra.after_recovery_mean_pct")]


def plot_coverage_stages(ax, runs, refs):
    """Coverage before, at the dip, from L + 60 s and after recovery, one line per run."""
    xs = np.arange(len(COVERAGE_STAGES))
    keys = [key for _, key in COVERAGE_STAGES]
    for r in runs:
        ax.plot(xs, [report.lookup(r["m"], key) for key in keys], color=RUN_GREY, lw=1,
                marker="o", ms=4)
    ax.plot(xs, [median_of(runs, key) for key in keys], color=ACCENT, lw=2.5, marker="o", ms=7)
    ax.axhline(0.9 * median_of(runs, "loss.before_pct"), color=INK_2, lw=1, ls=":")
    ax.text(xs[-1] + 0.15, 0.9 * median_of(runs, "loss.before_pct"), "0.9 P", fontsize=8,
            color=INK_2, va="bottom")
    if refs:
        attainable = median_of(refs, "coverage.mean_pct")
        ax.axhline(attainable, color=ACCENT, lw=1, ls="--")
        ax.text(xs[-1] + 0.15, attainable, "N-1 ref", fontsize=8, color=ACCENT, va="bottom")
    ax.set_xticks(xs, [label for label, _ in COVERAGE_STAGES], fontsize=8)
    ax.set_xlim(-0.3, xs[-1] + 0.9)
    ax.set_ylim(60, 100)   # (N-1)/N x P (about 65 %) is below every run: see results.md
    ax.set_title("Coverage (%)", fontsize=10)


def plot_before_after(runs, refs, out_dir):
    """Coverage through the loss, then longest unseen and churn before vs after."""
    panels = [("Worst revisit gap, longest unseen (s)", "unseen.before.worst_s",
               "unseen.after.worst_s"),
              ("Assignment churn (/agent/min)", "churn.before_per_agent_min",
               "churn.after_per_agent_min")]
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.9), gridspec_kw={"width_ratios": [1.6, 1, 1]})
    plot_coverage_stages(axes[0], runs, refs)
    for ax, (title, before_key, after_key) in zip(axes[1:], panels):
        for r in runs:
            pair = [report.lookup(r["m"], before_key), report.lookup(r["m"], after_key)]
            ax.plot([0, 1], pair, color=RUN_GREY, lw=1, marker="o", ms=5)
        medians = [median_of(runs, before_key), median_of(runs, after_key)]
        ax.plot([0, 1], medians, color=ACCENT, lw=2.5, marker="o", ms=8)
        ax.set_xticks([0, 1], ["before\n[0, L)", "from L\n[L, end]"], fontsize=8)
        ax.set_xlim(-0.3, 1.3)
        ax.set_title(title, fontsize=10)
        ax.set_ylim(bottom=0)
    fig.suptitle("Before the loss vs after it (grey: runs, blue: median)", x=0.01, ha="left",
                 fontweight="bold", fontsize=11)
    save(fig, out_dir, "before_after.png")


def plot_recovery_steps(runs, out_dir):
    """When each step of the recovery happened, per run, in seconds after the loss.

    Markers, not stacked bars: the steps do not always happen in this order (coverage can stay
    above 0.9 P throughout, so "recovered" is 0 s).
    """
    steps = [("lost_declared", "declared lost", "o"),
             ("assignment_received", "every survivor has the new assignment", "s"),
             ("on_new_loop", "first survivor on its new loop", "^"),
             ("recovered", "coverage back to 0.9 P (0 = never fell below)", "D")]
    fig, ax = plt.subplots(figsize=(8.5, 0.5 * len(runs) + 1.8))
    for row, r in enumerate(runs):
        milestones = r["m"]["loss"]["milestones_s"] or {}
        times = [milestones.get(key) for key, _, _ in steps]
        known = [t for t in times if t is not None]
        ax.plot([0, max(known)], [row, row], color=RUN_GREY, lw=1)
        for (key, label, marker), t, color in zip(steps, times, AGENT_COLORS):
            if t is not None:
                ax.plot(t, row, marker, color=color, ms=8, label=label if row == 0 else None)
        if times[3] is None:
            ax.text(max(known) + 1, row, "not recovered", va="center", fontsize=8, color=CRITICAL)
    ax.set_yticks(range(len(runs)), [r["m"]["run"] for r in runs])
    ax.set_xlabel("seconds after the loss")
    ax.set_title("Recovery, step by step")
    ax.legend(fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.25), ncol=2)
    save(fig, out_dir, "recovery_steps.png")


KILL_TIME_RAMP = ["#86b6ef", "#3987e5", "#1c5cab", "#0d366b"]   # one hue, light = early


def kill_time_color(loss_s):
    """Fault time 60 .. 90 s on the blue ramp: light early, dark late."""
    k = (min(max(loss_s, 60.0), 90.0) - 60.0) / 30.0 * (len(KILL_TIME_RAMP) - 1)
    return KILL_TIME_RAMP[int(round(k))]


def plot_by_drone(runs, out_dir):
    """Which drone failed and when: its runs' coverage over the mission, then the lowest
    coverage and the coverage 60 s on against the time of the fault, one line per drone."""
    drones = sorted({r["m"]["victim"] for r in runs})
    fig = plt.figure(figsize=(11, 7.2))
    grid = fig.add_gridspec(2, len(drones), height_ratios=[1, 1.05])
    for col, drone in enumerate(drones):
        ax = fig.add_subplot(grid[0, col])
        mine = sorted((r for r in runs if r["m"]["victim"] == drone),
                      key=lambda r: r["m"]["loss"]["loss_s"])
        for r in mine:
            loss_s, color = r["m"]["loss"]["loss_s"], kill_time_color(r["m"]["loss"]["loss_s"])
            ax.plot(r["t"], r["cov"], color=color, lw=1.4, label=f"fails at {loss_s:.0f} s")
            ax.plot(loss_s, np.interp(loss_s, r["t"], r["cov"]), "v", color=color, ms=8,
                    mec="white", mew=1.5)
        ax.set_xlim(0, max(r["m"]["end_s"] for r in runs))
        ax.set_ylim(55, 102)
        ax.set_title(f"Drone {drone} fails ({len(mine)} run{'s' if len(mine) > 1 else ''})")
        ax.set_xlabel("mission time (s)")
        if col == 0:
            ax.set_ylabel("area seen in the last 60 s (%)")
        ax.legend(fontsize=7, loc="lower left")
    ax = fig.add_subplot(grid[1, :])
    labels = []                                    # (y, text, colour) at the right end
    for drone in drones:
        color = DRONE_COLORS[(drone - 1) % len(DRONE_COLORS)]
        mine = sorted((r for r in runs if r["m"]["victim"] == drone),
                      key=lambda r: r["m"]["loss"]["loss_s"])
        at = [r["m"]["loss"]["loss_s"] for r in mine]
        dip = [r["m"]["loss"]["dip_pct"] for r in mine]
        after = [r["m"]["loss"]["after_mean_pct"] for r in mine]
        ax.plot(at, after, "-o", color=color, lw=2, ms=8, mec="white", mew=1.5)
        ax.plot(at, dip, "--v", color=color, lw=1.2, ms=8, mec="white", mew=1.5)
        labels.append([after[-1], f"drone {drone}", at[-1]])
    labels.sort()
    for below, above in zip(labels, labels[1:]):  # keep the labels 1.5 points apart
        above[0] = max(above[0], below[0] + 1.5)
    for y, text, x in labels:
        ax.annotate(text, (x, y), xytext=(10, 0), textcoords="offset points", va="center",
                    fontsize=9, color=INK)
    ax.plot([], [], "-o", color=INK_2, label="mean coverage from 60 s after the fault to the end")
    ax.plot([], [], "--v", color=INK_2, label="lowest coverage after the fault")
    ax.set_xlabel("time of the fault (mission s)")
    ax.set_ylabel("area seen in the last 60 s (%)")
    ax.set_title("Coverage after the fault, by drone and by when it fails")
    ax.legend(fontsize=8, loc="lower left")
    ax.set_xlim(57, 97)
    save(fig, out_dir, "by_drone.png")


# ------------------------------------------------------------------- one-run figures

def plot_age_maps(run, out_dir):
    """How long ago each cell was seen, just before the loss and 60 s after it."""
    times = [run.loss_s - 1.0, run.loss_s + metrics.WINDOW_S]
    ages = [(t - last).reshape(metrics.GRID_SHAPE)
            for t, last in metrics.last_seen_at(metrics.sightings(run), times)]
    colors = plt.get_cmap("Blues").with_extremes(over=STALE)
    fig, axes = plt.subplots(1, 2, figsize=(9, 4.4))
    for ax, t, age in zip(axes, times, ages):
        image = ax.imshow(age, origin="lower", extent=config.AREA, cmap=colors, vmin=0,
                          vmax=metrics.WINDOW_S, interpolation="nearest")
        ax.set_title(f"t = {t:.0f} s ({t - run.loss_s:+.0f} s from the loss)", fontsize=10)
        ax.set_xlabel("east (m)")
        ax.grid(False)
    axes[0].set_ylabel("north (m)")
    bar = fig.colorbar(image, ax=list(axes), extend="max", shrink=0.85)
    bar.set_label("s since last seen (yellow: 60+ or never)")
    fig.suptitle(f"Run {run.name}: drone {run.victim} lost at {run.loss_s:.1f} s", x=0.01,
                 ha="left", fontweight="bold", fontsize=11)
    fig.savefig(os.path.join(out_dir, "age_maps.png"), bbox_inches="tight")
    plt.close(fig)


def assigned_from(event):
    """Mission time an assignment holds from; one made before the start holds from t = 0."""
    if event["t_mission"] is None:
        return 0.0
    return max(0.0, event["t_mission"])


def assignment_steps(run):
    """[(from t, {agent id: share})] from the ground's assign events."""
    steps = []
    for e in run.events("assign"):
        shares = {int(k): a["share"] for k, a in e["assignments"].items()}
        steps.append((assigned_from(e), shares))
    return steps


def plot_assignments(run, out_dir):
    """Each agent's share (east and north extent) over time: flat bands = no oscillation."""
    steps = assignment_steps(run)
    fig, axes = plt.subplots(2, 1, figsize=(8.5, 5.6), sharex=True)
    for agent_id in run.agents:
        color = DRONE_COLORS[(agent_id - 1) % len(DRONE_COLORS)]
        for k, (start, shares) in enumerate(steps):
            if agent_id not in shares:
                continue
            end = steps[k + 1][0] if k + 1 < len(steps) else run.end_s
            xmin, xmax, ymin, ymax = shares[agent_id]
            label = f"drone {agent_id}" if k == 0 else None
            axes[0].fill_between([start, end], xmin, xmax, color=color, alpha=0.35, lw=0,
                                 label=label)
            axes[1].plot([start, end], [ymin, ymin], color=color, lw=2)   # shares often span
            axes[1].plot([start, end], [ymax, ymax], color=color, lw=2)   # all of north: lines
    for ax in axes:
        for start, _ in steps[1:]:
            ax.axvline(start, color=INK_2, lw=0.8, ls=":")
        if run.loss_s is not None:
            ax.axvline(run.loss_s, color=CRITICAL, lw=1.2)
            declared = run.lost_declared_s(run.victim)
            if declared is not None:
                ax.axvline(declared, color=CRITICAL, lw=1, ls="--")
    for e in run.events("infeasible"):
        axes[0].plot(e["t_mission"], e["slot_xy"][0], "v", color=INK, ms=7)
        axes[1].plot(e["t_mission"], e["slot_xy"][1], "v", color=INK, ms=7)
    for e in run.events("assign")[1:]:
        axes[0].text(assigned_from(e), config.AREA[1], f" new plan: {e['trigger']}", fontsize=7, color=INK_2, rotation=90, va="top")
    axes[0].set_ylabel("share, east (m)")
    axes[1].set_ylabel("share, north (m)")
    axes[1].set_xlabel("mission time (s). Red: loss; red dashed: declared lost; dotted: re-plan;"
                       " triangle: infeasible report")
    axes[0].legend(fontsize=8, loc="upper left", bbox_to_anchor=(1.0, 1.0))
    axes[0].set_title(f"Run {run.name}: each drone's share over time")
    axes[1].set_xlim(0, run.end_s)
    save(fig, out_dir, "assignments.png")


def plot_separation(run, out_dir):
    """3D distance of every pair (EKF positions) from t = 10 s, on an axis from 0 that holds
    the real distances (tens of metres) and the 8 m limit."""
    times, pairs = metrics.separation_series(run)
    fig, ax = plt.subplots(figsize=(8.5, 3.8))
    for color, ((a, b), distance) in zip(AGENT_COLORS, pairs.items()):
        ax.plot(times, distance, color=color, lw=1.2, label=f"drones {a}-{b}")
    for agent_id, log in run.agents.items():
        filtered = np.array(["filtered" in flags.split("|") for flags in log["flags"]], dtype=bool)
        filtered &= metrics.mission_rows(run, agent_id)
        ax.plot(log["t"][filtered], np.full(filtered.sum(), 1.0), "|", color=INK, ms=8)
    ax.axhline(config.D_MIN_M, color=CRITICAL, lw=1.5)
    ax.text(metrics.SAFETY_FROM_S + 1, config.D_MIN_M + 1.5, "8 m minimum", color=CRITICAL,
            fontsize=8)
    closest = np.fmin.reduce(np.array(list(pairs.values())), axis=0)
    shown = times >= metrics.SAFETY_FROM_S   # the 10 m start line before is shown in results.md
    highest = np.nanmax(np.array([d[shown] for d in pairs.values()]))
    ax.set_xlim(metrics.SAFETY_FROM_S, run.end_s)
    ax.set_ylim(0, 10 * np.ceil(highest / 10) + 10)
    ax.set_xlabel("mission time (s), from t = 10 s")
    ax.set_ylabel("distance (m)")
    ax.set_title(f"Run {run.name}: closest pair from t = 10 s "
                 f"{np.nanmin(closest[times > metrics.SAFETY_FROM_S]):.1f} m (whole log, with "
                 f"the start line: {np.nanmin(closest):.1f} m)", fontsize=10)
    ax.legend(fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=4,
              title="ticks at 1 m: safety filter active", title_fontsize=8)
    save(fig, out_dir, "separation.png")


def main():
    parser = argparse.ArgumentParser(description="figures for a set of runs")
    parser.add_argument("runs", nargs="+")
    parser.add_argument("--out", required=True)
    parser.add_argument("--ref", nargs="*", default=[])
    parser.add_argument("--baseline", nargs="*", default=[])
    args = parser.parse_args()
    runs = [load(d) for d in args.runs]
    refs = [load(d) for d in args.ref]
    baselines = [load(d) for d in args.baseline]
    report.refuse_fake_in_results(args.out, {r["m"]["backend"] for r in runs + refs + baselines})
    os.makedirs(args.out, exist_ok=True)
    first = load_run(args.runs[0])
    if first.victim is not None:
        plot_coverage(runs, refs, baselines, args.out)
        plot_before_after(runs, refs, args.out)
        plot_recovery_steps(runs, args.out)
        plot_by_drone(runs, args.out)
        plot_age_maps(first, args.out)
    plot_assignments(first, args.out)
    plot_separation(first, args.out)
    print(f"figures -> {args.out}/")


if __name__ == "__main__":
    main()
