"""Figures for a set of runs, sized to be read inside a README.

python -m analysis.plots --out results/<set> RUN_DIR... [--ref RUN_DIR...] [--baseline RUN_DIR...]
Set figures: coverage_vs_time, before_after, recovery_steps.
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

plt.rcParams.update({
    "figure.dpi": 110, "savefig.dpi": 150, "font.size": 10, "axes.edgecolor": INK_2,
    "axes.labelcolor": INK, "xtick.color": INK_2, "ytick.color": INK_2,
    "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True,
    "grid.color": "#e4e3df", "axes.titlesize": 11, "axes.titleweight": "bold",
    "axes.titlelocation": "left", "legend.frameon": False})


def load(run_dir):
    ticks, coverage = metrics.read_coverage_csv(run_dir)
    return {"m": report.load_metrics(run_dir), "t": ticks, "cov": coverage}


def median_of(runs, key):
    values = [report.lookup(r["m"], key) for r in runs]
    values = [v for v in values if v is not None]
    return float(np.median(values)) if values else None


def save(fig, out_dir, name):
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, name))
    plt.close(fig)


def reference_line(ax, y, color, style, text):
    ax.axhline(y, color=color, lw=1.2, ls=style)
    ax.text(ax.get_xlim()[1], y, " " + text, color=color, fontsize=8, va="center")


# ------------------------------------------------------------------- set figures

def plot_coverage(runs, refs, baselines, out_dir):
    """Coverage in absolute %, aligned on the loss, with the levels it is judged against."""
    fig, ax = plt.subplots(figsize=(8.5, 4.4))
    since_loss = np.arange(-60.0, min(r["m"]["end_s"] - r["m"]["loss"]["loss_s"] for r in runs))
    aligned = []
    for r in runs:
        loss_s = r["m"]["loss"]["loss_s"]
        y = np.interp(since_loss + loss_s, r["t"], r["cov"], left=np.nan)
        ax.plot(since_loss, y, color=RUN_GREY, lw=0.9)
        aligned.append(y)
    ax.plot(since_loss, np.nanmedian(aligned, axis=0), color=ACCENT, lw=2.5)
    ax.set_xlim(since_loss[0], since_loss[-1])
    if baselines:
        levels = [b["m"]["coverage"]["mean_pct"] for b in baselines]
        ax.axhspan(min(levels), max(levels), color=ACCENT, alpha=0.12, lw=0)
        ax.text(since_loss[0] + 1, max(levels) + 1, "no-loss runs with N agents", color=INK_2,
                fontsize=8)
    before = median_of(runs, "loss.before_pct")
    reference_line(ax, before, INK, "-", "P (before)")
    reference_line(ax, 0.9 * before, INK_2, ":", "0.9 P")
    if refs:
        reference_line(ax, median_of(refs, "coverage.mean_pct"), ACCENT, "--",
                       "attainable N-1")
    reference_line(ax, median_of(runs, "loss.share_pct"), ACCENT_2, "--", "(N-1)/N x P")
    ax.axvline(0.0, color=CRITICAL, lw=1.2)
    ax.text(1, 2, "agent lost", color=CRITICAL, fontsize=8)
    declared = median_of(runs, "loss.milestones_s.lost_declared")
    if declared is not None:
        ax.axvline(declared, color=INK_2, lw=1, ls="--")
        ax.text(declared + 1, 9, "declared lost\n(median)", color=INK_2, fontsize=8)
    ax.set_ylim(0, 105)
    ax.set_xlabel("time since the loss (s)")
    ax.set_ylabel("area seen in the last 60 s (%)")
    ax.set_title(f"Coverage around the loss: {len(runs)} runs (grey), median (blue)")
    save(fig, out_dir, "coverage_vs_time.png")


def plot_before_after(runs, refs, out_dir):
    """One line per run from before to after; the median in blue."""
    panels = [("Coverage, mean (%)", "loss.before_pct", "loss.after_mean_pct"),
              ("Worst revisit gap, longest unseen (s)", "unseen.before.worst_s",
               "unseen.after.worst_s"),
              ("Assignment churn (/agent/min)", "churn.before_per_agent_min",
               "churn.after_per_agent_min")]
    fig, axes = plt.subplots(1, 3, figsize=(10, 3.8))
    for ax, (title, before_key, after_key) in zip(axes, panels):
        for r in runs:
            pair = [report.lookup(r["m"], before_key), report.lookup(r["m"], after_key)]
            ax.plot([0, 1], pair, color=RUN_GREY, lw=1, marker="o", ms=5)
        medians = [median_of(runs, before_key), median_of(runs, after_key)]
        ax.plot([0, 1], medians, color=ACCENT, lw=2.5, marker="o", ms=8)
        ax.set_xticks([0, 1], ["before", "after"])
        ax.set_xlim(-0.3, 1.6)
        ax.set_title(title, fontsize=10)
        ax.set_ylim(bottom=0)
    coverage_ax = axes[0]
    reference_line(coverage_ax, median_of(runs, "loss.share_pct"), ACCENT_2, "--", "(N-1)/N P")
    if refs:
        reference_line(coverage_ax, median_of(refs, "coverage.mean_pct"), ACCENT, "--",
                       "N-1")
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
             ("recovered", "coverage back to 0.9 P", "D")]
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
    fig.suptitle(f"Run {run.name}: agent {run.victim} lost at {run.loss_s:.1f} s", x=0.01,
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
    for color, agent_id in zip(AGENT_COLORS, run.agents):
        for k, (start, shares) in enumerate(steps):
            if agent_id not in shares:
                continue
            end = steps[k + 1][0] if k + 1 < len(steps) else run.end_s
            xmin, xmax, ymin, ymax = shares[agent_id]
            label = f"agent {agent_id}" if k == 0 else None
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
        axes[0].text(assigned_from(e), config.AREA[1], f" {e['trigger']} (epoch "
                     f"{e['epoch']})", fontsize=7, color=INK_2, rotation=90, va="top")
    axes[0].set_ylabel("share, east (m)")
    axes[1].set_ylabel("share, north (m)")
    axes[1].set_xlabel("mission time (s). Red: loss; red dashed: declared lost; dotted: re-plan;"
                       " triangle: infeasible report")
    axes[0].legend(fontsize=8, loc="upper left", bbox_to_anchor=(1.0, 1.0))
    axes[0].set_title(f"Run {run.name}: each agent's share over time")
    axes[1].set_xlim(0, run.end_s)
    save(fig, out_dir, "assignments.png")


def plot_separation(run, out_dir):
    """True 3D distance of every pair; zoomed to 0-40 m so the 8 m limit is readable."""
    times, pairs = metrics.separation_series(run)
    fig, ax = plt.subplots(figsize=(8.5, 3.8))
    for color, ((a, b), distance) in zip(AGENT_COLORS, pairs.items()):
        ax.plot(times, distance, color=color, lw=1.2, label=f"agents {a}-{b}")
    for agent_id, log in run.agents.items():
        filtered = np.array(["filtered" in flags.split("|") for flags in log["flags"]], dtype=bool)
        filtered &= metrics.mission_rows(run, agent_id)
        ax.plot(log["t"][filtered], np.full(filtered.sum(), 1.0), "|", color=INK, ms=8)
    ax.axhline(config.D_MIN_M, color=CRITICAL, lw=1.5)
    ax.text(times[0], config.D_MIN_M + 0.5, "8 m minimum", color=CRITICAL, fontsize=8)
    closest = np.fmin.reduce(np.array(list(pairs.values())), axis=0)
    ax.set_ylim(0, 40)
    ax.set_xlabel("mission time (s; negative = before the start)")
    ax.set_ylabel("distance (m)")
    ax.set_title(f"Run {run.name}: closest pair {np.nanmin(closest):.1f} m "
                 f"(from t = 10 s: {np.nanmin(closest[times > metrics.SAFETY_FROM_S]):.1f} m); "
                 "ticks at 1 m: safety filter active")
    ax.legend(fontsize=8, loc="upper right")
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
        plot_age_maps(first, args.out)
    plot_assignments(first, args.out)
    plot_separation(first, args.out)
    print(f"figures -> {args.out}/")


if __name__ == "__main__":
    main()
