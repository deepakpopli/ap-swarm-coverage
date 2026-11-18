"""Summary plots (PNG) across analysed runs.

Usage: python -m analysis.plots runs/a runs/b ... [--out results]
Needs each run's metrics.json, coverage.csv, separation.csv, gapmaps.npz (from analysis.metrics;
computed here if missing). Writes into --out:
  coverage_vs_time.png    coverage / pre-loss value vs time since the loss, all runs
  revisit_gap.png         worst revisit gap and worst staleness, before vs after, per run
  recovery_churn.png      recovery times (a)/(b) and assignment churn per run
  gap_map.png             per-cell max revisit gap, pre-loss and post-recovery, representative run
  min_separation.png      min true 3D separation between live agents vs mission time
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analysis import aggregate, metrics

INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#e4e3df"
RUN = "#9a9993"          # individual runs: recessive grey
ACCENT = "#2a78d6"       # median / highlighted series (palette slot 1)
ACCENT2 = "#eb6834"      # second series (palette slot 2)
CRIT = "#e34948"         # reserved: limits (8 m line), kill marker

plt.rcParams.update({
    "figure.dpi": 110, "savefig.dpi": 150, "font.size": 10, "axes.edgecolor": INK2,
    "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2, "axes.spines.top": False,
    "axes.spines.right": False, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.titlesize": 11, "axes.titleweight": "bold", "axes.titlelocation": "left",
    "legend.frameon": False})


def read_csv_cols(path):
    with open(path) as f:
        r = list(csv.reader(f))
    data = np.array([[float(v) if v else np.nan for v in row] for row in r[1:]])
    return data[:, 0], data[:, 1]


def load(run_dir):
    m = aggregate.load_metrics(run_dir)
    t, cov = read_csv_cols(os.path.join(run_dir, "coverage.csv"))
    ts, sep = read_csv_cols(os.path.join(run_dir, "separation.csv"))
    maps = dict(np.load(os.path.join(run_dir, "gapmaps.npz")))
    return {"m": m, "t": t, "cov": cov, "ts": ts, "sep": sep, "maps": maps}


def title_suffix(runs):
    synth = any(aggregate.is_synthetic(r["m"]) for r in runs)
    return f" (n = {len(runs)} runs{', SYNTHETIC' if synth else ''})"


def plot_coverage(runs, out):
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    aligned = []
    n_agents = runs[0]["m"]["n_agents"]
    ideal = 100.0 * (n_agents - 1) / n_agents
    for r in runs:
        L, P = r["m"]["t_loss_s"], (r["m"].get("loss") or {}).get("pre_loss_pct")
        if L is None or not P:
            continue
        x, y = r["t"] - L, 100.0 * r["cov"] / P
        ax.plot(x, y, color=RUN, lw=1.0)
        aligned.append(np.interp(np.arange(-60, 91), x, y, left=np.nan, right=np.nan))
    if aligned:
        ax.plot(np.arange(-60, 91), np.nanmedian(np.array(aligned), axis=0), color=ACCENT, lw=2,
                label="median of runs")
    ax.axhspan(90, 100, color=ACCENT, alpha=0.08, lw=0)
    ax.axhspan(0.9 * ideal, ideal, color=ACCENT2, alpha=0.10, lw=0)
    ax.axhline(100, color=INK2, lw=1)
    ax.axhline(ideal, color=ACCENT2, lw=1.5, ls="--")
    ax.axvline(0, color=CRIT, lw=1.2)
    delays = [r["m"]["loss"]["detection_delay_s"] for r in runs
              if r["m"].get("loss") and r["m"]["loss"].get("detection_delay_s") is not None]
    if delays:
        ax.axvline(float(np.median(delays)), color=INK2, lw=1, ls=":")
        ax.text(float(np.median(delays)) + 1, 5, "ground declares lost\n(median)", color=INK2, fontsize=8)
    ax.text(1, 104, "agent killed", color=CRIT, fontsize=8)
    ax.text(-59, 101.5, "pre-loss value; band = within 10 % (reading a)", color=INK2, fontsize=8)
    ax.text(-59, ideal + 1.5, f"ideal (N-1)/N = {ideal:.0f} %; band = within 10 % (reading b)",
            color=ACCENT2, fontsize=8)
    ax.set_xlim(-60, 90)
    ax.set_ylim(0, 110)
    ax.set_xlabel("time since agent loss (s)")
    ax.set_ylabel("coverage, % of pre-loss value")
    ax.set_title("Area seen in the last 60 s, aligned on the loss" + title_suffix(runs))
    if aligned:
        ax.plot([], [], color=RUN, lw=1, label="individual runs")
        ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "coverage_vs_time.png"))
    plt.close(fig)


def _paired(ax, runs, key, label):
    for r in runs:
        g = r["m"].get("revisit_gap") or {}
        a, b = (g.get("pre") or {}).get(key), (g.get("post") or {}).get(key)
        if a is not None and b is not None:
            ax.plot([0, 1], [a, b], color=RUN, lw=1, marker="o", ms=5)
        elif a is not None:
            ax.plot([0], [a], color=RUN, marker="o", ms=5)
    vals = [[(r["m"].get("revisit_gap") or {}).get(p) for r in runs] for p in ("pre", "post")]
    med = [np.median([v[key] for v in vs if v and v.get(key) is not None]) if any(v and v.get(key) is not None for v in vs) else np.nan
           for vs in vals]
    ax.plot([0, 1], med, color=ACCENT, lw=2.2, marker="o", ms=8, label="median")
    ax.set_xticks([0, 1], ["before the loss", "after recovery"])
    ax.set_xlim(-0.3, 1.3)
    ax.set_ylim(bottom=0)
    ax.set_ylabel("seconds")
    ax.set_title(label)


def plot_gaps(runs, out):
    fig, axs = plt.subplots(1, 2, figsize=(8, 3.8))
    _paired(axs[0], runs, "worst_s", "Worst revisit gap (within phase)")
    _paired(axs[1], runs, "worst_age_s", "Worst staleness (unclipped age)")
    axs[1].legend(loc="lower right", fontsize=8)
    for ax in axs:
        ax.axhline(metrics.WINDOW_S, color=INK2, lw=1, ls=":")
        top = max([metrics.WINDOW_S] + [y for ln in ax.get_lines() for y in ln.get_ydata() if np.isfinite(y)])
        ax.set_ylim(0, top * 1.12)
        ax.text(1.28, metrics.WINDOW_S + 1, "60 s window", color=INK2, fontsize=8, ha="right")
    fig.suptitle("Longest time a cell went unseen" + title_suffix(runs), x=0.01, ha="left",
                 fontweight="bold", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "revisit_gap.png"))
    plt.close(fig)


def plot_recovery_churn(runs, out):
    fig, axs = plt.subplots(1, 2, figsize=(8.5, 3.8))
    ax = axs[0]
    names = [r["m"]["run"] for r in runs]
    ymax = max([1.0] + [v for r in runs for v in ((r["m"].get("loss") or {}).get("recovery_a_s"),
                                                   (r["m"].get("loss") or {}).get("recovery_b_s")) if v is not None])
    for k, (key, color, dx, lab) in enumerate((("recovery_a_s", ACCENT, -0.12, "(a) within 10 % of pre-loss"),
                                              ("recovery_b_s", ACCENT2, 0.12, "(b) within 10 % of (N-1)/N ideal"))):
        for i, r in enumerate(runs):
            v = (r["m"].get("loss") or {}).get(key)
            if v is None:
                ax.text(i + dx, ymax * 1.05, "x", color=color, ha="center", va="center", fontsize=11)
            else:
                ax.plot(i + dx, v, "o", color=color, ms=7, label=lab if i == 0 else None)
        ax.plot([], [], "o", color=color, label=lab) if all((r["m"].get("loss") or {}).get(key) is None for r in runs) else None
    ax.set_xticks(range(len(runs)), names, rotation=30, ha="right", fontsize=8)
    ax.set_ylim(-0.04 * ymax, ymax * 1.15)
    ax.set_ylabel("seconds after the loss")
    ax.set_title("Recovery time (x = not recovered)")
    ax.legend(fontsize=8, loc="upper left")

    ax = axs[1]
    phases = ("pre", "post", "overall")
    for j, (key, color, lab) in enumerate((("per_agent_min_all", RUN, "all events"),
                                           ("per_agent_min_excl", ACCENT, "excl. initial + loss re-plan"))):
        for i, ph in enumerate(phases):
            vals = [((r["m"].get("churn") or {}).get(ph) or {}).get(key) for r in runs]
            vals = [v for v in vals if v is not None]
            x = i + (j - 0.5) * 0.25
            ax.plot([x] * len(vals), vals, "o", color=color, ms=5, alpha=0.8,
                    label=lab if i == 0 else None)
    ax.set_xticks(range(3), ["before the loss", "after recovery", "whole mission"])
    ax.set_ylim(bottom=0)
    ax.set_ylabel("assignment changes per agent-minute")
    ax.set_title("Assignment churn (one dot per run)")
    ax.legend(fontsize=8)
    fig.suptitle(title_suffix(runs).strip(" ()"), x=0.99, ha="right", fontsize=8, color=INK2)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "recovery_churn.png"))
    plt.close(fig)


def representative(runs):
    """The run whose recovery (b) is the median of the recovered runs (else the first run)."""
    rec = [(r["m"]["loss"]["recovery_b_s"], i) for i, r in enumerate(runs)
           if r["m"].get("loss") and r["m"]["loss"].get("recovery_b_s") is not None]
    if not rec:
        return runs[0]
    rec.sort()
    return runs[rec[len(rec) // 2][1]]


def plot_gap_map(runs, out):
    r = representative(runs)
    maps = r["maps"]
    keys = [k for k in ("pre", "post") if k in maps] or ["overall"]
    fig, axs = plt.subplots(1, len(keys), figsize=(4.2 * len(keys) + 1.0, 4.4), squeeze=False)
    vmax = max(float(np.nanmax(maps[k])) for k in keys)
    ext = tuple(maps["extent"])
    im = None
    for ax, k in zip(axs[0], keys):
        im = ax.imshow(maps[k], origin="lower", extent=ext, cmap="Blues", vmin=0, vmax=vmax,
                       interpolation="nearest")
        ph = r["m"]["phases_s"][k]
        ax.set_title(f"{ {'pre': 'before the loss', 'post': 'after recovery', 'overall': 'whole mission'}[k]}"
                     f" [{ph[0]:.0f}-{ph[1]:.0f} s]")
        ax.set_xlabel("east (m)")
        ax.grid(False)
        ax.set_aspect("equal")
    axs[0][0].set_ylabel("north (m)")
    cb = fig.colorbar(im, ax=axs[0].tolist(), shrink=0.85)
    cb.set_label("longest unseen time in phase (s)")
    synth = " (SYNTHETIC)" if aggregate.is_synthetic(r["m"]) else ""
    fig.suptitle(f"Per-cell worst revisit gap, run {r['m']['run']}{synth}, "
                 f"agent {r['m'].get('victim')} lost at "
                 + (f"{r['m']['t_loss_s']:.1f} s" if r['m'].get('t_loss_s') is not None else "(no loss)"),
                 x=0.01, ha="left", fontweight="bold", fontsize=11)
    fig.savefig(os.path.join(out, "gap_map.png"), bbox_inches="tight")
    plt.close(fig)


def plot_separation(runs, out):
    fig, ax = plt.subplots(figsize=(7.5, 3.6))
    for r in runs:
        ax.plot(r["ts"], r["sep"], color=RUN, lw=0.9)
        L = r["m"].get("t_loss_s")
        if L is not None:
            ax.plot([L], [np.interp(L, r["ts"], np.nan_to_num(r["sep"]))], "x", color=CRIT, ms=7)
    ax.axhline(8.0, color=CRIT, lw=1.5)
    ax.text(1, 12, "8 m minimum", color=CRIT, fontsize=8)
    ax.plot([], [], color=RUN, label="one line per run")
    ax.plot([], [], "x", color=CRIT, label="agent killed")
    ax.legend(fontsize=8, loc="upper right")
    ax.set_ylim(bottom=0)
    ax.set_xlim(0, max(r["m"]["mission_s"] for r in runs))
    ax.set_xlabel("mission time (s)")
    ax.set_ylabel("min distance between live agents (m)")
    ax.set_title("Closest approach, true 3D positions" + title_suffix(runs))
    fig.tight_layout()
    fig.savefig(os.path.join(out, "min_separation.png"))
    plt.close(fig)


def make_all(run_dirs, out):
    runs = [load(d) for d in run_dirs]
    aggregate.check_out_dir(out, any(aggregate.is_synthetic(r["m"]) for r in runs))
    os.makedirs(out, exist_ok=True)
    plot_coverage(runs, out)
    plot_gaps(runs, out)
    plot_recovery_churn(runs, out)
    plot_gap_map(runs, out)
    plot_separation(runs, out)


def main():
    ap = argparse.ArgumentParser(description="summary plots")
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--out", default="results")
    a = ap.parse_args()
    make_all(a.runs, a.out)
    print(f"plots -> {a.out}/")


if __name__ == "__main__":
    main()
