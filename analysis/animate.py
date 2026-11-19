"""Top-down replay of one run as an MP4, at 4x speed (a 180 s mission takes 45 s).

python -m analysis.animate RUN_DIR --out FILE.mp4
Map colour = seconds since each 2 m cell was last seen; yellow = 60 s or more, or never.
"""
import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FFMpegWriter  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402

from analysis import metrics, report  # noqa: E402
from analysis.logs import load_run  # noqa: E402
from analysis.plots import (ACCENT, AGENT_COLORS, CRITICAL, INK, INK_2, STALE,  # noqa: E402
                            assignment_steps)
from swarm import config, sensor  # noqa: E402

SPEED = 4.0          # mission seconds per video second
FPS = 15
TRAIL_S = 8.0


def shares_at(steps, t):
    """The assignment in force at mission time t: {agent id: share}."""
    current = {}
    for start, shares in steps:
        if start <= t:
            current = shares
    return current


def last_row(log, t, live_until):
    """Index of the agent's last log row at or before t (and before it stopped counting)."""
    return int(np.searchsorted(log["t"], min(t, live_until), side="right")) - 1


def banner_text(run, t):
    lines = []
    if run.loss_s is not None and t >= run.loss_s:
        lines.append(f"agent {run.victim} lost at t = {run.loss_s:.1f} s")
        declared = run.lost_declared_s(run.victim)
        if declared is not None and t >= declared:
            lines.append(f"declared lost by the ground at t = {declared:.1f} s")
    for kind in ("blackout", "mute"):      # "mute" is only in some scenario runs' meta
        window = run.meta.get(kind)
        if window and window["t0"] <= t < window["t1"]:
            lines.append(f"agent {window['agent']}: radio {kind}")
    return "\n".join(lines)


def draw_static(fig, run):
    """Axes, area outline, priority zone; returns (map axes, strip axes, image)."""
    map_ax = fig.add_axes([0.08, 0.27, 0.74, 0.66])
    strip_ax = fig.add_axes([0.08, 0.06, 0.74, 0.13])
    colors = plt.get_cmap("Blues").with_extremes(over=STALE)
    image = map_ax.imshow(np.zeros(metrics.GRID_SHAPE), origin="lower", extent=config.AREA,
                          cmap=colors, vmin=0, vmax=metrics.WINDOW_S, interpolation="nearest")
    bar = fig.colorbar(image, cax=fig.add_axes([0.85, 0.27, 0.025, 0.66]), extend="max")
    bar.set_label("s since last seen (yellow: 60+ or never)")
    xmin, xmax, ymin, ymax = config.AREA
    map_ax.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, fill=False, ec=INK))
    if run.meta["priority_zone"]:
        zx0, zy0, zx1, zy1 = run.meta["priority_zone"]
        map_ax.add_patch(Rectangle((zx0, zy0), zx1 - zx0, zy1 - zy0, fill=False, ec=CRITICAL,
                                   lw=2, ls="-."))
    map_ax.set_xlim(xmin - 5, xmax + 5)
    map_ax.set_ylim(ymin - 5, ymax + 5)
    map_ax.set_aspect("equal")
    map_ax.grid(False)
    map_ax.set_xlabel("east (m)")
    map_ax.set_ylabel("north (m)")
    return map_ax, strip_ax, image


def draw_strip(strip_ax, run, ticks, coverage):
    """Coverage over the whole run in grey, with the 0.9 P line if there is a loss."""
    strip_ax.plot(ticks, coverage, color="#c3c2bd", lw=1)
    if run.loss_s is not None:
        before = coverage[metrics.in_phase(ticks, run.loss_s - metrics.WINDOW_S, run.loss_s)]
        strip_ax.axhline(0.9 * np.mean(before), color=INK_2, lw=1, ls=":")
        strip_ax.axvline(run.loss_s, color=CRITICAL, lw=1)
    strip_ax.set_xlim(0, run.end_s)
    strip_ax.set_ylim(0, 105)
    strip_ax.set_ylabel("coverage %")
    strip_ax.set_xlabel("mission time (s); dotted: 0.9 x coverage before the loss")
    return strip_ax.plot([], [], color=ACCENT, lw=2)[0]


def render(run_dir, out):
    run = load_run(run_dir)
    report.refuse_fake_in_results(out, {run.meta["backend"]})
    seen = metrics.sightings(run)
    ticks = np.arange(0.0, np.floor(run.end_s) + 1.0)
    coverage = metrics.percent_fresh(seen, ticks, metrics.WINDOW_S, metrics.ALL_CELLS)
    steps = assignment_steps(run)
    frame_times = np.arange(0.0, run.end_s, SPEED / FPS)

    fig = plt.figure(figsize=(8, 9.6), dpi=100)
    map_ax, strip_ax, image = draw_static(fig, run)
    strip_line = draw_strip(strip_ax, run, ticks, coverage)
    title = fig.text(0.08, 0.955, "", fontsize=14, fontweight="bold", color=INK)
    banner = fig.text(0.08, 0.205, "", fontsize=11, fontweight="bold", color=CRITICAL,
                      va="bottom")
    agents = list(run.agents)
    color = {a: AGENT_COLORS[i % len(AGENT_COLORS)] for i, a in enumerate(agents)}
    share_box = {a: map_ax.add_patch(Rectangle((0, 0), 0, 0, fill=False, ec=color[a], lw=1.5,
                                               ls="--")) for a in agents}
    footprint = {a: map_ax.add_patch(Polygon(np.zeros((4, 2)), fc=color[a], alpha=0.3, lw=2,
                                             ec=color[a])) for a in agents}
    trail = {a: map_ax.plot([], [], color=color[a], lw=2)[0] for a in agents}
    dot = {a: map_ax.plot([], [], "o", ms=9, mfc=color[a], mec=INK)[0] for a in agents}

    writer = FFMpegWriter(fps=FPS, codec="libx264", extra_args=["-pix_fmt", "yuv420p"])
    with writer.saving(fig, out, dpi=100):
        for t, last in metrics.last_seen_at(seen, frame_times):
            age = np.minimum(t - last, metrics.WINDOW_S + 1.0)      # never seen -> "60+"
            image.set_data(age.reshape(metrics.GRID_SHAPE))
            shares = shares_at(steps, t)
            for a in agents:
                box = share_box[a]
                box.set_visible(a in shares)
                if a in shares:
                    xmin, xmax, ymin, ymax = shares[a]
                    box.set_bounds(xmin, ymin, xmax - xmin, ymax - ymin)
                    box.set_hatch("//" if a == run.victim and t >= run.loss_s else None)
                log = run.agents[a]
                i = last_row(log, t, run.live_until(a))
                if i < 0:
                    continue
                x, y, z, yaw = log["x"][i], log["y"][i], log["z"][i], log["yaw"][i]
                dead = t > run.live_until(a)
                dot[a].set_data([x], [y])
                dot[a].set_marker("X" if dead else "o")
                dot[a].set_markerfacecolor(CRITICAL if dead else color[a])
                footprint[a].set_visible(not dead)
                footprint[a].set_xy(sensor.footprint_polygon(x, y, z, yaw))
                recent = (log["t"] >= t - TRAIL_S) & (log["t"] <= t) & ~dead
                trail[a].set_data(log["x"][recent], log["y"][recent])
            now = ticks <= t
            strip_line.set_data(ticks[now], coverage[now])
            title.set_text(f"{run.name}   t = {t:5.1f} s   coverage {coverage[now][-1]:5.1f} %")
            banner.set_text(banner_text(run, t))
            writer.grab_frame()
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="top-down MP4 replay of one run")
    parser.add_argument("run_dir")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    render(args.run_dir, args.out)
    print(args.out)


if __name__ == "__main__":
    main()
