"""Top-down MP4 of one run: freshness heatmap, shares, drones with footprints and trails,
the kill and the ground's "lost" declaration, running coverage % and clock.

Usage: nice -n 19 python -m analysis.animate runs/<name> [--out runs/<name>/replay.mp4]
                                              [--speed 4] [--fps 15] [--px 960]
Default: 180 s mission at 4x = 45 s of video, 15 fps, 960 x 960 px (cheap to render).
Heatmap = time since each cell was last seen (0-60 s, same grid and footprint model as
analysis.metrics); cells never seen or older than 60 s are white.
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.colors import ListedColormap
from matplotlib.patches import Polygon, Rectangle
import numpy as np

from swarm import config, sensor
from analysis import metrics
from analysis.logs import load_run

AGENT_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#6250d6", "#e87ba4"]   # palette slots 1,2,3,7,5
INK, INK2, CRIT = "#0b0b0b", "#52514e", "#e34948"
TRAIL_S = 8.0
BANNER_S = 10.0       # mission seconds a banner stays fully visible


def freshness_cmap():
    """Single-hue grey ramp: fresh (0 s) = mid grey, 60 s old = white."""
    g = plt.get_cmap("Greys")(np.linspace(0.55, 0.0, 256))
    return ListedColormap(g)


def assign_timeline(run):
    """[(t, {agent_id: share}), ...] from ground 'assign' events, time-sorted."""
    out = []
    for e in run.events("assign"):
        t = metrics.event_time(e, run)
        shares = {int(k): v.get("share") for k, v in (e.get("assignments") or {}).items()
                  if isinstance(v, dict) and v.get("share")}
        out.append((0.0 if t is None or t < 0 else t, shares))
    out.sort(key=lambda p: p[0])
    return out


def pose_at(log, t):
    """(x, y, z, yaw, alive) of an agent at mission time t; holds the last pose after its log ends."""
    tm = log["t_mission"]
    ok = np.isfinite(tm)
    tm = tm[ok]
    if len(tm) == 0 or t < tm[0]:
        return None
    alive = t <= tm[-1] + 0.5
    k = min(np.searchsorted(tm, t), len(tm) - 1)
    return (log["x"][ok][k], log["y"][ok][k], log["z"][ok][k], log["yaw"][ok][k], alive)


def trail(log, t):
    tm = log["t_mission"]
    sel = np.isfinite(tm) & (tm <= t) & (tm >= t - TRAIL_S)
    return log["x"][sel], log["y"][sel]


def render(run_dir, out=None, speed=4.0, fps=15, px=960, cell=metrics.CELL_M):
    run = load_run(run_dir)
    mission_s = float(run.meta_get("mission_s", config.MISSION_S))
    victim, t_kill = run.meta.get("kill_agent"), metrics.loss_time(run)
    t_lost = metrics.lost_declared_time(run, victim, t_kill) if victim is not None else None
    synthetic = run.meta.get("backend") == "synthetic"
    grid = metrics.Grid(cell)
    ls = metrics.LastSeen(grid, metrics.coverage_samples(run, mission_s))
    shares_tl = assign_timeline(run)
    ids = sorted(run.agents)
    color = {aid: AGENT_COLORS[i % len(AGENT_COLORS)] for i, aid in enumerate(ids)}
    out = out or os.path.join(run_dir, "replay.mp4")

    dpi = 100
    fig = plt.figure(figsize=(px / dpi, px / dpi), dpi=dpi)
    ax = fig.add_axes([0.07, 0.06, 0.78, 0.80])
    cax = fig.add_axes([0.88, 0.06, 0.025, 0.80])
    xmin, xmax, ymin, ymax = config.AREA
    img = ax.imshow(np.full(grid.shape, np.nan), origin="lower", extent=grid.extent,
                    cmap=freshness_cmap(), vmin=0, vmax=metrics.WINDOW_S, interpolation="nearest")
    img.cmap.set_bad("white")
    img.cmap.set_over("white")
    cb = fig.colorbar(img, cax=cax)
    cb.set_label("time since last seen (s)")
    ax.set_xlim(xmin - 5, xmax + 5)
    ax.set_ylim(ymin - 5, ymax + 5)
    ax.set_aspect("equal")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, fill=False, ec=INK, lw=1.2))
    title = fig.text(0.07, 0.95, "", fontsize=15, fontweight="bold", color=INK)
    sub = fig.text(0.07, 0.915, "", fontsize=11, color=INK2)
    banner = fig.text(0.07, 0.88, "", fontsize=12, fontweight="bold", color=CRIT)
    if synthetic:
        fig.text(0.99, 0.01, "SYNTHETIC test data", ha="right", fontsize=9, color=INK2)

    share_patches = {aid: ax.add_patch(Rectangle((0, 0), 0, 0, fill=False, ec=color[aid], lw=1.5,
                                                 ls="--", visible=False)) for aid in ids}
    fp = {aid: ax.add_patch(Polygon(np.zeros((4, 2)), closed=True, fc=color[aid], alpha=0.15,
                                    ec=color[aid], lw=1.5)) for aid in ids}
    tr = {aid: ax.plot([], [], color=color[aid], lw=2)[0] for aid in ids}
    dot = {aid: ax.plot([], [], "o", ms=10, mfc=color[aid], mec=INK, mew=1.0)[0] for aid in ids}
    lbl = {aid: ax.text(0, 0, f" {aid}", fontsize=11, fontweight="bold", color=INK) for aid in ids}

    frame_dt = speed / fps
    times = np.arange(0.0, mission_s + 1e-9, frame_dt)
    writer = FFMpegWriter(fps=fps, codec="libx264", bitrate=1800,
                          extra_args=["-pix_fmt", "yuv420p", "-preset", "veryfast"])
    with writer.saving(fig, out, dpi):
        for t in times:
            last = ls.advance(t)
            age = t - last
            img.set_data(np.where(np.isfinite(age), age, np.nan))
            cov = 100.0 * ls.fresh_fraction(t)

            shares = {}
            for ts, sh in shares_tl:
                if ts <= t:
                    shares = sh
            for aid in ids:
                s = shares.get(aid)
                p = share_patches[aid]
                p.set_visible(s is not None)
                if s:
                    p.set_bounds(s[0], s[2], s[1] - s[0], s[3] - s[2])

                pose = pose_at(run.agents[aid], t)
                if pose is None:
                    continue
                x, y, z, yaw, alive = pose
                dot[aid].set_data([x], [y])
                lbl[aid].set_position((x + 3, y + 3))
                if alive:
                    fp[aid].set_xy(np.array(sensor.footprint_polygon(x, y, z, yaw)))
                    tx, ty = trail(run.agents[aid], t)
                    tr[aid].set_data(tx, ty)
                else:
                    fp[aid].set_visible(False)
                    tr[aid].set_data([], [])
                    dot[aid].set_marker("X")
                    dot[aid].set_markersize(16)
                    dot[aid].set_markerfacecolor(CRIT)

            title.set_text(f"t = {t:5.1f} s     coverage (last 60 s) {cov:5.1f} %")
            alive_ids = [a for a in ids if not (a == victim and t_kill is not None and t >= t_kill)]
            sub.set_text(f"run {run.name}   agents alive: {', '.join(map(str, alive_ids))}   "
                         f"replay {speed:g}x")
            msg = ""
            if t_kill is not None and t >= t_kill:
                msg = f"Agent {victim} KILLED at t = {t_kill:.1f} s"
                if t_lost is not None and t >= t_lost:
                    msg += f"  |  ground declared it LOST at t = {t_lost:.1f} s (+{t_lost - t_kill:.1f} s)"
            banner.set_text(msg)
            banner.set_alpha(1.0 if (t_kill is not None and t - max(t_kill, t_lost or t_kill) < BANNER_S) else 0.7)
            writer.grab_frame()
    plt.close(fig)
    return out, len(times)


def main():
    ap = argparse.ArgumentParser(description="top-down MP4 replay of one run")
    ap.add_argument("run_dir")
    ap.add_argument("--out")
    ap.add_argument("--speed", type=float, default=4.0, help="mission seconds per video second (>= 3)")
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--px", type=int, default=960)
    a = ap.parse_args()
    out, n = render(a.run_dir, a.out, a.speed, a.fps, a.px)
    print(f"{n} frames -> {out}")


if __name__ == "__main__":
    main()
