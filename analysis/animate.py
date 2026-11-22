"""Top-down replay of one run with the dropout: 1280 x 720 H.264, under 60 s, no narration needed.

python -m analysis.animate RUN_DIR --out FILE.mp4

Left: the 300 x 300 m area. Colour = seconds since each 2 m cell was last seen (white = just
seen, dark = 60 s, yellow = 60 s or more, or never), the 4 lanes with their owners, each drone's
camera footprint, drones coloured by the ground's liveness view. Right: what is happening and
the coverage % so far. The quiet parts play faster than the loss; the speed is always on screen.
Coverage comes from metrics.sightings (swarm/sensor.py footprints), as in the analysis.
"""
import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FFMpegWriter  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402

from analysis import metrics, report  # noqa: E402
from analysis.logs import load_run  # noqa: E402
from analysis.plots import ACCENT, CRITICAL, INK, INK_2, STALE  # noqa: E402
from swarm import allocator, config, sensor  # noqa: E402

FPS = 20
WIDTH_PX, HEIGHT_PX, DPI = 1280, 720, 100
TRAIL_S = 8.0
HOLD_VIDEO_S = 2.0          # freeze on each key moment, long enough to read its caption
QUIET_SPEED = 8.0           # mission s per video s while nothing changes
SETTLE_SPEED = 4.0          # survivors re-flying the orphaned lanes, until recovery
EVENT_SPEED = 2.0           # from just before the kill to just after the declaration
EVENT_MARGIN_S = 4.0        # mission s of slower play on each side of the key moments
LANE_MATCH_M = 1.0          # a slot this close to a lane's x flies that lane

LIVENESS_COLORS = {"alive": "#1baf7a", "suspect": "#f08c00", "lost": CRITICAL}
KILLED_COLOR = INK
PAST_CAPTION = "#8a8984"
SIDE_X = 0.61               # left edge of the right-hand panel, figure fraction
CAPTIONS_TOP = 0.68
CAPTION_LINE_H = 0.026      # one 12 pt line at 720 px, figure fraction
CAPTION_GAP = 0.014


# ----------------------------------------------------------------------- run facts

def liveness_at(run, agent_id, t_s):
    """The ground's view of the agent at t_s: alive | suspect | lost (alive before any event)."""
    state = "alive"
    for e in run.events("liveness"):
        if e["agent"] == agent_id and e["t_mission"] is not None and e["t_mission"] <= t_s:
            state = e["to"]
    return state


def killed_at(run, agent_id, t_s):
    """Ground truth from meta.json: the agent's vehicle process was killed at or before t_s."""
    return agent_id == run.victim and run.loss_s is not None and t_s >= run.loss_s


def first_liveness_s(run, agent_id, state):
    for e in run.events("liveness"):
        if e["agent"] == agent_id and e["to"] == state and e["t_mission"] is not None:
            return e["t_mission"]
    return None


def lane_owners(event, lane_xs_m):
    """{lane index: agent id} of one assign event: the agent whose loop flies that lane's x."""
    owners = {}
    for agent, a in event["assignments"].items():
        for x_m, _ in a["slots"][a["loop_from"]:]:
            for i, lane_x_m in enumerate(lane_xs_m):
                if abs(x_m - lane_x_m) <= LANE_MATCH_M:
                    owners[i] = int(agent)
    return owners


def owner_steps(run, lane_xs_m):
    """[(from t, {lane index: agent id})], one per assign event; before the start = from 0."""
    steps = []
    for e in run.events("assign"):
        t_s = 0.0 if e["t_mission"] is None else max(0.0, e["t_mission"])
        steps.append((t_s, lane_owners(e, lane_xs_m)))
    return steps


def step_at(steps, t_s):
    current = {}
    for start_s, value in steps:
        if start_s <= t_s:
            current = value
    return current


def key_moments(run, loss):
    """{name: mission s} of the story: kill, suspect, declared, recovered (None if absent)."""
    if run.loss_s is None:
        return {"kill": None, "suspect": None, "declared": None, "recovered": None}
    recovered_s = None
    if loss["recovery_s"] is not None:
        recovered_s = run.loss_s + loss["recovery_s"]
    return {"kill": run.loss_s,
            "suspect": first_liveness_s(run, run.victim, "suspect"),
            "declared": loss["lost_declared_s"],
            "recovered": recovered_s}


def captions(run, loss, moments):
    """[(mission s, text)] in time order; each one stays on screen from its time on."""
    k = run.victim
    lost_after_s = run.meta.get("lost_after_s", config.LOST_S)
    out = [(0.0, f"{len(run.agents)} drones fly 4 fixed lanes over 300 x 300 m.\n"
                 "Map colour: seconds since each cell was last seen.")]
    if moments["kill"] is not None:
        out.append((moments["kill"], f"t = {moments['kill']:.1f} s: drone {k} killed\n"
                                     "(process killed, no landing)"))
    if moments["suspect"] is not None:
        out.append((moments["suspect"], f"t = {moments['suspect']:.1f} s: silent "
                                        f"{config.SUSPECT_S:.0f} s: suspect"))
    if moments["declared"] is not None:
        out.append((moments["declared"],
                    f"t = {moments['declared']:.1f} s: silent {lost_after_s:.0f} s: "
                    "declared lost,\n"
                    "re-plan: survivors take its lanes"))
    if moments["recovered"] is not None:
        out.append((moments["recovered"],
                    f"t = {moments['recovered']:.1f} s: recovered to within 10 % of pre-loss\n"
                    f"(coverage back above 0.9 x {loss['before_pct']:.1f} % and stays)"))
    return out


# ----------------------------------------------------------------------- playback clock

def speed_segments(end_s, moments):
    """[(from s, to s, speed)] covering [0, end_s]: slow around the loss, fast elsewhere."""
    kill_s = moments["kill"]
    if kill_s is None:
        return [(0.0, end_s, QUIET_SPEED)]
    slow_to_s = (moments["declared"] or kill_s) + EVENT_MARGIN_S
    settle_to_s = max(slow_to_s, (moments["recovered"] or slow_to_s) + EVENT_MARGIN_S)
    edges = [0.0, kill_s - EVENT_MARGIN_S, slow_to_s, settle_to_s, end_s]
    edges = [min(max(e, 0.0), end_s) for e in edges]
    speeds = [QUIET_SPEED, EVENT_SPEED, SETTLE_SPEED, QUIET_SPEED]
    return [(a, b, s) for a, b, s in zip(edges, edges[1:], speeds) if b > a]


def frame_plan(end_s, moments, fps=FPS):
    """[(mission s, frames to show it, speed)]: each segment at its speed, a hold per moment."""
    holds = sorted(t for t in moments.values() if t is not None)
    plan = [(0.0, int(HOLD_VIDEO_S * fps), QUIET_SPEED)]       # time to read the intro
    for start_s, stop_s, speed in speed_segments(end_s, moments):
        for t_s in np.arange(start_s, stop_s, speed / fps):
            plan.append((float(t_s), 1, speed))
            while holds and holds[0] <= t_s:
                holds.pop(0)
                plan.append((float(t_s), int(HOLD_VIDEO_S * fps), speed))
    return plan


def video_seconds(plan, fps=FPS):
    return sum(n for _, n, _ in plan) / fps


# ----------------------------------------------------------------------- drawing

def draw_map(fig):
    """The area, the age image and its colour bar; returns (axes, image)."""
    ax = fig.add_axes([0.035, 0.07, 0.49, 0.87])
    colors = plt.get_cmap("Blues").with_extremes(over=STALE)
    image = ax.imshow(np.zeros(metrics.GRID_SHAPE), origin="lower", extent=config.AREA,
                      cmap=colors, vmin=0, vmax=metrics.WINDOW_S, interpolation="nearest")
    bar = fig.colorbar(image, cax=fig.add_axes([0.535, 0.07, 0.012, 0.87]), extend="max")
    bar.set_label("seconds since last seen", fontsize=11, labelpad=2)
    bar.ax.set_title("60+\nor never", fontsize=9, color=INK_2, pad=4)
    xmin, xmax, ymin, ymax = config.AREA
    ax.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, fill=False, ec=INK, lw=1.5))
    ax.set_xlim(xmin - 3, xmax + 3)
    ax.set_ylim(ymin - 3, ymax + 22)          # room above the area for the lane labels
    ax.set_aspect("equal")
    ax.grid(False)
    ax.set_xticks(np.arange(xmin, xmax + 1, 50))
    ax.set_yticks(np.arange(ymin, ymax + 1, 50))
    ax.tick_params(labelsize=10)
    ax.set_xlabel("east (m)", fontsize=11, labelpad=2)
    ax.set_ylabel("north (m)", fontsize=11, labelpad=2)
    return ax, image


def draw_lanes(ax, lane_xs_m):
    """Dashed lane centre lines; returns one label per lane (its owner is filled in per frame)."""
    labels = []
    for x_m in lane_xs_m:
        ax.plot([x_m, x_m], [config.AREA[2], config.AREA[3]], color=INK_2, lw=0.8,
                ls=(0, (4, 4)), alpha=0.7)
        labels.append(ax.text(x_m, config.AREA[3] + 10, "", ha="center", va="center",
                              fontsize=11, fontweight="bold", color=INK))
    return labels


def draw_drones(ax, agents):
    """Per agent: footprint, trail, marker, number and state label artists."""
    artists = {}
    for a in agents:
        artists[a] = {
            "footprint": ax.add_patch(Polygon(np.zeros((4, 2)), fc="white", alpha=0.35, lw=2)),
            "trail": ax.plot([], [], lw=2.5)[0],
            "dot": ax.plot([], [], "o", ms=17, mec=INK, mew=1.5, zorder=5)[0],
            "cross": ax.plot([], [], "x", ms=22, color=KILLED_COLOR, mew=3.5, zorder=6)[0],
            "number": ax.text(0, 0, str(a), ha="center", va="center", fontsize=11,
                              fontweight="bold", color="white", zorder=7),
            "label": ax.text(0, 0, "", fontsize=11, fontweight="bold", zorder=7,
                             bbox={"fc": "white", "ec": "none", "alpha": 0.85, "pad": 1.5}),
        }
    return artists


def caption_tops(story):
    """Figure y of each caption's top: stacked by line count, so 1- and 2-line ones pack evenly."""
    tops, y = [], CAPTIONS_TOP
    for _, text in story:
        tops.append(y)
        y -= CAPTION_LINE_H * (text.count("\n") + 1) + CAPTION_GAP
    return tops


def draw_side(fig, run, story):
    """Title, clock, legend and the caption lines; returns (clock, speed, caption texts)."""
    fig.text(SIDE_X, 0.945, "3 drones, one killed mid-mission", fontsize=17,
             fontweight="bold", color=INK)
    fig.text(SIDE_X, 0.885, f"ArduPilot SITL + Gazebo, run {run.name}: the run closest to the\n"
                            f"12-run medians, {run.meta['radio']['loss'] * 100:.0f} % radio loss",
             fontsize=11, color=INK_2, linespacing=1.2)
    clock = fig.text(SIDE_X, 0.815, "", fontsize=15, color=INK, family="DejaVu Sans Mono")
    speed = fig.text(0.04, 0.965, "", fontsize=12, color=INK_2)     # above the map
    legend_ax = fig.add_axes([SIDE_X, 0.715, 0.38, 0.07])
    legend_ax.axis("off")
    handles = [Line2D([], [], marker="o", ls="", ms=12, mec=INK, mfc=c, label=s)
               for s, c in LIVENESS_COLORS.items()]
    handles.append(Line2D([], [], marker="x", ls="", ms=12, mew=3, color=KILLED_COLOR,
                          label="killed (truth)"))
    legend_ax.legend(handles=handles, loc="center left", ncol=4, fontsize=11, frameon=False,
                     handletextpad=0.3, columnspacing=1.0, alignment="left", borderaxespad=0,
                     title="drone colour = the ground's view of it", title_fontsize=11)
    lines = [fig.text(SIDE_X, top, "", fontsize=12, va="top", color=PAST_CAPTION,
                      linespacing=1.15) for top in caption_tops(story)]
    return clock, speed, lines


def draw_coverage(fig, run, loss, moments):
    """Coverage % axes with the 0.9 x pre-loss line; returns (line, {moment: its mark artists})."""
    ax = fig.add_axes([SIDE_X + 0.045, 0.085, 0.32, 0.235])
    if loss is not None:
        ax.axhline(loss["recovery_threshold_pct"], color=INK_2, lw=1, ls=":")
        ax.text(2, loss["recovery_threshold_pct"] - 1.5, "0.9 x pre-loss", fontsize=10,
                color=INK_2, va="top")
    styles = {"kill": (CRITICAL, "--", "kill"), "declared": (CRITICAL, "-", "lost"),
              "recovered": (LIVENESS_COLORS["alive"], "-", "recovered")}
    marks = {}
    for key, (color, style, text) in styles.items():
        if moments[key] is not None:
            marks[key] = (ax.axvline(moments[key], color=color, lw=1.5, ls=style),
                          ax.text(moments[key] + 1.5, 101, text, fontsize=10, color=color,
                                  va="bottom"))
    ax.set_xlim(0, run.end_s)
    ax.set_ylim(60, 100)
    ax.tick_params(labelsize=10)
    ax.set_ylabel("coverage %", fontsize=11)
    ax.set_xlabel("mission time (s); coverage = cells seen in the last 60 s", fontsize=11)
    return ax.plot([], [], color=ACCENT, lw=2.5)[0], marks


# ----------------------------------------------------------------------- per frame

def update_drone(art, run, a, t_s):
    """Move one drone's artists to mission time t_s, coloured by the ground's view."""
    log = run.agents[a]
    shown_until_s = min(t_s, run.live_until(a))        # the victim stays where it was killed
    i = int(np.searchsorted(log["t"], shown_until_s, side="right")) - 1
    for artist in art.values():
        artist.set_visible(i >= 0)
    if i < 0:
        return
    x, y, z, yaw = log["x"][i], log["y"][i], log["z"][i], log["yaw"][i]
    state = liveness_at(run, a, t_s)
    killed = killed_at(run, a, t_s)
    color = LIVENESS_COLORS.get(state, INK_2)
    art["dot"].set_data([x], [y])
    art["dot"].set_markerfacecolor(color)
    art["cross"].set_data([x], [y])
    art["cross"].set_visible(killed)
    art["number"].set_position((x, y))
    art["number"].set_visible(not killed)          # the cross replaces the number
    art["label"].set_position((x + 9, y + 6))
    art["label"].set_text(f"D{a} killed; ground: {state}" if killed else f"D{a} {state}")
    art["label"].set_color(color)
    art["footprint"].set_visible(not killed)       # a dead camera sees nothing
    art["footprint"].set_xy(sensor.footprint_polygon(x, y, z, yaw))
    art["footprint"].set_edgecolor(color)
    recent = (log["t"] >= t_s - TRAIL_S) & (log["t"] <= shown_until_s)
    art["trail"].set_data(log["x"][recent], log["y"][recent])
    art["trail"].set_color(color)


def update_lanes(labels, steps, run, t_s):
    """Lane labels name their owner, coloured by the ground's view of it; a re-plan shows "was"."""
    first = steps[0][1] if steps else {}
    owners = step_at(steps, t_s)
    for i, label in enumerate(labels):
        a, was = owners.get(i), first.get(i)
        moved = a is not None and was is not None and a != was
        label.set_text(f"lane {i + 1}: D{was} \u2192 D{a}" if moved else f"lane {i + 1}: D{a}")
        label.set_color(LIVENESS_COLORS.get(liveness_at(run, a, t_s), INK) if a else INK)
        label.set_bbox({"fc": "white", "ec": INK, "pad": 2} if moved else None)


def update_captions(lines, story, t_s):
    """Captions so far in grey, the latest in bold black."""
    shown = [text for start_s, text in story if start_s <= t_s]
    for i, line in enumerate(lines):
        line.set_text(shown[i] if i < len(shown) else "")
        latest = i == len(shown) - 1
        line.set_color(INK if latest else PAST_CAPTION)
        line.set_fontweight("bold" if latest else "normal")


# ----------------------------------------------------------------------- render

def render(run_dir, out):
    """Write the MP4; returns its length in video seconds."""
    run = load_run(run_dir)
    report.refuse_fake_in_results(out, {run.meta["backend"]})
    seen = metrics.sightings(run)
    ticks = np.arange(0.0, np.floor(run.end_s) + 1.0)
    coverage = metrics.percent_fresh(seen, ticks, metrics.WINDOW_S, metrics.ALL_CELLS)
    loss = metrics.loss_block(run, ticks, coverage) if run.loss_s is not None else None
    moments = key_moments(run, loss)
    story = captions(run, loss, moments)
    plan = frame_plan(run.end_s, moments)
    lane_xs_m = allocator.area_lanes()
    steps = owner_steps(run, lane_xs_m)

    fig = plt.figure(figsize=(WIDTH_PX / DPI, HEIGHT_PX / DPI), dpi=DPI)
    map_ax, image = draw_map(fig)
    lane_labels = draw_lanes(map_ax, lane_xs_m)
    drones = draw_drones(map_ax, list(run.agents))
    clock, speed_text, lines = draw_side(fig, run, story)
    cover_line, marks = draw_coverage(fig, run, loss, moments)

    writer = FFMpegWriter(fps=FPS, codec="libx264",
                          extra_args=["-pix_fmt", "yuv420p", "-crf", "20"])
    times = [t for t, _, _ in plan]
    with writer.saving(fig, out, dpi=DPI):
        for (t_s, last), (_, repeats, speed) in zip(metrics.last_seen_at(seen, times), plan):
            age_s = np.minimum(t_s - last, metrics.WINDOW_S + 1.0)    # never seen -> "60+"
            image.set_data(age_s.reshape(metrics.GRID_SHAPE))
            for a, art in drones.items():
                update_drone(art, run, a, t_s)
            update_lanes(lane_labels, steps, run, t_s)
            update_captions(lines, story, t_s)
            now = ticks <= t_s
            cover_line.set_data(ticks[now], coverage[now])
            for key, artists in marks.items():
                for artist in artists:
                    artist.set_visible(t_s >= moments[key])    # no spoilers
            clock.set_text(f"t = {t_s:5.1f} s   coverage {coverage[now][-1]:5.1f} %")
            paused = " (paused)" if repeats > 1 else ""
            speed_text.set_text(f"playback x{speed:.0f}: {speed:.0f} mission s per video s{paused}")
            for _ in range(repeats):
                writer.grab_frame()
    plt.close(fig)
    return video_seconds(plan)


def main():
    parser = argparse.ArgumentParser(description="top-down MP4 replay of one run")
    parser.add_argument("run_dir")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    seconds = render(args.run_dir, args.out)
    print(f"{args.out}: {seconds:.1f} s")


if __name__ == "__main__":
    main()
