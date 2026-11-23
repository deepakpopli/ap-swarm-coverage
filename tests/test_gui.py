"""gui: the live-run choice, the run model, and the map maths of static/geom.js."""
import json
import math
import os
import shutil
import subprocess

import pytest

from swarm import sensor
from gui import server
from gui.model import RunModel

GEOM_JS = os.path.join(os.path.dirname(server.__file__), "static", "geom.js")
NODE = shutil.which("node") or shutil.which("nodejs")


# ---- which run a cold dashboard shows
def test_pick_live_takes_the_newest_growing_running_run():
    now = 1000.0
    runs = [("new_crashed", "running", now - 60), ("live", "running", now - 2),
            ("old_ok", "ok", now - 1)]
    assert server.pick_live(runs, now) == "live"


def test_pick_live_none_when_every_run_ended_or_stalled():
    now = 1000.0
    assert server.pick_live([("a", "ok", now), ("b", "running", now - 100)], now) is None
    assert server.pick_live([], now) is None


def test_live_dir_follows_a_run_being_written(tmp_path):
    for name, outcome in (("old", "ok"), ("new", "running")):
        (tmp_path / name).mkdir()
        (tmp_path / name / "meta.json").write_text(json.dumps({"outcome": outcome}))
    os.utime(tmp_path / "old" / "meta.json", (1, 1))            # old: long ago
    dash = server.Dashboard([str(tmp_path)])
    assert dash.run_dirs()[0] == str(tmp_path / "new")
    assert dash.live_dir() == str(tmp_path / "new")
    (tmp_path / "new" / "meta.json").write_text(json.dumps({"outcome": "ok"}))
    assert dash.live_dir() is None


def make_run(run_dir):
    run_dir.mkdir()
    (run_dir / "meta.json").write_text(json.dumps({"t0_wall": 1000.0, "outcome": "ok",
                                                   "priority_zone": [120, 120, 180, 180]}))
    (run_dir / "agent_1.csv").write_text("t_wall,x,y,z,yaw,status\n990.0,1,2,50,3.1416,cover\n")
    (run_dir / "ground.jsonl").write_text(json.dumps({"t_wall": 995.0, "event": "start",
                                                      "t0": 1000.0}) + "\n")
    (run_dir / "agent_1.events.jsonl").write_text(json.dumps({"t_wall": 991.0, "event": "cruise"})
                                                  + "\n")
    return run_dir


def test_model_reads_ground_events_once(tmp_path):
    model = RunModel(str(make_run(tmp_path / "r")))
    model.refresh()
    payload, cursor = model.delta(None)
    assert [e["event"] for e in payload["events"]] == ["start"]          # agent events not shown
    assert payload["zone"] == [120, 120, 180, 180] and payload["agents"][1][0][1:5] == [1, 2, 50, 3.142]
    model.refresh()
    assert model.delta(cursor)[0]["events"] == []                          # read once


# ---- static/geom.js under node
def geom(expr):
    out = subprocess.run([NODE, "-e", f"const g = require({json.dumps(GEOM_JS)});"
                                      f"console.log(JSON.stringify({expr}))"],
                         capture_output=True, text=True, check=True).stdout
    return json.loads(out)


needs_node = pytest.mark.skipif(NODE is None, reason="node not installed")


@needs_node
@pytest.mark.parametrize("yaw, tip", [(0.0, (0, -10)), (math.pi / 2, (10, 0)),
                                      (math.pi, (0, 10)), (-math.pi / 2, (-10, 0))])
def test_arrow_points_along_the_yaw(yaw, tip):
    # canvas y is down: north (yaw 0) is up, east (pi/2) right, south (pi) down
    points = geom(f"g.arrowPoints(100, 200, {yaw}, 10)")
    assert points[0] == pytest.approx([100 + tip[0], 200 + tip[1]], abs=1e-9)
    tail = points[2]                                    # the notch is behind the centre
    assert tail == pytest.approx([100 - 0.35 * tip[0], 200 - 0.35 * tip[1]], abs=1e-9)


@needs_node
def test_arrow_wings_left_and_right_of_the_heading():
    # facing east, the right wing is south of the nose line (canvas: larger y)
    _, right, _, left = geom(f"g.arrowPoints(0, 0, {math.pi / 2}, 10)")
    assert right[1] > 0 > left[1]


@needs_node
@pytest.mark.parametrize("yaw", [0.0, 0.7, math.pi, 4.0])
def test_footprint_corners_match_sensor(yaw):
    corners = geom(f"g.footprintCorners(150, 80, 50, {yaw}, {sensor.TAN_HALF_ACROSS},"
                   f" {sensor.TAN_HALF_ALONG})")
    expected = sensor.footprint_polygon(150, 80, 50, yaw)
    assert sorted(map(tuple, corners)) == pytest.approx(sorted(map(tuple, expected)))


@needs_node
def test_heat_colours():
    assert geom("g.heatRgb(-1, 60)") == [255, 255, 255]
    assert geom("g.heatRgb(60, 60)") == geom("g.heatRgb(600, 60)") == [228, 228, 228]


@needs_node
def test_recovery_levels_and_time():
    series = [[t, 90.0] for t in range(0, 60)] + [[60, 50.0], [61, 70.0]] + \
        [[t, 85.0] for t in range(62, 80)]
    rec = geom(f"g.recovery({json.dumps(series)}, 60, 3, 60)")
    assert rec["before"] == pytest.approx(90.0) and rec["share"] == pytest.approx(60.0)
    assert rec["back"] == 62                       # 85 >= 0.9 * 90, held to the end (>= 10 s)
    assert geom(f"g.recovery({json.dumps(series[:65])}, 60, 3, 60)")["back"] is None


def test_zone_age_and_metrics_loss(tmp_path):
    run_dir = make_run(tmp_path / "r")                # agent 1 at (1, 2): not in the zone
    model = RunModel(str(run_dir))
    model.refresh()
    assert model.heat(0.0)["zone_age_s"] is None and model.delta(None)[0]["loss"] is None
    (run_dir / "metrics.json").write_text(json.dumps({"loss": {"loss_s": 64, "before_pct": 97,
                                                               "share_pct": 64.7, "recovery_s": 30}}))
    assert model.metrics_loss() == {"loss_s": 64, "before_pct": 97, "share_pct": 64.7,
                                    "recovery_s": 30}


def test_a_run_root_may_itself_be_a_run(tmp_path):
    run_dir = make_run(tmp_path / "r")
    dash = server.Dashboard([str(run_dir)])
    assert dash.checked_dir(str(run_dir)) == str(run_dir)
    with pytest.raises(ValueError):
        dash.checked_dir(str(tmp_path))


def test_zone_age_series_is_the_stalest_zone_cell(tmp_path):
    """Two passes over the zone: once all of it is seen, its age counts from the older pass."""
    run_dir = make_run(tmp_path / "r")
    (run_dir / "agent_1.csv").write_text("t_wall,x,y,z,yaw,status\n990.0,150,130,50,0,cover\n"
                                         "991.0,150,170,50,0,cover\n994.0,0,0,50,0,cover\n")
    model = RunModel(str(run_dir))
    model.refresh()
    ages = {t: age for t, _, _, age in model.series_points}
    assert ages[0.0] is None                       # only the south half seen yet
    assert ages[1.0] == 1.0 and ages[3.0] == 3.0   # all seen; the south half was seen at 990
