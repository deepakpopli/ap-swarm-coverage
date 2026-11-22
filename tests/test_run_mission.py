"""run_mission.py checks that need no simulator."""
import pytest

import run_mission


def test_low_real_time_factor_invalidates_the_run():
    run_mission.check_rtf({"start": 0.99, "end": None})          # gz did not answer: no verdict
    with pytest.raises(run_mission.Invalid, match="0.90 < 0.95 at the end"):
        run_mission.check_rtf({"start": 0.99, "end": 0.90})


def test_gpsfail_replaces_the_kill_and_names_the_victim():
    import argparse
    import pytest
    from run_mission import plan
    args = argparse.Namespace(seed=1, n=3, mission="A", scenario="gpsfail", backend="gazebo",
                              zone=None)
    out = plan(args)
    assert [a for _, a, _ in out["faults"]] == ["gps_off"]      # no kill on top
    assert out["kill_agent"] == plan(argparse.Namespace(**{**vars(args), "scenario": None}))["kill_agent"]
    with pytest.raises(SystemExit):                             # the fake vehicle has no GPS
        plan(argparse.Namespace(**{**vars(args), "backend": "fake"}))
