"""run_mission.py checks that need no simulator."""
import pytest

import run_mission


def test_low_real_time_factor_invalidates_the_run():
    run_mission.check_rtf({"start": 0.99, "end": None})          # gz did not answer: no verdict
    with pytest.raises(run_mission.Invalid, match="0.90 < 0.95 at the end"):
        run_mission.check_rtf({"start": 0.99, "end": 0.90})


def test_payloadfail_replaces_the_kill_and_names_the_victim():
    import argparse
    import pytest
    from run_mission import plan
    args = argparse.Namespace(seed=1, n=3, mission="A", scenario="payloadfail", backend="gazebo",
                              zone=None)
    out = plan(args)
    assert [a for _, a, _ in out["faults"]] == ["land"]      # no kill on top
    assert out["kill_agent"] == plan(argparse.Namespace(**{**vars(args), "scenario": None}))["kill_agent"]
    with pytest.raises(SystemExit):                             # the fake vehicle has no autopilot
        plan(argparse.Namespace(**{**vars(args), "backend": "fake"}))


class _Ended:
    returncode = 2

    def poll(self):
        return 2


def test_live_view_a_drone_gone_by_itself_is_a_loss_not_an_invalid_run(monkeypatch, tmp_path):
    mission = run_mission.Mission.__new__(run_mission.Mission)
    mission.args = type("Args", (), {"mission_s": 180.0})()
    mission.out, mission.meta, mission.may_exit, mission.fleet = str(tmp_path), {}, set(), None
    mission.procs = {"agent_2": _Ended()}
    with pytest.raises(run_mission.Invalid, match="agent_2 died unexpectedly"):
        mission.check(10.0)                               # the evaluation: off by default
    monkeypatch.setattr(run_mission, "LIVE", True)
    mission.check(10.0)
    assert "agent_2" in mission.may_exit
    assert mission.meta["unplanned_losses"][0]["agent"] == 2


def test_victim_and_kill_time_can_be_chosen():
    import argparse
    from run_mission import plan
    args = argparse.Namespace(seed=1, n=3, mission="B", scenario=None, backend="gazebo", zone=None)
    drawn = plan(args)["faults"][0]
    chosen = plan(argparse.Namespace(**vars(args), victim=3, kill_at=80.0))
    assert chosen["faults"] == [(80.0, "kill", "sitl:3")] and chosen["kill_agent"] == 3
    assert plan(argparse.Namespace(**vars(args), victim=None, kill_at=None))["faults"][0] == drawn
