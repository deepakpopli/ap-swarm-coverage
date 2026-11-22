"""run_mission.py checks that need no simulator."""
import pytest

import run_mission


def test_low_real_time_factor_invalidates_the_run():
    run_mission.check_rtf({"start": 0.99, "end": None})          # gz did not answer: no verdict
    with pytest.raises(run_mission.Invalid, match="0.90 < 0.95 at the end"):
        run_mission.check_rtf({"start": 0.99, "end": 0.90})
