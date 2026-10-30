from copy import deepcopy

import pytest

from services.piper.effort_characterization import (
    PiperEffortNoiseRunner,
    effort_statistics,
)


def arm(effort):
    return {
        "joints_rad": [0.0, 0.2, -0.2, 0.0, 0.0, 0.0],
        "motor": {"effort_nm": list(effort)},
        "feedback": {
            "joint_age_s": 0.01,
            "status_age_s": 0.01,
            "motor_age_s": 0.01,
        },
        "controller": {"arm_status": 0, "errors": {}},
    }


class Reader:
    def __init__(self):
        self.count = 0

    def snapshot(self):
        self.count += 1
        effort = [0.1 * index + 0.001 * (self.count % 2) for index in range(6)]
        return {"arms": {"left": arm(effort), "right": arm(effort)}}


class DelayedReader(Reader):
    def snapshot(self):
        value = super().snapshot()
        if self.count == 1:
            value["arms"]["right"]["feedback"]["motor_age_s"] = None
        return value


def test_effort_statistics_reports_peak_to_peak_noise():
    value = effort_statistics([[0, 1, 2, 3, 4, 5], [0.1, 1, 2, 3, 4, 5]])
    assert value["joints"][0]["peak_to_peak_nm"] == pytest.approx(0.1)
    assert value["max_stationary_peak_to_peak_nm"] == pytest.approx(0.1)


def test_stationary_runner_is_read_only_and_does_not_admit_threshold():
    result = PiperEffortNoiseRunner(Reader(), sleeper=lambda _: None).run(
        arm="right", samples=20, interval_s=0.001
    )
    assert result["status"] == "passed"
    assert result["motion_commands_sent"] is False
    assert result["threshold_admitted"] is False
    assert result["statistics"]["max_stationary_peak_to_peak_nm"] == pytest.approx(0.001)


def test_stationary_runner_waits_for_initial_feedback():
    reader = DelayedReader()
    result = PiperEffortNoiseRunner(reader, sleeper=lambda _: None).run(
        arm="right", samples=20, interval_s=0.001
    )
    assert result["status"] == "passed"
    assert reader.count == 21
