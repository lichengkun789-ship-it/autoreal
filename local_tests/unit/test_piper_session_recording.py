from services.piper.episode_recording import (
    EpisodeRecordingSettings,
    SessionEpisodeRecorder,
)


class Reader:
    def snapshot(self):
        arm = {
            "joints_rad": [0.0] * 6,
            "gripper_width_m": 0.04,
            "motor": {
                "effort_nm": [0.0] * 6,
                "speed_rad_s": [0.0] * 6,
            },
        }
        return {"arms": {"left": arm, "right": arm}}


class Recorder:
    def __init__(self, *_args, **_kwargs):
        self.samples = []
        self.events = []
        self.errors = []
        self.started = False

    def start(self):
        self.started = True

    def mark(self, event, **_details):
        self.events.append(event)

    def record_robot_sample(self, arm, state):
        self.samples.append((arm, state))

    def record_error(self, message):
        self.errors.append(message)

    def stop(self, *, reason):
        return {"status": "passed", "scope": "task", "reason": reason}

    def reference(self):
        return {"status": "recording" if self.started else "idle", "scope": "task"}


def test_session_recorder_samples_both_arms_and_stops_once(tmp_path):
    value = SessionEpisodeRecorder(
        tmp_path,
        Reader(),
        object(),
        ("cam_high",),
        EpisodeRecordingSettings(fps=10.0),
        recorder_factory=Recorder,
    )
    value.start()
    result = value.stop()

    assert value.recorder.events == ["session_recording_started"]
    assert [arm for arm, _state in value.recorder.samples[:2]] == ["left", "right"]
    assert result["status"] == "passed"
    assert value.stop() is result
