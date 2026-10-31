"""RealSense adapter tests use an SDK-shaped fake and never open USB devices."""

from types import SimpleNamespace

import numpy as np
import pytest

from services.piper.camera import CameraSpec, RealSenseRig, list_realsense_devices


class FakeConfig:
    def __init__(self):
        self.device = None
        self.streams = []

    def enable_device(self, serial):
        self.device = serial

    def enable_stream(self, *arguments):
        self.streams.append(arguments)


class FakeFrame:
    def __init__(self, data, *, timestamp=123.0):
        self.data = data
        self.timestamp = timestamp
        intrinsics = SimpleNamespace(
            width=data.shape[1],
            height=data.shape[0],
            fx=100.0,
            fy=101.0,
            ppx=4.0,
            ppy=3.0,
            model="brown_conrady",
            coeffs=[0.0] * 5,
        )
        self.profile = SimpleNamespace(
            as_video_stream_profile=lambda: SimpleNamespace(intrinsics=intrinsics)
        )

    def get_data(self):
        return self.data

    def get_timestamp(self):
        return self.timestamp


class FakeFrameSet:
    def __init__(self):
        self.color = FakeFrame(np.full((6, 8, 3), [1, 2, 3], dtype=np.uint8))
        self.depth = FakeFrame(np.full((6, 8), 500, dtype=np.uint16))

    def get_color_frame(self):
        return self.color

    def get_depth_frame(self):
        return self.depth


class FakePipeline:
    def __init__(self):
        self.config = None
        self.waits = 0
        self.stopped = False

    def start(self, config):
        self.config = config
        device = SimpleNamespace(
            first_depth_sensor=lambda: SimpleNamespace(get_depth_scale=lambda: 0.001)
        )
        return SimpleNamespace(get_device=lambda: device)

    def wait_for_frames(self, *, timeout_ms):
        assert timeout_ms > 0
        self.waits += 1
        return FakeFrameSet()

    def stop(self):
        self.stopped = True


class FakeRealSense:
    stream = SimpleNamespace(color="color", depth="depth")
    format = SimpleNamespace(bgr8="bgr8", z16="z16")
    camera_info = SimpleNamespace(
        name="name",
        serial_number="serial_number",
        physical_port="physical_port",
    )

    def __init__(self):
        self.pipelines = []
        self.configs = []

    def pipeline(self):
        value = FakePipeline()
        self.pipelines.append(value)
        return value

    def config(self):
        value = FakeConfig()
        self.configs.append(value)
        return value

    @staticmethod
    def align(_stream):
        return SimpleNamespace(process=lambda frames: frames)

    @staticmethod
    def context():
        device = SimpleNamespace(
            get_info=lambda key: {
                "name": "RealSense D435",
                "serial_number": "12345",
                "physical_port": "usb-1",
            }[key]
        )
        return SimpleNamespace(query_devices=lambda: [device])


def test_realsense_rig_binds_serial_converts_color_and_scales_depth():
    rs = FakeRealSense()
    rig = RealSenseRig(
        [CameraSpec("cam_high", "12345")],
        width=8,
        height=6,
        fps=30,
        depth=True,
        warmup_frames=1,
        rs_module=rs,
        clock=lambda: 50.0,
    )
    captured = rig.capture(timeout_ms=1000)["cam_high"]

    assert rs.configs[0].device == "12345"
    assert rs.configs[0].streams == [
        ("color", 8, 6, "bgr8", 30),
        ("depth", 8, 6, "z16", 30),
    ]
    assert captured.color_rgb[0, 0].tolist() == [3, 2, 1]
    assert captured.depth_mm.dtype == np.uint16
    assert int(captured.depth_mm[0, 0]) == 500
    assert captured.captured_at == 50.0
    assert captured.intrinsics["fx"] == 100.0
    assert rs.pipelines[0].waits == 2

    rig.close()
    assert rs.pipelines[0].stopped is True


def test_realsense_rig_close_is_idempotent():
    rs = FakeRealSense()
    rig = RealSenseRig(
        [CameraSpec("cam_high", "12345")],
        warmup_frames=0,
        rs_module=rs,
    )
    rig.start()
    rig.close()
    rig.close()
    assert rs.pipelines[0].stopped is True


def test_realsense_rig_releases_pipeline_whose_start_raises():
    class FailingPipeline(FakePipeline):
        def start(self, config):
            super().start(config)
            raise RuntimeError("device became busy")

    class FailingSecondRealSense(FakeRealSense):
        def pipeline(self):
            value = FailingPipeline() if self.pipelines else FakePipeline()
            self.pipelines.append(value)
            return value

    rs = FailingSecondRealSense()
    rig = RealSenseRig(
        [CameraSpec("cam_high", "12345"), CameraSpec("cam_right", "67890")],
        warmup_frames=0,
        rs_module=rs,
    )

    with pytest.raises(RuntimeError, match="device became busy"):
        rig.start()

    assert [pipeline.stopped for pipeline in rs.pipelines] == [True, True]
    assert rig._pipelines == {}


def test_list_realsense_devices_uses_read_only_context_query():
    assert list_realsense_devices(rs_module=FakeRealSense()) == [
        {
            "name": "RealSense D435",
            "serial": "12345",
            "physical_port": "usb-1",
        }
    ]
