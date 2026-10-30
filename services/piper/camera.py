"""Read-only RealSense capture for the Piper backend.

The camera driver is intentionally independent from the Piper SDK.  It owns only
RealSense pipelines and returns RGB/depth frames to the host-owned RAX backend.
"""

from __future__ import annotations

from dataclasses import dataclass
import re
import time
from typing import Any, Iterable


@dataclass(frozen=True)
class CameraSpec:
    name: str
    serial: str
    mount: str | None = None
    extrinsics: str | None = None


@dataclass(frozen=True)
class CameraFrame:
    name: str
    serial: str
    color_rgb: Any
    depth_mm: Any | None
    captured_at: float
    device_timestamp_ms: float | None
    intrinsics: dict[str, Any]


def camera_specs(options: Any) -> tuple[CameraSpec, ...]:
    if not isinstance(options, list) or not options:
        raise ValueError(
            "rgb-only/rgbd Piper sessions require backend option 'cameras': "
            "a non-empty list of name/serial objects"
        )
    specs: list[CameraSpec] = []
    for index, value in enumerate(options):
        allowed = {"name", "serial", "mount", "extrinsics"}
        if not isinstance(value, dict) or not {"name", "serial"}.issubset(value) or set(value) - allowed:
            raise ValueError(
                f"cameras[{index}] requires name and serial, with optional mount and extrinsics"
            )
        name = value["name"]
        serial = value["serial"]
        if not isinstance(name, str) or re.fullmatch(r"[A-Za-z0-9_]+", name) is None:
            raise ValueError(f"cameras[{index}].name must contain letters, digits, or underscores")
        if not isinstance(serial, str) or not serial.strip():
            raise ValueError(f"cameras[{index}].serial must be a non-empty string")
        mount = value.get("mount")
        extrinsics = value.get("extrinsics")
        if (mount is None) != (extrinsics is None):
            raise ValueError(
                f"cameras[{index}].mount and extrinsics must be configured together"
            )
        if mount is not None and mount not in {
            "left_base",
            "right_base",
            "left_tcp",
            "right_tcp",
        }:
            raise ValueError(
                f"cameras[{index}].mount must be left_base, right_base, "
                "left_tcp, or right_tcp"
            )
        if extrinsics is not None and (not isinstance(extrinsics, str) or not extrinsics.strip()):
            raise ValueError(f"cameras[{index}].extrinsics must be a non-empty path")
        specs.append(
            CameraSpec(name=name, serial=serial, mount=mount, extrinsics=extrinsics)
        )
    names = [spec.name for spec in specs]
    serials = [spec.serial for spec in specs]
    if len(set(names)) != len(names):
        raise ValueError("camera names must be unique")
    if len(set(serials)) != len(serials):
        raise ValueError("camera serials must be unique")
    return tuple(specs)


def require_realsense():
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise RuntimeError(
            "RealSense observation requires pyrealsense2; install requirements-robot.txt"
        ) from exc
    return rs


def list_realsense_devices(*, rs_module=None) -> list[dict[str, str | None]]:
    """List attached RealSense devices without opening a streaming pipeline."""
    rs = rs_module or require_realsense()

    def device_info(device, key):
        try:
            return str(device.get_info(key))
        except Exception:
            return None

    devices = []
    for device in rs.context().query_devices():
        devices.append(
            {
                "name": device_info(device, rs.camera_info.name),
                "serial": device_info(device, rs.camera_info.serial_number),
                "physical_port": device_info(device, rs.camera_info.physical_port),
            }
        )
    return devices


class RealSenseRig:
    """Own multiple RealSense pipelines and capture synchronized-per-device frames."""

    def __init__(
        self,
        specs: Iterable[CameraSpec],
        *,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        depth: bool = False,
        warmup_frames: int = 15,
        rs_module=None,
        clock=time.time,
    ):
        self.specs = tuple(specs)
        if not self.specs:
            raise ValueError("at least one RealSense camera is required")
        if width <= 0 or height <= 0 or fps <= 0:
            raise ValueError("camera width, height, and fps must be positive")
        if warmup_frames < 0:
            raise ValueError("camera warmup_frames must be non-negative")
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.depth = bool(depth)
        self.warmup_frames = int(warmup_frames)
        self.rs = rs_module
        self.clock = clock
        self._pipelines: dict[str, Any] = {}
        self._aligners: dict[str, Any] = {}
        self._depth_scales: dict[str, float] = {}

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(spec.name for spec in self.specs)

    def start(self) -> None:
        if self._pipelines:
            return
        rs = self.rs or require_realsense()
        self.rs = rs
        try:
            for spec in self.specs:
                pipeline = rs.pipeline()
                config = rs.config()
                config.enable_device(spec.serial)
                config.enable_stream(
                    rs.stream.color,
                    self.width,
                    self.height,
                    rs.format.bgr8,
                    self.fps,
                )
                if self.depth:
                    config.enable_stream(
                        rs.stream.depth,
                        self.width,
                        self.height,
                        rs.format.z16,
                        self.fps,
                    )
                profile = pipeline.start(config)
                self._pipelines[spec.name] = pipeline
                if self.depth:
                    self._aligners[spec.name] = rs.align(rs.stream.color)
                    sensor = profile.get_device().first_depth_sensor()
                    self._depth_scales[spec.name] = float(sensor.get_depth_scale())
            for _ in range(self.warmup_frames):
                for pipeline in self._pipelines.values():
                    pipeline.wait_for_frames(timeout_ms=3000)
        except BaseException:
            self.close()
            raise

    @staticmethod
    def _intrinsics(frame) -> dict[str, Any]:
        profile = frame.profile.as_video_stream_profile()
        value = profile.intrinsics
        return {
            "width": int(value.width),
            "height": int(value.height),
            "fx": float(value.fx),
            "fy": float(value.fy),
            "ppx": float(value.ppx),
            "ppy": float(value.ppy),
            "model": str(value.model),
            "coeffs": [float(item) for item in value.coeffs],
        }

    def capture(
        self,
        names: Iterable[str] | None = None,
        *,
        timeout_ms: int = 3000,
    ) -> dict[str, CameraFrame]:
        if timeout_ms <= 0:
            raise ValueError("camera timeout_ms must be positive")
        self.start()
        selected = tuple(names or self.names)
        unknown = sorted(set(selected) - set(self.names))
        if unknown:
            raise ValueError(f"unknown Piper cameras: {unknown}")
        if len(set(selected)) != len(selected):
            raise ValueError("camera selection contains duplicates")

        import numpy as np

        frames: dict[str, CameraFrame] = {}
        specs = {spec.name: spec for spec in self.specs}
        for name in selected:
            frame_set = self._pipelines[name].wait_for_frames(timeout_ms=timeout_ms)
            if self.depth:
                frame_set = self._aligners[name].process(frame_set)
            color = frame_set.get_color_frame()
            if not color:
                raise RuntimeError(f"camera {name}: no color frame")
            depth_frame = frame_set.get_depth_frame() if self.depth else None
            if self.depth and not depth_frame:
                raise RuntimeError(f"camera {name}: no depth frame")

            bgr = np.asanyarray(color.get_data()).copy()
            rgb = bgr[:, :, ::-1].copy()
            depth_mm = None
            if depth_frame is not None:
                raw_depth = np.asanyarray(depth_frame.get_data()).copy()
                scale_mm = self._depth_scales[name] * 1000.0
                depth_mm = np.clip(
                    np.rint(raw_depth.astype(np.float64) * scale_mm), 0, 65535
                ).astype(np.uint16)
            timestamp = None
            try:
                timestamp = float(color.get_timestamp())
            except Exception:
                pass
            frames[name] = CameraFrame(
                name=name,
                serial=specs[name].serial,
                color_rgb=rgb,
                depth_mm=depth_mm,
                captured_at=float(self.clock()),
                device_timestamp_ms=timestamp,
                intrinsics=self._intrinsics(color),
            )
        return frames

    def close(self) -> None:
        pipelines = list(self._pipelines.values())
        self._pipelines.clear()
        self._aligners.clear()
        self._depth_scales.clear()
        for pipeline in pipelines:
            try:
                pipeline.stop()
            except Exception:
                pass
