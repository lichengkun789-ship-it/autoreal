"""Session-owned RGB-D video and Piper telemetry recording for one motion."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import math
from pathlib import Path
import threading
import time
from typing import Any
import uuid

import numpy as np

from services.json_io import jsonable, write_private_json


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass(frozen=True)
class EpisodeRecordingSettings:
    fps: float = 5.0
    save_depth: bool = True

    @classmethod
    def from_options(cls, value, *, required: bool = False):
        value = {} if value is None else value
        if not isinstance(value, dict):
            raise ValueError("click_bell recording options must be an object")
        unknown = set(value) - {"enabled", "fps", "save_depth"}
        if unknown:
            raise ValueError(f"unknown click_bell recording options: {sorted(unknown)}")
        enabled = value.get("enabled", False)
        if type(enabled) is not bool:
            raise ValueError("click_bell recording enabled must be true or false")
        if required and not enabled:
            raise ValueError("guarded ClickBell press requires recording.enabled=true")
        if not enabled:
            return None
        fps = float(value.get("fps", 5.0))
        save_depth = value.get("save_depth", True)
        if not math.isfinite(fps) or not 1.0 <= fps <= 10.0:
            raise ValueError("click_bell recording fps must be in 1..10")
        if type(save_depth) is not bool:
            raise ValueError("click_bell recording save_depth must be true or false")
        return cls(fps=fps, save_depth=save_depth)


class MotionEpisodeRecorder:
    """Record camera streams in a worker and motion samples on the control thread."""

    def __init__(
        self,
        root: Path,
        cameras,
        camera_names,
        settings: EpisodeRecordingSettings,
        *,
        camera_timeout_ms: int = 3000,
        cv2_module=None,
        clock=time.time,
        monotonic=time.monotonic,
    ):
        self.root = Path(root)
        self.cameras = cameras
        self.camera_names = tuple(camera_names)
        self.settings = settings
        self.camera_timeout_ms = int(camera_timeout_ms)
        self.cv2 = cv2_module
        self.clock = clock
        self.monotonic = monotonic
        if not self.camera_names:
            raise ValueError("episode recording requires at least one camera")
        if self.camera_timeout_ms <= 0:
            raise ValueError("camera_timeout_ms must be positive")

        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.recording_id = f"{stamp}_{uuid.uuid4().hex[:8]}"
        self.directory = self.root / "recordings" / self.recording_id
        self._stop = threading.Event()
        self._thread = None
        self._writers: dict[str, Any] = {}
        self._camera_samples: list[dict] = []
        self._robot_samples: list[dict] = []
        self._last_rgb: dict[str, np.ndarray] = {}
        self._errors: list[str] = []
        self._lock = threading.Lock()
        self._started_at = None
        self._finished_at = None

    def _cv2(self):
        if self.cv2 is None:
            import cv2

            self.cv2 = cv2
        return self.cv2

    def _open_writer(self, name: str, rgb: np.ndarray):
        cv2 = self._cv2()
        height, width = rgb.shape[:2]
        path = self.directory / "video" / f"{name}.mp4"
        path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        writer = cv2.VideoWriter(
            str(path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            self.settings.fps,
            (int(width), int(height)),
        )
        if not writer.isOpened():
            raise RuntimeError(f"cannot open video writer for {name}")
        self._writers[name] = writer

    def _write_capture(self, frames: dict, sequence: int) -> None:
        cv2 = self._cv2()
        sample = {"sequence": sequence, "captured_at": float(self.clock()), "cameras": {}}
        for name in self.camera_names:
            frame = frames[name]
            rgb = np.asarray(frame.color_rgb, dtype=np.uint8)
            if rgb.ndim != 3 or rgb.shape[2] != 3:
                raise ValueError(f"camera {name} returned an invalid RGB frame")
            if name not in self._writers:
                self._open_writer(name, rgb)
            bgr = rgb[:, :, ::-1]
            self._writers[name].write(bgr)
            self._last_rgb[name] = rgb.copy()

            stills = self.directory / "stills"
            if sequence == 0:
                stills.mkdir(parents=True, exist_ok=True, mode=0o700)
                if not cv2.imwrite(str(stills / f"{name}_before.jpg"), bgr):
                    raise RuntimeError(f"cannot save initial still for {name}")

            depth_path = None
            if self.settings.save_depth:
                if frame.depth_mm is None:
                    raise RuntimeError(f"camera {name} has no depth frame")
                depth_dir = self.directory / "depth" / name
                depth_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
                depth_path = depth_dir / f"{sequence:06d}.png"
                if not cv2.imwrite(str(depth_path), np.asarray(frame.depth_mm, dtype=np.uint16)):
                    raise RuntimeError(f"cannot save depth frame for {name}")

            sample["cameras"][name] = {
                "captured_at": float(frame.captured_at),
                "device_timestamp_ms": frame.device_timestamp_ms,
                "depth": None if depth_path is None else str(depth_path),
            }
        with self._lock:
            self._camera_samples.append(sample)

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("episode recorder is already running")
        self.directory.mkdir(parents=True, exist_ok=False, mode=0o700)
        self._started_at = _utc_now()
        try:
            initial = self.cameras.capture(
                self.camera_names, timeout_ms=self.camera_timeout_ms
            )
            self._write_capture(initial, 0)
        except BaseException:
            self._release_writers()
            raise
        self._thread = threading.Thread(
            target=self._camera_loop,
            name=f"piper-recorder-{self.recording_id}",
            daemon=True,
        )
        self._thread.start()

    def _camera_loop(self) -> None:
        sequence = 1
        period = 1.0 / self.settings.fps
        deadline = self.monotonic() + period
        while not self._stop.wait(max(0.0, deadline - self.monotonic())):
            try:
                frames = self.cameras.capture(
                    self.camera_names, timeout_ms=self.camera_timeout_ms
                )
                self._write_capture(frames, sequence)
                sequence += 1
            except BaseException as exc:
                with self._lock:
                    self._errors.append(f"{type(exc).__name__}: {exc}")
                return
            deadline += period

    def record_robot_sample(self, arm: str, state: dict) -> None:
        try:
            sample = {
                "recorded_at": float(self.clock()),
                "arm": arm,
                "joints_rad": state["joints"],
                "effort_nm": state["effort"],
                "speed_rad_s": state["speed"],
                "gripper_width_m": state["gripper_width_m"],
            }
            with self._lock:
                self._robot_samples.append(jsonable(sample))
        except BaseException as exc:
            with self._lock:
                self._errors.append(f"robot sample: {type(exc).__name__}: {exc}")

    def _release_writers(self) -> None:
        for writer in self._writers.values():
            try:
                writer.release()
            except Exception:
                pass
        self._writers.clear()

    def stop(self) -> dict:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(5.0, self.camera_timeout_ms / 1000.0 + 1.0))
            if self._thread.is_alive():
                self._errors.append("camera recorder thread did not stop")
        self._finished_at = _utc_now()
        self._release_writers()

        cv2 = self._cv2()
        stills = self.directory / "stills"
        for name, rgb in self._last_rgb.items():
            if not cv2.imwrite(str(stills / f"{name}_after.jpg"), rgb[:, :, ::-1]):
                self._errors.append(f"cannot save final still for {name}")

        with self._lock:
            camera_samples = list(self._camera_samples)
            robot_samples = list(self._robot_samples)
            errors = list(self._errors)
        robot_path = self.directory / "robot_state.json"
        camera_path = self.directory / "camera_timestamps.json"
        write_private_json(robot_path, {"samples": robot_samples}, directory_mode=0o700)
        write_private_json(camera_path, {"samples": camera_samples}, directory_mode=0o700)
        status = "passed" if camera_samples and robot_samples and not errors else "failed"
        manifest = {
            "schema_version": 1,
            "kind": "piper_motion_episode_recording",
            "status": status,
            "recording_id": self.recording_id,
            "started_at": self._started_at,
            "finished_at": self._finished_at,
            "fps": self.settings.fps,
            "save_depth": self.settings.save_depth,
            "cameras": list(self.camera_names),
            "camera_samples": len(camera_samples),
            "robot_samples": len(robot_samples),
            "video": {
                name: str(self.directory / "video" / f"{name}.mp4")
                for name in self.camera_names
            },
            "before_stills": {
                name: str(stills / f"{name}_before.jpg") for name in self.camera_names
            },
            "after_stills": {
                name: str(stills / f"{name}_after.jpg") for name in self.camera_names
            },
            "robot_state": str(robot_path),
            "camera_timestamps": str(camera_path),
            "errors": errors,
        }
        manifest_path = self.directory / "manifest.json"
        write_private_json(manifest_path, manifest, directory_mode=0o700)
        return {**manifest, "manifest": str(manifest_path), "directory": str(self.directory)}
