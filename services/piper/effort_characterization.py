"""Read-only stationary Piper motor-effort characterization."""

from __future__ import annotations

from datetime import datetime, timezone
import math
import time

import numpy as np


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def effort_statistics(samples) -> dict:
    values = np.asarray(samples, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 6 or values.shape[0] < 2:
        raise ValueError("effort statistics require at least two six-joint samples")
    if not np.all(np.isfinite(values)):
        raise ValueError("effort samples must be finite")
    rows = []
    for index in range(6):
        column = values[:, index]
        median = float(np.median(column))
        rows.append(
            {
                "joint": index + 1,
                "mean_nm": round(float(np.mean(column)), 6),
                "median_nm": round(median, 6),
                "std_nm": round(float(np.std(column)), 6),
                "mad_nm": round(float(np.median(np.abs(column - median))), 6),
                "min_nm": round(float(np.min(column)), 6),
                "max_nm": round(float(np.max(column)), 6),
                "peak_to_peak_nm": round(float(np.ptp(column)), 6),
            }
        )
    return {
        "joints": rows,
        "max_stationary_peak_to_peak_nm": max(row["peak_to_peak_nm"] for row in rows),
    }


class PiperEffortNoiseRunner:
    def __init__(self, reader, *, sleeper=time.sleep):
        self.reader = reader
        self.sleeper = sleeper

    @staticmethod
    def _arm(snapshot: dict, side: str, feedback_max_age_s: float) -> tuple[np.ndarray, np.ndarray]:
        arm = snapshot.get("arms", {}).get(side, {})
        feedback = arm.get("feedback", {})
        for source in ("joint", "status", "motor"):
            age = feedback.get(f"{source}_age_s")
            if not isinstance(age, (int, float)) or not 0 <= age <= feedback_max_age_s:
                raise ValueError(f"{side} {source} feedback is stale or missing")
        controller = arm.get("controller", {})
        if controller.get("arm_status") != 0 or any(controller.get("errors", {}).values()):
            raise ValueError(f"{side} controller is not healthy")
        joints = np.asarray(arm.get("joints_rad"), dtype=np.float64)
        effort = np.asarray(arm.get("motor", {}).get("effort_nm"), dtype=np.float64)
        if joints.shape != (6,) or effort.shape != (6,) or not np.all(
            np.isfinite(np.concatenate((joints, effort)))
        ):
            raise ValueError(f"{side} joint or effort feedback is invalid")
        return joints, effort

    def run(
        self,
        *,
        arm: str,
        samples: int = 200,
        interval_s: float = 0.05,
        feedback_max_age_s: float = 0.25,
        max_joint_drift_rad: float = 0.001,
        startup_timeout_s: float = 3.0,
    ) -> dict:
        receipt = {
            "schema_version": 1,
            "kind": "piper_stationary_effort_noise",
            "status": "failed",
            "started_at": _utc_now(),
            "arm": arm,
            "motion_commands_sent": False,
            "threshold_admitted": False,
        }
        try:
            if arm not in ("left", "right"):
                raise ValueError("arm must be left or right")
            if type(samples) is not int or not 20 <= samples <= 2000:
                raise ValueError("samples must be in 20..2000")
            if not math.isfinite(interval_s) or not 0 < interval_s <= 1:
                raise ValueError("interval_s must be in (0, 1]")
            if not math.isfinite(startup_timeout_s) or startup_timeout_s <= 0:
                raise ValueError("startup_timeout_s must be positive and finite")
            deadline = time.monotonic() + startup_timeout_s
            while True:
                try:
                    ready_joints, ready_effort = self._arm(
                        self.reader.snapshot(), arm, feedback_max_age_s
                    )
                    break
                except ValueError as exc:
                    if "stale or missing" not in str(exc):
                        raise
                    if time.monotonic() >= deadline:
                        raise TimeoutError(
                            f"Piper feedback did not become ready: {exc}"
                        ) from exc
                    self.sleeper(min(interval_s, 0.05))
            first_joints = None
            efforts = []
            max_drift = 0.0
            for index in range(samples):
                if index == 0:
                    joints, effort = ready_joints, ready_effort
                else:
                    joints, effort = self._arm(
                        self.reader.snapshot(), arm, feedback_max_age_s
                    )
                if first_joints is None:
                    first_joints = joints.copy()
                max_drift = max(max_drift, float(np.max(np.abs(joints - first_joints))))
                efforts.append(effort.tolist())
                if index + 1 < samples:
                    self.sleeper(interval_s)
            receipt.update(
                {
                    "samples": efforts,
                    "samples_collected": len(efforts),
                    "interval_s": interval_s,
                    "max_joint_drift_rad": max_drift,
                    "statistics": effort_statistics(efforts),
                }
            )
            if max_drift > max_joint_drift_rad:
                raise RuntimeError(
                    f"arm moved during stationary sampling ({max_drift:.6f} rad)"
                )
            receipt["status"] = "passed"
            receipt["threshold_note"] = (
                "stationary noise is only a lower bound; characterize pose-dependent "
                "effort during a no-contact approach before admitting a contact threshold"
            )
        except BaseException as exc:
            receipt.setdefault("failure_reasons", []).append(f"{type(exc).__name__}: {exc}")
        receipt["finished_at"] = _utc_now()
        return receipt
