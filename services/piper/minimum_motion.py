"""Operator-only Piper minimum-motion characterization.

This module is intentionally outside the RAX Agent tool surface. It supports one
small joint command and a software-only last-target hold for supervised bring-up.
"""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time

from .hold import PiperHoldController, PiperJointCommandAdapter
from .safety import validate_joint_target


MAX_DELTA_RAD = 0.005
MAX_SPEED_PERCENT = 5
MAX_FEEDBACK_AGE_S = 0.1
MAX_OTHER_JOINT_TRAVEL_RAD = 0.002
MAX_TOTAL_ACTIVE_TRAVEL_RAD = 0.010
MAX_POST_HOLD_DRIFT_RAD = 0.002
MIN_OBSERVED_TRAVEL_RAD = 0.0005
MAX_COMMAND_WINDOW_S = 2.0
PROBE_MAX_AGE_SECONDS = 600.0
STARTUP_TIMEOUT_S = 2.0
STARTUP_POLL_INTERVAL_S = 0.05
FLOAT_COMPARISON_EPSILON_RAD = 1e-12
BASELINE_SAMPLES = 5
BASELINE_INTERVAL_S = 0.2
MAX_BASELINE_DRIFT_RAD = 0.0005
SOFTWARE_HOLD_ACKNOWLEDGEMENTS = (
    "operator_at_keyboard",
    "workspace_clear",
    "robot_base_secured",
    "no_payload",
    "gravity_descent_path_clear",
    "no_physical_estop_available",
    "software_hold_is_not_safety_rated",
    "process_can_fail_before_hold",
    "can_or_power_loss_can_defeat_hold",
    "no_automatic_return_or_recovery",
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def validate_software_hold_confirmation(value: dict) -> dict:
    if not isinstance(value, dict) or value.get("schema_version") != 1:
        raise ValueError("software-hold confirmation schema_version must be 1")
    if value.get("kind") != "piper_software_hold_only_confirmation":
        raise ValueError("confirmation kind must be piper_software_hold_only_confirmation")
    checked = {"schema_version": 1, "kind": value["kind"]}
    for field in ("operator", "robot_asset_id"):
        text = value.get(field)
        if not isinstance(text, str) or not text.strip():
            raise ValueError(f"confirmation.{field} must be a non-empty string")
        checked[field] = text.strip()
    acknowledgements = value.get("acknowledgements")
    if not isinstance(acknowledgements, dict):
        raise ValueError("confirmation.acknowledgements must be an object")
    for field in SOFTWARE_HOLD_ACKNOWLEDGEMENTS:
        if acknowledgements.get(field) is not True:
            raise ValueError(f"confirmation.acknowledgements.{field} must be true")
    checked["acknowledgements"] = {
        field: True for field in SOFTWARE_HOLD_ACKNOWLEDGEMENTS
    }
    checked["notes"] = str(value.get("notes", "")).strip()
    return checked


def validate_probe_receipt(path: Path, backend_options: dict) -> dict:
    raw = path.read_bytes()
    value = json.loads(raw)
    if value.get("kind") != "piper_readonly_probe" or value.get("status") != "passed":
        raise ValueError("probe receipt must be a passed piper_readonly_probe")
    if value.get("motion_commands_sent") is not False:
        raise ValueError("probe receipt must confirm that no motion command was sent")
    if value.get("tool_surface", {}).get("mutating_tools") != []:
        raise ValueError("probe receipt must show an empty mutating tool surface")
    expected = {
        side: spec["can_name"] for side, spec in backend_options.get("arms", {}).items()
    }
    observed = {
        side: details.get("before", {}).get("name")
        for side, details in value.get("can_interfaces", {}).items()
    }
    if expected != observed:
        raise ValueError("probe receipt CAN interfaces do not match backend options")
    finished_at = value.get("finished_at")
    try:
        finished = datetime.fromisoformat(finished_at)
        age_seconds = (datetime.now(timezone.utc) - finished).total_seconds()
    except (TypeError, ValueError) as exc:
        raise ValueError("probe receipt finished_at is missing or invalid") from exc
    if not 0 <= age_seconds <= PROBE_MAX_AGE_SECONDS:
        raise ValueError(
            f"probe receipt must be no older than {PROBE_MAX_AGE_SECONDS:.0f} seconds"
        )
    return {
        "path": str(path.resolve()),
        "sha256": hashlib.sha256(raw).hexdigest(),
        "finished_at": finished_at,
        "age_seconds": age_seconds,
        "can_interfaces": expected,
    }


def _enabled(interface) -> bool | None:
    message = interface.GetArmLowSpdInfoMsgs()
    motors = [getattr(message, f"motor_{index}") for index in range(1, 7)]
    if any(int(getattr(motor, "can_id", 0) or 0) == 0 for motor in motors):
        return None
    return all(
        bool(
            getattr(
                motor.foc_status,
                "driver_enable_status",
                False,
            )
        )
        for motor in motors
    )


def _travel(initial: dict, sample: dict) -> dict:
    return {
        side: [
            abs(float(after) - float(before))
            for before, after in zip(
                initial["arms"][side]["joints_rad"],
                sample["arms"][side]["joints_rad"],
            )
        ]
        for side in ("left", "right")
    }


def _per_joint_motion(initial: dict, samples: list[dict]) -> dict:
    result = {}
    for side in ("left", "right"):
        rows = []
        for index in range(6):
            origin = float(initial["arms"][side]["joints_rad"][index])
            signed = [
                float(sample["arms"][side]["joints_rad"][index]) - origin
                for sample in samples
            ]
            rows.append(
                {
                    "joint": index + 1,
                    "max_abs_travel_rad": max((abs(value) for value in signed), default=0.0),
                    "final_signed_travel_rad": signed[-1] if signed else 0.0,
                }
            )
        result[side] = rows
    return result


def _validate_characterization_target(
    current: list[float] | tuple[float, ...],
    target: list[float] | tuple[float, ...],
    *,
    active_joint: int,
) -> dict:
    hard_limits = validate_joint_target(
        current,
        target,
        margin_rad=0.0,
        max_step_rad=MAX_DELTA_RAD + FLOAT_COMPARISON_EPSILON_RAD,
    )
    active_margin = validate_joint_target(
        current,
        target,
        margin_rad=0.02,
        max_step_rad=MAX_DELTA_RAD + FLOAT_COMPARISON_EPSILON_RAD,
    )
    rows = deepcopy(hard_limits["joints"])
    rows[active_joint - 1] = active_margin["joints"][active_joint - 1]
    active_row = rows[active_joint - 1]
    if not active_row["safe_min_rad"] <= active_row["target_rad"] <= active_row["safe_max_rad"]:
        if "outside_joint_limit_margin" not in active_row["reasons"]:
            active_row["reasons"].append("outside_joint_limit_margin")
    violations = [row for row in rows if row["reasons"]]
    return {
        "accepted": not violations,
        "max_delta_rad": max(row["delta_rad"] for row in rows),
        "active_joint": active_joint,
        "active_joint_margin_rad": 0.02,
        "unchanged_joint_margin_rad": 0.0,
        "joints": rows,
        "violations": violations,
    }


class PiperMinimumMotionRunner:
    """Run one bounded operator command and finish in a last-target hold latch."""

    def __init__(self, reader, *, sleeper=time.sleep, monotonic=time.monotonic):
        self.reader = reader
        self.sleeper = sleeper
        self.monotonic = monotonic

    def _wait_for_fresh_state(self, hold: PiperHoldController) -> tuple[dict, dict]:
        attempts = math.ceil(STARTUP_TIMEOUT_S / STARTUP_POLL_INTERVAL_S) + 1
        last_error = None
        for attempt in range(attempts):
            snapshot = self.reader.snapshot()
            try:
                return snapshot, hold.fresh_targets(snapshot)
            except (TypeError, ValueError) as exc:
                last_error = exc
            if attempt + 1 < attempts:
                self.sleeper(STARTUP_POLL_INTERVAL_S)
        raise TimeoutError(
            f"dual-arm feedback did not become fresh within {STARTUP_TIMEOUT_S:.1f}s: "
            f"{last_error}"
        )

    def _wait_for_enable_feedback(self, interface) -> bool:
        attempts = math.ceil(STARTUP_TIMEOUT_S / STARTUP_POLL_INTERVAL_S) + 1
        for attempt in range(attempts):
            enabled = _enabled(interface)
            if enabled is not None:
                return enabled
            if attempt + 1 < attempts:
                self.sleeper(STARTUP_POLL_INTERVAL_S)
        raise TimeoutError(
            f"motor enable feedback did not become complete within {STARTUP_TIMEOUT_S:.1f}s"
        )

    def _collect_stable_baseline(self, hold: PiperHoldController, initial: dict) -> tuple[dict, dict]:
        samples = [initial]
        for _ in range(BASELINE_SAMPLES - 1):
            self.sleeper(BASELINE_INTERVAL_S)
            sample = self.reader.snapshot()
            hold.fresh_targets(sample)
            samples.append(sample)
        per_joint = _per_joint_motion(samples[0], samples)
        max_drift = max(
            row["max_abs_travel_rad"]
            for rows in per_joint.values()
            for row in rows
        )
        motion_statuses = {
            side: [
                sample["arms"][side]["controller"].get("motion_status")
                for sample in samples
            ]
            for side in ("left", "right")
        }
        summary = {
            "samples": samples,
            "per_joint": per_joint,
            "max_joint_drift_rad": max_drift,
            "motion_statuses": motion_statuses,
            "motion_status_is_advisory": True,
            "accepted": max_drift <= MAX_BASELINE_DRIFT_RAD,
            "limit_rad": MAX_BASELINE_DRIFT_RAD,
        }
        return samples[-1], summary

    def run(
        self,
        *,
        arm: str,
        joint: int,
        delta_rad: float,
        speed_percent: int,
        operator_lease_id: str,
        execute: bool,
        allow_enable: bool = False,
        observation_seconds: float = 0.5,
        drift_samples: int = 5,
        drift_interval_seconds: float = 0.2,
        authorization_guard=None,
    ) -> dict:
        receipt = {
            "schema_version": 1,
            "kind": "piper_minimum_motion_characterization",
            "status": "failed",
            "started_at": _utc_now(),
            "software_hold_only": True,
            "safety_rated_stop_available": False,
            "agent_tool_exposed": False,
            "requested": {
                "arm": arm,
                "joint": joint,
                "delta_rad": delta_rad,
                "speed_percent": speed_percent,
                "execute": execute,
                "allow_enable": allow_enable,
            },
            "motion_command_attempted": False,
            "automatic_return_attempted": False,
            "authorization_guarded": authorization_guard is not None,
        }
        reasons = []
        if arm not in ("left", "right"):
            reasons.append("arm must be left or right")
        if not isinstance(joint, int) or not 1 <= joint <= 6:
            reasons.append("joint must be in 1..6")
        if not math.isfinite(delta_rad) or delta_rad == 0 or abs(delta_rad) > MAX_DELTA_RAD:
            reasons.append(f"absolute delta must be in (0, {MAX_DELTA_RAD}] rad")
        if not isinstance(speed_percent, int) or not 1 <= speed_percent <= MAX_SPEED_PERCENT:
            reasons.append(f"speed_percent must be in 1..{MAX_SPEED_PERCENT}")
        if observation_seconds < 0 or not 1 <= drift_samples <= 20 or drift_interval_seconds < 0:
            reasons.append("observation and drift sampling parameters are invalid")
        if reasons:
            receipt["failure_reasons"] = reasons
            receipt["finished_at"] = _utc_now()
            return receipt

        interfaces = self.reader.interfaces
        commander = PiperJointCommandAdapter(interfaces, speed_percent=speed_percent)
        hold = PiperHoldController(
            self.reader, commander, feedback_max_age_s=MAX_FEEDBACK_AGE_S
        )
        try:
            initial, targets = self._wait_for_fresh_state(hold)
        except BaseException as exc:
            receipt["failure_reasons"] = [f"{type(exc).__name__}: {exc}"]
            receipt["finished_at"] = _utc_now()
            return receipt
        try:
            enabled_before = self._wait_for_enable_feedback(interfaces[arm])
        except BaseException as exc:
            receipt["failure_reasons"] = [f"{type(exc).__name__}: {exc}"]
            receipt["finished_at"] = _utc_now()
            return receipt
        receipt["selected_arm_enabled_before"] = enabled_before
        if execute and not enabled_before:
            if not allow_enable:
                receipt["failure_reasons"] = [
                    "selected arm is disabled; rerun only after operator review with --allow-enable"
                ]
                receipt["finished_at"] = _utc_now()
                return receipt
            interfaces[arm].EnableArm(7)
            receipt["enable_command_sent"] = True
            enabled_after = False
            for attempt in range(20):
                self.sleeper(0.1)
                enabled_after = _enabled(interfaces[arm]) is True
                if enabled_after:
                    break
            if not enabled_after:
                receipt["failure_reasons"] = ["selected arm did not report all motors enabled"]
                receipt["finished_at"] = _utc_now()
                return receipt
            try:
                initial, targets = self._wait_for_fresh_state(hold)
            except BaseException as exc:
                receipt["failure_reasons"] = [f"{type(exc).__name__}: {exc}"]
                receipt["finished_at"] = _utc_now()
                return receipt
        else:
            receipt["enable_command_sent"] = False

        try:
            initial, baseline = self._collect_stable_baseline(hold, initial)
            receipt["baseline"] = baseline
            if not baseline["accepted"]:
                raise ValueError(
                    "pre-command joint drift "
                    f"{baseline['max_joint_drift_rad']:.6f} rad exceeds "
                    f"{MAX_BASELINE_DRIFT_RAD:.6f} rad"
                )
            targets = hold.fresh_targets(initial)
            target = list(targets[arm])
            target[joint - 1] += delta_rad
            validation = _validate_characterization_target(
                list(targets[arm]),
                target,
                active_joint=joint,
            )
            if not validation["accepted"]:
                failed = [
                    f"J{row['joint']}:{'/'.join(row['reasons'])}"
                    for row in validation["violations"]
                ]
                raise ValueError("target rejected: " + ", ".join(failed))
        except Exception as exc:
            receipt["initial_state"] = initial
            receipt["failure_reasons"] = [f"{type(exc).__name__}: {exc}"]
            receipt["finished_at"] = _utc_now()
            return receipt

        targets = deepcopy(targets)
        targets[arm] = tuple(target)
        receipt["initial_state"] = initial
        receipt["validated_target_rad"] = {
            side: list(values) for side, values in targets.items()
        }
        receipt["target_validation"] = validation
        if not execute:
            receipt["status"] = "planned"
            receipt["finished_at"] = _utc_now()
            return receipt

        if authorization_guard is not None:
            try:
                authorization_guard()
            except BaseException as exc:
                receipt["failure_reasons"] = [f"{type(exc).__name__}: {exc}"]
                receipt["finished_at"] = _utc_now()
                return receipt

        hold.operator_arm_for_characterization(
            operator_lease_id=operator_lease_id,
            operator_protocol_acknowledged=True,
            passive_probe_passed=True,
        )
        hold.record_last_commanded_targets(targets, hold_sides=(arm,))
        command_attempted = False
        failure = None
        try:
            command_attempted = True
            receipt["motion_command_attempted"] = True
            receipt["command_monotonic_s"] = self.monotonic()
            receipt["native_command"] = list(commander.command_position(arm, target))
            receipt["command_return_monotonic_s"] = self.monotonic()
            if authorization_guard is None:
                self.sleeper(observation_seconds)
            else:
                remaining = observation_seconds
                while remaining > 0:
                    authorization_guard()
                    interval = min(0.05, remaining)
                    self.sleeper(interval)
                    remaining -= interval
                authorization_guard()
            receipt["pre_hold_state"] = self.reader.snapshot()
        except BaseException as exc:
            failure = f"{type(exc).__name__}: {exc}"
        finally:
            if command_attempted:
                receipt["hold_request_monotonic_s"] = self.monotonic()
                try:
                    receipt["hold"] = hold.request_position_hold(
                        failure or "bounded minimum-motion command complete"
                    )
                    receipt["hold_return_monotonic_s"] = self.monotonic()
                except BaseException as exc:
                    receipt["hold"] = hold.status()
                    hold_failure = f"{type(exc).__name__}: {exc}"
                    failure = (
                        f"{failure}; hold failed: {hold_failure}"
                        if failure
                        else hold_failure
                    )

        receipt["post_hold_samples"] = []
        if hold.status()["hold_sdk_calls_returned"]:
            for index in range(drift_samples):
                try:
                    receipt["post_hold_samples"].append(self.reader.snapshot())
                except BaseException as exc:
                    failure = failure or (
                        f"post-hold feedback failed: {type(exc).__name__}: {exc}"
                    )
                    break
                if index + 1 < drift_samples:
                    try:
                        self.sleeper(drift_interval_seconds)
                    except BaseException as exc:
                        failure = failure or (
                            "post-hold observation interrupted: "
                            f"{type(exc).__name__}: {exc}"
                        )
                        break

        samples = receipt["post_hold_samples"]
        if samples:
            travels = [_travel(initial, sample) for sample in samples]
            active_travel = max(value[arm][joint - 1] for value in travels)
            other_travel = max(
                value[side][index]
                for value in travels
                for side in ("left", "right")
                for index in range(6)
                if (side, index + 1) != (arm, joint)
            )
            hold_origin = samples[0]
            drift = [_travel(hold_origin, sample) for sample in samples[1:]]
            post_hold_drift = max(
                (
                    amount
                    for value in drift
                    for amounts in value.values()
                    for amount in amounts
                ),
                default=0.0,
            )
            receipt["measurements"] = {
                "active_joint_travel_rad": active_travel,
                "max_other_joint_travel_rad": other_travel,
                "max_post_hold_drift_rad": post_hold_drift,
                "command_window_s": (
                    receipt["hold_request_monotonic_s"]
                    - receipt["command_monotonic_s"]
                ),
                "per_joint": _per_joint_motion(initial, samples),
            }
            if active_travel < MIN_OBSERVED_TRAVEL_RAD:
                reasons.append("active joint movement was not observed")
            if active_travel > MAX_TOTAL_ACTIVE_TRAVEL_RAD:
                reasons.append("active joint travel exceeded characterization limit")
            if other_travel > MAX_OTHER_JOINT_TRAVEL_RAD:
                reasons.append("another joint moved beyond characterization limit")
            if post_hold_drift > MAX_POST_HOLD_DRIFT_RAD:
                reasons.append("post-hold drift exceeded characterization limit")
            if receipt["measurements"]["command_window_s"] > MAX_COMMAND_WINDOW_S:
                reasons.append("command window exceeded characterization limit")
        else:
            reasons.append("no post-hold feedback samples were captured")
        if failure:
            reasons.append(failure)
        receipt["status"] = "passed" if not reasons else "failed"
        if reasons:
            receipt["failure_reasons"] = reasons
        receipt["finished_at"] = _utc_now()
        return receipt
