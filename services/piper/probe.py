"""Operator-side, read-only Piper bring-up probe."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
import time

from .backend import PiperBackend


IFF_UP = 0x1
STATISTICS = ("rx_packets", "tx_packets", "rx_errors", "tx_errors", "rx_dropped")


def _read(path: Path, default=None):
    try:
        return path.read_text().strip()
    except OSError:
        return default


def inspect_can_interface(name: str, *, sys_class_net: Path = Path("/sys/class/net")) -> dict:
    """Return Linux link counters without changing interface configuration."""
    root = sys_class_net / name
    if not root.exists():
        return {"name": name, "exists": False, "up": False}
    raw_flags = _read(root / "flags", "0")
    try:
        flags = int(raw_flags, 0)
    except (TypeError, ValueError):
        flags = 0
    counters = {}
    for statistic in STATISTICS:
        raw = _read(root / "statistics" / statistic)
        counters[statistic] = int(raw) if raw is not None else None
    return {
        "name": name,
        "exists": True,
        "up": bool(flags & IFF_UP),
        "operstate": _read(root / "operstate"),
        "flags": raw_flags,
        "statistics": counters,
    }


def _interface_delta(before: dict, after: dict) -> dict:
    result = {}
    for key in STATISTICS:
        first = before.get("statistics", {}).get(key)
        last = after.get("statistics", {}).get(key)
        result[key] = last - first if isinstance(first, int) and isinstance(last, int) else None
    return result


def run_readonly_probe(
    backend_options: dict,
    *,
    samples: int = 5,
    startup_seconds: float = 1.0,
    interval_seconds: float = 0.2,
    backend_factory=PiperBackend,
    sys_class_net: Path = Path("/sys/class/net"),
    sleeper=time.sleep,
) -> dict:
    """Collect several backend snapshots and return a machine-readable receipt."""
    if not 1 <= samples <= 100:
        raise ValueError("samples must be in 1..100")
    if startup_seconds < 0 or interval_seconds < 0:
        raise ValueError("probe delays must be non-negative")
    arms = backend_options.get("arms")
    if not isinstance(arms, dict) or set(arms) != {"left", "right"}:
        raise ValueError("backend options must define exactly left and right arms")
    names = {side: spec.get("can_name") for side, spec in arms.items() if isinstance(spec, dict)}
    if set(names) != {"left", "right"} or not all(isinstance(name, str) and name for name in names.values()):
        raise ValueError("each arm must define a non-empty can_name")
    if names["left"] == names["right"]:
        raise ValueError("left and right arms must use different CAN interfaces")

    started_at = datetime.now(timezone.utc).isoformat()
    before = {side: inspect_can_interface(name, sys_class_net=sys_class_net) for side, name in names.items()}
    receipt = {
        "schema_version": 1,
        "kind": "piper_readonly_probe",
        "status": "failed",
        "started_at": started_at,
        "can_queries_sent": False,
        "motion_commands_sent": False,
        "can_interfaces": {side: {"before": state} for side, state in before.items()},
        "samples_requested": samples,
        "samples": [],
    }
    unavailable = [side for side, state in before.items() if not state["exists"] or not state["up"]]
    if unavailable:
        receipt["failure_reasons"] = [f"{side} CAN interface is missing or down" for side in unavailable]
        receipt["finished_at"] = datetime.now(timezone.utc).isoformat()
        return receipt

    backend = None
    try:
        backend = backend_factory(SimpleNamespace(backend_options=backend_options))
        tools = [tool["name"] for tool in backend.tools()]
        mutating = sorted(getattr(backend, "mutating", ()))
        receipt["tool_surface"] = {"tools": tools, "mutating_tools": mutating}
        if mutating:
            raise RuntimeError(f"read-only probe refuses a backend with mutating tools: {mutating}")
        sleeper(startup_seconds)
        for index in range(samples):
            snapshot = backend.status()
            receipt["samples"].append(snapshot)
            if index + 1 < samples:
                sleeper(interval_seconds)
    except Exception as exc:  # Report operator-facing diagnostics in the receipt.
        receipt["failure_reasons"] = [f"{type(exc).__name__}: {exc}"]
    finally:
        if backend is not None:
            backend.close()

    after = {side: inspect_can_interface(name, sys_class_net=sys_class_net) for side, name in names.items()}
    for side in names:
        receipt["can_interfaces"][side]["after"] = after[side]
        receipt["can_interfaces"][side]["delta"] = _interface_delta(before[side], after[side])

    transmitted = [
        side
        for side in names
        if receipt["can_interfaces"][side]["delta"]["tx_packets"] not in (None, 0)
    ]
    if transmitted:
        receipt.setdefault("failure_reasons", []).append(
            "CAN transmit counter changed during passive probe: " + ", ".join(transmitted)
        )

    readiness = [sample.get("ready") is True for sample in receipt["samples"]]
    if len(readiness) == samples and all(readiness) and not receipt.get("failure_reasons"):
        ages = [
            arm["feedback"][key]
            for sample in receipt["samples"]
            for arm in sample["arms"].values()
            for key in ("joint_age_s", "status_age_s", "gripper_age_s")
            if arm["feedback"][key] is not None
        ]
        receipt["status"] = "passed"
        receipt["summary"] = {
            "ready_samples": sum(readiness),
            "max_feedback_age_s": max(ages) if ages else None,
        }
    elif not receipt.get("failure_reasons"):
        reasons = []
        for sample in receipt["samples"]:
            reasons.extend(sample.get("readiness_reasons", ()))
        receipt["failure_reasons"] = sorted(set(reasons)) or ["not every sample was ready"]
    receipt["finished_at"] = datetime.now(timezone.utc).isoformat()
    return receipt
