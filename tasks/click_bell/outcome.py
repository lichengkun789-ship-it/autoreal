"""Bind ClickBell task evaluation to a durable human visual confirmation."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import re

from services.controller import operator
from services.json_io import write_private_json

from .evaluator import evaluate


OUTCOMES = {
    "success": True,
    "成功": True,
    "failure": False,
    "失败": False,
    "unclear": None,
    "不确定": None,
}


def _parse_reply(text: str) -> tuple[str, bool | None]:
    normalized = str(text).strip()
    first = re.split(r"[\s,:，：]", normalized, maxsplit=1)[0].lower()
    if first not in OUTCOMES:
        raise ValueError(
            "visual confirmation must begin with success, failure, unclear, 成功, 失败, or 不确定"
        )
    canonical = {
        True: "success",
        False: "failure",
        None: "unclear",
    }[OUTCOMES[first]]
    return canonical, OUTCOMES[first]


class ClickBellOutcomeController:
    def __init__(self, root: Path, press_controller, *, gripper_controller=None):
        self.root = Path(root)
        self.press_controller = press_controller
        self.gripper_controller = gripper_controller
        self.last_result = None

    def _validated(self, press_id: str, request_id: str):
        motion = self.press_controller.last_motion
        if not isinstance(motion, dict) or motion.get("press_id") != press_id:
            raise ValueError("press_id is not the latest guarded press in this session")
        if motion.get("status") not in {"passed", "contact_unverified"}:
            raise ValueError("latest guarded press did not complete")
        if motion.get("retreat_status") != "passed":
            raise ValueError("latest guarded press has no confirmed retreat")
        if self.last_result is not None and self.last_result.get("press_id") == press_id:
            raise PermissionError("this guarded press already has a visual confirmation")
        gripper = None
        if self.gripper_controller is not None:
            gripper = self.gripper_controller.completed_task_evidence(press_id)
        answered = operator.answered_reply(
            self.root, request_id, kind="visual_confirmation"
        )
        if answered["answer"].get("done") is not True:
            raise ValueError("operator visual confirmation is not marked done")
        outcome, visual_success = _parse_reply(answered["answer"].get("reply", ""))
        return motion, gripper, answered, outcome, visual_success

    def validate_request(self, press_id: str, request_id: str) -> None:
        self._validated(press_id, request_id)

    def confirm(self, press_id: str, request_id: str) -> dict:
        motion, gripper, answered, outcome, visual_success = self._validated(
            press_id, request_id
        )
        reply = answered["answer"]["reply"]
        evaluation = evaluate(
            {
                "visual_success": visual_success,
                "visual_reason": f"Operator visual confirmation: {reply}",
            }
        )
        result = {
            "schema_version": 1,
            "kind": "click_bell_operator_visual_result",
            "press_id": press_id,
            "motion_receipt": motion["receipt"],
            "recording": motion["recording"],
            "gripper": gripper,
            "task_recordings": [
                *([] if gripper is None else gripper["recordings"][:1]),
                motion["recording"],
                *([] if gripper is None else gripper["recordings"][1:]),
            ],
            "operator_request_id": request_id,
            "operator_reply": reply,
            "operator": answered["answer"].get("by"),
            "outcome": outcome,
            "evaluation": evaluation,
            "confirmed_at": datetime.now(timezone.utc).isoformat(),
        }
        path = self.root / "click-bell-results" / f"{press_id}.json"
        write_private_json(path, result, directory_mode=0o700)
        self.last_result = {**result, "receipt": str(path)}
        return self.last_result

    def status(self):
        return self.last_result
