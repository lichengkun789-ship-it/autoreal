"""Evaluate click-bell task evidence without relying on sound."""


def evaluate(evidence: dict) -> dict:
    visual_success = evidence.get("visual_success")
    if visual_success is None:
        return {
            "status": "unverified",
            "success": None,
            "reason": "A before/after visual success assessment is required.",
        }
    success = visual_success is True
    return {
        "status": "passed" if success else "failed",
        "success": success,
        "reason": evidence.get("visual_reason") or (
            "Visual evidence confirms cap actuation."
            if success
            else "Visual evidence does not confirm cap actuation."
        ),
    }
