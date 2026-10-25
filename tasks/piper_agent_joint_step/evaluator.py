"""Evaluate the host-side evidence from one Piper Agent commissioning run."""

EXPECTED_ARGUMENTS = {"arm": "right", "joint": 1, "delta_rad": 0.005}


def evaluate(evidence: dict) -> dict:
    reasons = []
    requests = evidence.get("motion_requests") or []
    receipts = evidence.get("motion_receipts") or []

    if evidence.get("launcher_returncode") != 0:
        reasons.append(f"Agent launcher returned {evidence.get('launcher_returncode')!r}")
    if len(requests) != 1:
        reasons.append(f"expected exactly one piper_move_joint request, observed {len(requests)}")
    elif requests[0].get("arguments") != EXPECTED_ARGUMENTS:
        reasons.append(f"unexpected motion arguments: {requests[0].get('arguments')!r}")

    if len(receipts) != 1:
        reasons.append(f"expected exactly one motion receipt, observed {len(receipts)}")
    else:
        receipt = receipts[0]
        requested = receipt.get("requested") or {}
        hold = receipt.get("hold") or {}
        if receipt.get("kind") != "piper_agent_joint_step":
            reasons.append("motion receipt has the wrong kind")
        if receipt.get("status") != "passed" or receipt.get("motion_command_attempted") is not True:
            reasons.append("motion receipt does not confirm one passed executed command")
        observed = {key: requested.get(key) for key in EXPECTED_ARGUMENTS}
        if observed != EXPECTED_ARGUMENTS:
            reasons.append(f"motion receipt requested unexpected arguments: {observed!r}")
        if hold.get("state") != "hold_latched" or hold.get("hold_sdk_calls_returned") is not True:
            reasons.append("motion receipt does not confirm a returned last-target hold")

    final_arming = evidence.get("final_arming") or {}
    if final_arming.get("armed") is not False:
        reasons.append("session was not disarmed after the Agent exited")

    success = not reasons
    return {
        "status": "passed" if success else "failed",
        "success": success,
        "reasons": reasons,
        "summary": {
            "motion_request_count": len(requests),
            "motion_receipt_count": len(receipts),
            "final_armed": final_arming.get("armed"),
        },
    }
