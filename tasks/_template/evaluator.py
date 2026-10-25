"""Task-local evidence evaluator template."""


def evaluate(evidence: dict) -> dict:
    return {
        "status": "unverified",
        "success": None,
        "reason": "Implement observable completion checks for this task.",
        "evidence": evidence,
    }
