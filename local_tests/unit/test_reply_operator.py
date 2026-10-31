import json
from pathlib import Path
import subprocess
import sys

from services.controller import operator


PROJECT = Path(__file__).resolve().parents[2]


def test_reply_operator_answers_the_only_pending_request(tmp_path):
    root = tmp_path / "session"
    root.mkdir()
    (root / "operator-input.json").write_text("{}")
    request = operator.open_request(root, "visual_confirmation", "Did it work?")

    completed = subprocess.run(
        [
            sys.executable,
            str(PROJECT / "scripts" / "reply_operator.py"),
            "--session",
            str(root / "operator-input.json"),
            "--result",
            "success",
            "--note",
            "button moved",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    result = json.loads(completed.stdout)
    assert result["request_id"] == request["id"]
    assert result["answer"]["reply"] == "success: button moved"


def test_reply_operator_refuses_when_no_request_is_pending(tmp_path):
    root = tmp_path / "session"
    root.mkdir()
    (root / "operator-input.json").write_text("{}")

    completed = subprocess.run(
        [
            sys.executable,
            str(PROJECT / "scripts" / "reply_operator.py"),
            "--session",
            str(root),
            "--result",
            "unclear",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert "found 0" in completed.stderr
