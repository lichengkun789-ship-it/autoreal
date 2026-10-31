#!/usr/bin/env python3
"""Answer the single pending operator request for one RAX session."""

import argparse
import json
from pathlib import Path
import sys


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from services.controller import operator  # noqa: E402


def session_root(value: Path) -> Path:
    path = value.expanduser().resolve()
    root = path.parent if path.name == "operator-input.json" else path
    if not (root / "operator-input.json").is_file():
        raise ValueError(f"not a RAX session: {root}")
    return root


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--request-id")
    parser.add_argument("--result", choices=("success", "failure", "unclear"), required=True)
    parser.add_argument("--note", default="")
    args = parser.parse_args()

    try:
        root = session_root(args.session)
        pending = operator.pending(root)
        if args.request_id:
            matches = [item for item in pending if item["id"] == args.request_id]
        else:
            matches = pending
        if len(matches) != 1:
            ids = [item["id"] for item in pending]
            raise ValueError(
                f"expected one matching pending request, found {len(matches)}; pending={ids}"
            )
        request = matches[0]
        note = args.note.strip()
        text = args.result if not note else f"{args.result}: {note}"
        answer = operator.reply(
            root,
            request["id"],
            text,
            done=True,
            by="operator-cli",
        )
    except Exception as exc:
        print(f"OPERATOR REPLY FAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    print(
        json.dumps(
            {
                "session": str(root),
                "request_id": request["id"],
                "kind": request["kind"],
                "answer": answer,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
