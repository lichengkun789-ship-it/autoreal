#!/usr/bin/env python3
"""Import legacy Piper calibration evidence without retaining a source dependency."""

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.json_io import write_private_json  # noqa: E402
from services.piper.calibration import MOUNTS  # noqa: E402
from services.piper.calibration_import import import_calibration_result  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path, help="calibration_result.json")
    parser.add_argument("--mount", required=True, choices=sorted(MOUNTS))
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--force", action="store_true", help="Replace an existing output file")
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    try:
        if output.exists() and not args.force:
            raise FileExistsError(f"output already exists: {output}; pass --force to replace it")
        candidate = import_calibration_result(args.source, mount=args.mount)
        write_private_json(output, candidate, directory_mode=0o700)
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    print(
        json.dumps(
            {
                "status": candidate["status"],
                "mount": candidate["mount"],
                "output": str(output),
                "motion_authorized": False,
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
