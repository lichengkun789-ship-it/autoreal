#!/usr/bin/env python3
"""Detect and localize the yellow click-bell press cap in a saved RGB-D frame."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from PIL import Image, ImageDraw


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from services.json_io import write_private_json  # noqa: E402
from tasks.click_bell.perception import localize_saved_frame  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frame_directory", type=Path)
    parser.add_argument("--camera", default="cam_right_wrist")
    parser.add_argument("--radius", type=int, default=3)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--overlay", type=Path)
    args = parser.parse_args()

    try:
        result = localize_saved_frame(
            args.frame_directory.expanduser().resolve(),
            args.camera,
            radius=args.radius,
        )
    except Exception as exc:
        result = {
            "schema_version": 1,
            "kind": "click_bell_press_cap_localization",
            "status": "failed",
            "error": {"type": type(exc).__name__, "message": str(exc)},
        }

    if result["status"] == "passed" and args.overlay:
        metadata = json.loads((args.frame_directory / "meta.json").read_text())
        image_path = args.frame_directory / metadata["cameras"][args.camera]["rgb"]
        image = Image.open(image_path).convert("RGB")
        draw = ImageDraw.Draw(image)
        draw.rectangle(result["bell"]["bbox_xyxy"], outline=(0, 170, 255), width=3)
        draw.rectangle(result["press_cap"]["bbox_xyxy"], outline=(255, 230, 0), width=3)
        u, v = result["press_pixel"]
        draw.line((u - 8, v, u + 8, v), fill=(255, 0, 0), width=2)
        draw.line((u, v - 8, u, v + 8), fill=(255, 0, 0), width=2)
        args.overlay.parent.mkdir(parents=True, exist_ok=True)
        image.save(args.overlay)
        result["overlay"] = str(args.overlay.resolve())

    if args.output:
        write_private_json(args.output, result, directory_mode=0o700)
        result["receipt"] = str(args.output.resolve())
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
