# Piper camera observations

The Piper backend owns RealSense cameras directly through `pyrealsense2`. Camera
support is independent from the Piper CAN/SDK adapter and does not import or execute
code from another workspace.

## Profiles

- `state-only`: does not construct or open a camera pipeline.
- `rgb-only`: `piper_observe` returns JPEG previews and records full RGB frames.
- `rgbd`: also records color-aligned 16-bit depth PNGs in millimetres.

Selecting `rgb-only` or `rgbd` requires a non-empty `cameras` list in the private
backend options. Each entry contains a stable logical name and the physical RealSense
serial number. Serial numbers stay outside source-controlled examples.

```json
{
  "cameras": [
    {"name": "cam_high", "serial": "REAL_SERIAL"}
  ]
}
```

Use `.venv/bin/python scripts/list_piper_cameras.py` to list connected serial numbers
without opening streams. Start with one camera, verify its view, then add the wrist
cameras. A serial number binds a logical view to a physical device and prevents USB
enumeration order from silently swapping cameras.

## Observation artifacts

One `piper_observe` call creates:

```text
<session>/agent-workspace/runtime/frames/<frame_id>/
  cam_high.jpg
  cam_high_depth.png       # rgbd only
  meta.json
```

`meta.json` records camera intrinsics and timestamps plus measured dual-arm state
before and after capture. It also reports the largest measured joint change during the
capture interval. The MCP result contains downscaled JPEG image blocks in the order
listed by `images_in_order`; base64 image data is not duplicated into `meta.json`.

## Current boundary

The camera backend supplies images, aligned depth, and color-camera intrinsics. It
does not yet provide calibrated camera-to-robot extrinsics, pixel-to-base-frame
conversion, object detection, continuous recording, or visual servoing. Therefore an
Agent can inspect a button and describe what it sees, but it cannot yet convert a
button pixel into a safe Piper Cartesian target. Adding that path requires calibration,
Piper FK/IK, workspace and collision guards, and an operator-characterized Cartesian
motion tool.

Camera observation never enables motion. `agent_motion.enabled` remains an independent
operator setting, and its current tool is still limited to an evidence-bound single
joint step.
