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
serial number. Serial numbers stay outside source-controlled examples. `mount` and
`extrinsics` are optional and must be given together.

```json
{
  "cameras": [
    {"name": "cam_high", "serial": "REAL_SERIAL"}
  ]
}
```

A calibrated entry uses one of `left_base`, `right_base`, `left_tcp`, or
`right_tcp`. A stationary overview camera still names the arm base in which its
extrinsic matrix was solved:

```json
{
  "name": "cam_right_wrist",
  "serial": "REAL_SERIAL",
  "mount": "right_tcp",
  "extrinsics": "right_tcp_camera.json"
}
```

Relative extrinsics paths resolve under `PIPER_CALIBRATION_DIR`; absolute paths are
also accepted. The JSON calibration file contains a finite rigid 4x4 matrix with this
explicit convention:

```json
{
  "convention": "T_mount_camera",
  "matrix": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
}
```

The identity above documents the file shape only. Do not use it as real calibration.
`T_mount_camera` maps a point expressed in camera coordinates into the configured
mount frame. A `_base` camera is stationary relative to the named Piper arm base; a
`_tcp` camera is attached to the named arm's Piper SDK end-pose frame. One fixed-camera
matrix localizes into one named arm base. Supporting both arm bases requires a second
calibration or a separately validated transform between the two installations.

Existing standalone Piper calibration results can be converted without importing or
depending on their original project:

```bash
.venv/bin/python scripts/import_piper_calibration.py \
  --source /path/to/calibration_result.json \
  --mount left_base \
  --output /private/calibration/cam_high_left_base.json
```

For eye-in-hand results use `left_tcp` or `right_tcp`. The importer selects
`supplemental_eye_in_hand.T_gripper_camera`, validates the matrix, and records a source
SHA-256 plus the available quality metrics. Imported files are marked
`read_only_validation_required`; importing evidence never authorizes motion.

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
before and after capture. For calibrated cameras it also records the capture-time
`T_base_camera`. It reports the largest measured joint change during the capture
interval. The MCP result contains downscaled JPEG image blocks in the order listed by
`images_in_order`; base64 image data is not duplicated into `meta.json`.

The intrinsics reported by the currently connected RealSense stream are authoritative.
Values copied from simulation configs or an older calibration run are comparison
evidence only; the backend does not substitute them for capture-time intrinsics.

In a calibrated `rgbd` session, `piper_pixel_to_position` takes full-resolution pixels
from a saved frame. It selects the median non-zero depth in a configurable pixel
window, deprojects through the recorded RealSense intrinsics, and transforms the point
to the explicit `piper_left_base` or `piper_right_base` frame recorded with that
camera. The tool is read-only and is omitted when no camera has external calibration.

## Current boundary

The backend can load operator-supplied extrinsics and perform pixel-to-base conversion.
Calibration collection, solving, and held-out validation are provided by the Piper
calibration scripts. Read-only numerical IK and straight Cartesian joint-path checks
are available through `piper_plan_pose`; collision-aware planning, visual servoing, and
commissioned Cartesian motion are not implemented yet. `tasks/click_bell/` contains one
narrow, deterministic detector and robust depth-plane estimator for the yellow press
cap above a blue bell. It is composed into ClickBell sessions as the read-only
`click_bell_localize_press_cap` tool; it is not a general object detector. A localized
surface or passed read-only plan is geometric evidence, not a safe Cartesian command.

Camera observation never enables motion. `agent_motion.enabled` remains an independent
operator setting, and its current tool is still limited to an evidence-bound single
joint step.
