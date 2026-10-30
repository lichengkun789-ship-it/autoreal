# ClickBell task bring-up

ClickBell is the first task package layered on the task-neutral Piper backend. Its
current tool detects the yellow press cap geometrically associated with the largest
blue bell component in a saved RGB-D frame. It erodes the selected component, rejects
gross depth outliers, fits the remaining surface plane, and returns the surface centre,
normal, fit quality, pixel, and legacy centre-pixel depth in that camera's named Piper
base frame. It never commands the robot.

## Current commissioned cameras

The operator-reviewed runtime configuration is
`/home/edemlab/autorax_ws/artifacts/config/piper_rgbd_active.json`. As of 2026-10-30:

- `cam_high` is calibrated in `piper_left_base`. Its 21-sample hand-eye holdout passed
  at 4.436 mm RMS and 6.052 mm maximum translation error; five static captures passed
  at 0.083 mm RMS and 0.155 mm maximum repeatability error.
- `cam_right_wrist` is calibrated in `piper_right_base`. Its five-pose validation
  passed at 3.824 mm RMS and 6.662 mm maximum translation error.
- `cam_left_wrist` is provisionally enabled by an explicit operator tolerance. The
  newer hand-eye candidate reports 6.875 mm holdout RMS and 8.860 mm maximum error,
  so it still fails the standard 5 mm RMS commissioning gate. The active configuration
  exposes its left-base transform under a temporary 7.5 mm RMS / 10 mm maximum policy;
  precision contact motion remains uncommissioned. The earlier 7.293 mm five-pose
  result evaluated the legacy matrix, not this newer candidate.

The active configuration keeps `agent_motion.enabled=false`. Calibration activation
does not authorize robot motion.

## Offline detector

Run the detector against any previously saved `piper_observe` frame:

```bash
.venv/bin/python scripts/detect_click_bell_press_cap.py \
  /path/to/runtime/frames/<frame_id> \
  --camera cam_right_wrist \
  --output /private/path/press_cap_localization.json \
  --overlay /private/path/press_cap_overlay.jpg
```

The detector deliberately uses color and component geometry rather than an external
vision model. A high-confidence result still means only that the image and aligned
depth support the returned point. `position_m` is the fitted surface centre;
`legacy_center_position_m` preserves the former centre-pixel result. The reported
approximate 95-percent fit spread covers depth and plane-fit residuals only and
explicitly excludes camera-extrinsic error.

On the saved three-camera frame `20261030T065405Z33c462`, surface fitting reported
0.711 mm plane RMSE from `cam_high`, 0.595 mm from `cam_right_wrist`, and 0.352 mm from
`cam_left_wrist`. The high and left-wrist centres still disagreed by 22.62 mm in
`piper_left_base`; therefore the new estimator improves within-view depth evidence but
does not clear the provisional left-wrist calibration for cross-view fusion.

## Read-only press-path audit

`audit_click_bell_press_path.py` converts a fitted surface contact point into the
Piper SDK TCP frame using the owned tool profile, then checks staged 60 mm, 20 mm,
5 mm, and nominal-contact targets. It never sends motion. RPY remains explicit.

The Piper URDF places both gripper-finger joint origins 135.8 mm along the
gripper-base tool axis. The operator accepted this as matching the installed Piper on
2026-10-30, so AutoReal owns `[0, 0, 0.1358] m` as the closed-gripper centre contact
offset. Pass `--unverified-tool-geometry` only when deliberately testing another tool.

```bash
.venv/bin/python scripts/audit_click_bell_press_path.py \
  /path/to/runtime/frames/<frame_id> \
  --camera cam_right_wrist --arm right \
  --rpy-deg 180 10 -150 \
  --backend-options /home/edemlab/autorax_ws/artifacts/config/piper_rgbd_active.json \
  --output /home/edemlab/autorax_ws/artifacts/plans/click_bell_press_path.json
```

Even a geometrically passed report has `motion_authorized=false` and
`execution_admissible=false`. `geometry_admissible` can become true only after the tool
offset, workspace bounds, and distal-link minimum height are operator-commissioned;
execution still requires operator commissioning of the guarded press runner.

## Guarded contact and retreat

`services/piper/contact_motion.py` contains an operator-only press executor. It uses
the audited joint waypoints, accepts only a live pose matching the audited start, and
requires the gripper to be closed for the commissioned centre-contact geometry. It
treats the final 5 mm stage as contact travel. Contact terminates after a characterized
motor-effort increase or repeated tracking stall. The executor then commands the fresh
measured joints as a position hold and returns through only the waypoints actually
reached. Any failure attempts the same measured-position hold.

This is software behavior, not a safety-rated emergency stop. The effort threshold is
required because it must be measured on this arm; the project does not invent one.
The runner remains outside the Agent tool surface:

```bash
.venv/bin/python scripts/run_piper_guarded_press.py \
  --backend-options /home/edemlab/autorax_ws/artifacts/config/piper_rgbd_active.json \
  --plan /path/to/admitted_press_path.json \
  --probe-receipt /path/to/fresh_readonly_probe.json \
  --confirmation /path/to/software_hold_confirmation.json \
  --effort-delta-nm MEASURED_VALUE
```

Without `--execute`, this validates admission and writes a plan receipt. Execution is
still refused until the press path has commissioned workspace bounds and distal-link
clearance. Piper gripper SDK conversion and hard command bounds now live in
`services/piper/gripper.py`. The general backend exposes no gripper command; the
ClickBell task can opt into the separately commissioned `click_bell_set_gripper` tool.

### Gripper and effort commissioning

Before a guarded press, characterize the selected empty gripper with the operator-only
close-and-restore runner. It sends no arm-joint command, closes in bounded increments,
monitors both arms for unexpected joint drift, and restores the measured initial
opening. The 2026-10-30 right-arm run used 10 mm increments and a 0.5 N m gripper
command: feedback moved from 98.8 mm to 2.3 mm and returned to 96.3 mm. Its receipt is
stored under `artifacts/bringup/gripper/piper_gripper_cycle_20261030_right.json` in the
AutoRAX workspace.

```bash
.venv/bin/python scripts/run_piper_gripper_cycle.py \
  --backend-options /path/to/piper_rgbd_active.json \
  --probe-receipt /path/to/fresh_readonly_probe.json \
  --confirmation /path/to/software_hold_confirmation.json \
  --arm right --step-m 0.01 --effort 500 --execute
```

The read-only effort-noise runner records a stationary six-joint baseline without
sending motion commands. On 2026-10-30 the right arm remained at zero measured joint
drift and the largest stationary peak-to-peak effort was 0.064215 N m at J6. This does
not admit a contact threshold: gravity, friction, and acceleration during a no-contact
approach must be measured before `--effort-delta-nm` is commissioned.

```bash
.venv/bin/python scripts/characterize_piper_effort_noise.py \
  --backend-options /path/to/piper_rgbd_active.json \
  --arm right --samples 200 --interval-s 0.05
```

Workspace bounds are a task-specific allowed TCP box, not Piper's factory joint
limits. The distal-link minimum height is a separate floor for the wrist and final arm
links. Both remain uncommissioned until the operator maps the real table, obstacles,
and clear approach corridor in the selected arm-base frame; therefore dynamic
no-contact approach characterization and guarded pressing remain blocked.

Workspace commissioning is per arm. `derive_piper_workspace_candidate.py` computes a
narrow review-only corridor from every FK waypoint in a passed press plan. It does not
edit the active configuration, keeps Agent motion disabled, and records the source-plan
hash. For the 2026-10-30 right-arm plan, the observed TCP envelope was
`[0.092397, 0.028943, 0.171405]` to `[0.353880, 0.101839, 0.242784]` m. A 10 mm
margin produced this candidate:

```json
{
  "workspace_bounds": {
    "right": {
      "min": [0.082397, 0.018943, 0.161405],
      "max": [0.363880, 0.111839, 0.252784]
    }
  },
  "link_min_z_m": 0.161405
}
```

Generate and audit it without motion:

```bash
.venv/bin/python scripts/derive_piper_workspace_candidate.py \
  --plan /path/to/passed_press_path.json \
  --backend-options /path/to/piper_rgbd_active.json --arm right \
  --tcp-margin-m 0.01 0.01 0.01 --link-margin-m 0.01 \
  --candidate-config /path/to/piper_click_bell_right_workspace_candidate.json \
  --output /path/to/piper_click_bell_right_workspace_candidate_receipt.json

.venv/bin/python scripts/audit_click_bell_press_path.py \
  /path/to/runtime/frames/<frame_id> --camera cam_right_wrist --arm right \
  --rpy-deg 180 10 -150 \
  --backend-options /path/to/piper_click_bell_right_workspace_candidate.json \
  --output /path/to/candidate_path_audit.json
```

Passing this audit proves only that the planned geometry stays inside the numeric
corridor. The operator must still inspect the real swept volume for the table, bell
fixture, cameras, cables, and the other arm before promoting these values into the
active backend configuration.

## Calibration gate

Each camera result remains in either `piper_left_base` or `piper_right_base`. Never
combine those frames using the simulation's arm-base placement. Before right-arm
pressing, validate `T_right_tcp_camera` by observing one fixed target from at least five
right-arm poses. The transformed target should remain fixed; the initial project gate
is at most 5 mm RMS and 10 mm worst-case spread. These are AutoRAX commissioning gates,
not manufacturer specifications.

Multi-camera or dual-arm fusion additionally requires measured transforms from both
real arm bases into one table/world frame. That is a separate calibration from the
camera intrinsics and eye-in-hand transform.

### Recalibration procedure

Use the existing 100 mm `DICT_APRILTAG_36h11`, marker id `0`, as a physical target,
but implement and run the collector from AutoRAX rather than importing another
workspace at runtime.

- Right wrist (`right_tcp`): fix the tag to the table, move the right arm through at
  least 20 poses with meaningful translation and rotation diversity, and record
  synchronized `T_base_tcp` plus `T_camera_tag`. Solve `T_tcp_camera` with OpenCV
  hand-eye calibration.
- High camera (`left_base`): fix the tag to the left end effector, move that arm
  through at least 20 diverse poses, and solve the fixed `T_left_base_camera`.
- Keep several poses out of the solver. On those holdout poses, transform the tag into
  the appropriate arm base and report translation/rotation RMS and worst-case error.
- Import only a result that passes the holdout gate, then repeat the static-target
  point consistency test using live RGB-D depth.

The old calibration utilities may be used as a read-only algorithm reference, but the
new repository must own its collector, solver, result schema, tests, and receipts.

AutoRAX owns that workflow through `calibrate_piper_wrist_extrinsics.py`. It keeps the
tag fixed on the table, records synchronized joint FK plus `T_camera_tag`, selects an
OpenCV hand-eye method on training poses, and evaluates the selected result on five
untouched holdout poses. A passed candidate is evidence only; the script never changes
the active backend configuration or authorizes motion.

Initialize one left-wrist run:

```bash
export RAX_ARTIFACT_ROOT=/home/edemlab/autorax_ws/artifacts
.venv/bin/python scripts/calibrate_piper_wrist_extrinsics.py init \
  --backend-options "$RAX_ARTIFACT_ROOT/config/piper_rgbd_calibrated_candidate.json" \
  --camera cam_left_wrist
```

Keep the tag fixed. After the operator places the arm at each substantially different,
stationary pose, append exactly one sample with:

```bash
.venv/bin/python scripts/calibrate_piper_wrist_extrinsics.py capture \
  --run /path/printed/by/init
```

Use at least 15 samples and prefer 20 or more with broad translation and wrist-rotation
coverage. Check progress or solve without moving the robot:

```bash
.venv/bin/python scripts/calibrate_piper_wrist_extrinsics.py status \
  --run /path/to/run
.venv/bin/python scripts/calibrate_piper_wrist_extrinsics.py solve \
  --run /path/to/run
```

Only a candidate that passes the held-out 5 mm RMS and 10 mm maximum translation gates
may proceed to a separate static-target validation run. Activation remains an explicit
operator-reviewed configuration change after that validation.

For the fixed high camera, rigidly attach the 100 mm tag to the left TCP. The tag must
not move relative to the TCP during the run, and `cam_high` must remain fixed. Initialize
the separate eye-to-hand workflow with:

```bash
export RAX_ARTIFACT_ROOT=/home/edemlab/autorax_ws/artifacts
.venv/bin/python scripts/calibrate_piper_fixed_camera_extrinsics.py init \
  --backend-options "$RAX_ARTIFACT_ROOT/config/piper_rgbd_calibrated_candidate.json" \
  --camera cam_high
```

After the operator places the left arm at each stationary pose, capture one sample:

```bash
.venv/bin/python scripts/calibrate_piper_fixed_camera_extrinsics.py capture \
  --run /path/printed/by/init
```

Collect at least 20 samples with broad translation and wrist rotation while keeping the
whole tag visible. Then run `status` and `solve` with the same `--run` path. The solver
estimates `T_left_base_camera`, evaluates five untouched holdout poses, and never
activates the candidate or authorizes motion.

After printing and fixing the 100 mm tag to the table, capture one right-wrist sample
without moving the robot:

```bash
export RAX_ARTIFACT_ROOT=/home/edemlab/autorax_ws/artifacts
.venv/bin/python scripts/validate_piper_extrinsics.py capture \
  --backend-options /private/path/piper_rgbd_calibrated_candidate.json \
  --camera cam_right_wrist
```

Repeat the command after the operator places the right arm at five substantially
different, stationary poses. Then evaluate the five printed receipt paths:

```bash
.venv/bin/python scripts/validate_piper_extrinsics.py evaluate \
  /path/to/sample-1.json /path/to/sample-2.json /path/to/sample-3.json \
  /path/to/sample-4.json /path/to/sample-5.json
```

Both commands are read-only. A passed consistency evaluation validates the evidence
set only; it does not authorize motion.

## Motion gates

The shortest controlled path to a press primitive is:

1. Validate the right-wrist external calibration on a static target.
2. Validate the implemented read-only `piper_plan_pose` IK and straight-line waypoint
   checks against operator-configured workspace bounds. Joint limits, per-waypoint joint
   steps, and Jacobian conditioning are reported, but the active configuration still has
   no commissioned workspace box or distal-link minimum height.
3. Characterize an operator-only 5 mm Cartesian step and last-target hold.
4. Characterize a task-neutral approach and retreat path above the detected point.
5. Add bounded downward increments with contact or stall termination and a strict travel cap.
6. Expose the resulting press macro to the Agent only through operator arming, a task
   allowlist, one-call budget, timeout, receipt, and guaranteed retreat attempt.

These gates admit only the commissioned right-arm ClickBell path. Other poses, arms,
objects, or task geometries remain perception/planning-only until separately tested.

### Right-arm no-contact commissioning result (2026-10-30)

The task-scoped candidate workspace passed the full staged physical path. This does
not prove that contact pressing succeeded.

- Configuration: `artifacts/config/piper_click_bell_right_workspace_candidate.json`
- Read-only path audit: `artifacts/plans/click_bell_right_workspace_candidate_audit_20261030.json`
- Preflight: `artifacts/probes/piper_readonly_20261030_no_contact_retry_preflight.json`
- 60 mm receipt: `artifacts/bringup/no_contact_approach/piper_right_prefix64_20261030.json`
- 20 mm receipt: `artifacts/bringup/no_contact_approach/piper_right_clearance20mm_20261030.json`
- 5 mm receipt: `artifacts/bringup/no_contact_approach/piper_right_clearance5mm_20261030.json`
- Guarded press receipt: `artifacts/bringup/guarded_press/piper_right_guarded_press_execute_20261030.json`
- Postcheck: `artifacts/probes/piper_readonly_20261030_guarded_press_postcheck.json`

The operator confirmed the swept corridor was clear. At 2% speed, the right arm passed
all 64 waypoints to 60 mm clearance, all 88 cumulative waypoints to 20 mm, and all 104
cumulative waypoints to 5 mm. Every stage returned through the reached waypoints and
finished within 0.01 rad of the audited start. The closed-gripper guarded run then
executed the 5 mm contact stage and retreated, but its effort/stall detector did not
fire, so its status is `contact_unverified`. Visual operator evidence is still needed
to decide whether the yellow cap actually moved.

The measured Piper settling behavior supports a 5 s waypoint timeout and 0.002 rad
settling tolerance for this path. The first Agent capability is intentionally narrower
than the operator runner: `click_bell_approach_preview` replays only the commissioned
64-waypoint path to 60 mm clearance and its reverse retreat. It requires a live
operator arming lease, permits one call per lease, and binds the plan, commissioning
receipt, backend safety configuration, arm, distance, speed, and waypoint count. It
does not contact the bell. Generic `piper_move_joint` remains disabled.

### Agent approach-preview result (2026-10-30)

DeepSeek Harness completed the first end-to-end RAX-to-Piper motion trial through the
task-specific MCP tool. It independently read the task and live contract, checked
fresh robot status and the arming budget, called `click_bell_approach_preview {}` once,
and reacquired status after the motion.

- Session: `artifacts/sessions/20261030T081140Z_click_bell_6861b1/`
- Commissioning report: `piper-agent-click-bell-preview.json`
- Motion receipt: `piper-motion/agent_click_bell_preview_20261030T081337Z_18b2ee02.json`
- Independent postcheck: `artifacts/probes/piper_readonly_20261030_agent_preview_postcheck.json`
- Result: 64-waypoint right-arm approach to 60 mm clearance, reverse retreat passed,
  final audited-start error `0.006475 rad`, no contact intended.
- Postcondition: both arms remained ready and error-free, the one-call budget was
  exhausted, and the runner automatically disarmed the session.

This confirms Agent-selected physical motion through the AutoRAX boundary. It does not
confirm a bell press.

### Guarded-press Agent tool and recording (2026-10-30)

The second task-specific motion tool is `click_bell_guarded_press`. It does not accept
a target pose or motion parameter from the model. Startup validates hashes and
semantics for the audited plan, the complete operator guarded-press receipt, the
immediately preceding 5 mm no-contact receipt, and the active backend safety
configuration. It requires a live arming lease, a closed gripper, fresh healthy
feedback, and an initial pose within 0.01 rad of the audited start. One arming lease
permits one call.

Every admitted attempt starts a session-owned episode recorder before any motion. It
saves one MP4 per configured camera, aligned depth PNGs, before/after stills, camera
timestamps, and measured joint/effort/speed/gripper samples under
`<session>/recordings/<recording_id>/`. Recording failure does not interrupt the
executor's retreat attempt, but it prevents the tool call from being accepted as a
complete experiment.

The paired `click_bell_set_gripper` tool accepts only `close` or `open`; it never
accepts a numeric target from the model. Its width, 10 mm incremental step, effort,
right-arm selection, feedback gates, and 0.002 rad arm-drift limit come from the passed
close-and-restore characterization receipt. One arming lease enforces this state
machine:

1. `close` from the commissioned open width;
2. one `click_bell_guarded_press` with confirmed reverse retreat;
3. `open` back to the commissioned width.

Opening before retreat, repeating either gripper action, switching arms, or changing
the arming lease is rejected locally. Close and open each produce their own three-camera
MP4/depth/telemetry recording, so a complete trial has three recording manifests:
gripper close, press/retreat, and gripper open.

After retreat and gripper opening, task success remains pending. The Agent must call
`request_operator(kind="visual_confirmation")`; the human reply must begin with
`success`, `failure`, or `unclear`. `click_bell_confirm_visual_result` verifies the
durable operator request/reply pair and links the evaluation to the motion receipt and
recording. Force, stall, sound, or the model's own interpretation cannot mark success.

The first live DeepSeek Flash commissioning attempt reached all read-only prechecks,
but the provider's final model request did not return, so it was interrupted before
the press tool was called. Its MCP audit contains zero motion requests; the runner
disarmed the session, and the independent five-sample postcheck passed with zero
transmitted CAN frames. This is a harness/provider interruption, not a completed press
experiment.
