# ClickBell task bring-up

ClickBell is the first task package layered on the task-neutral Piper backend. The
active Agent contract contains no `click_bell_*` robot tools. DeepSeek or a
research-mode perception program identifies the yellow press cap in a fresh RGB-D
frame, converts pixels with `piper_pixel_to_position`, and composes generic Piper
planning, segmented execution, hold, retreat, and gripper tools.

## AutoRAX ownership and invocation

- `tasks/click_bell/` owns the task goal, reset procedure, optional offline detector,
  and evaluator.
- `agent_skills/task/click-bell/SKILL.md` tells the Agent how to use the live contract.
- `services/piper/` owns task-independent Piper geometry, motion, gripper, hold,
  recording, and receipts.
- `services/piper/task_adapters.json` intentionally registers no ClickBell adapter.
- `scripts/run_click_bell_experiment.sh` creates and launches one complete AutoRAX session.

The experiment wrapper does not prescribe actions. DeepSeek reads `TASK.md` and the
installed Skill, then chooses and sequences generic Piper MCP tools. Geometry, IK,
motion, retreat, gripper commands, and complete-session recording execute locally;
the model never receives direct SDK access.

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
2026-10-30, so AutoRAX owns `[0, 0, 0.1358] m` as the closed-gripper centre contact
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
clearance. Piper gripper SDK conversion and hard command bounds live in
`services/piper/gripper.py`; commissioning admission and Agent execution live in
`services/piper/gripper_motion.py` and are exposed generically as `piper_gripper`.

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

These gates admit only the commissioned right-arm ClickBell geometry and motion
envelope. Dynamic targets must still pass the same workspace, link-height, joint-limit,
step-size, feedback, contact-travel, and retreat checks.

### Dynamic target planning and region audit

The Agent never presses a fixed button coordinate. It captures fresh overview and
wrist RGB-D images, selects several pixels inside the current yellow cap, rejects depth
outliers, and converts the retained pixels with `piper_pixel_to_position`. It then
calls `piper_plan_pose` for the configured closed-gripper tool point. The returned
`plan_id` is short-lived and bound to measured state and the current arming lease.

`piper_execute_plan(plan_id, waypoints=N)` advances only `N` planned waypoints and
returns progress to the Agent. Between segments the Agent can observe, continue,
`piper_hold`, partially retreat with `piper_retreat_plan(..., waypoints=M)`, fully
retreat by omitting the count, or abandon the plan and generate a revised research
controller. This is Agent-supervised adaptation between deterministic local motion
segments; it is not language-model hard-real-time control.

The backend still checks configured tool-point workspace bounds, distal-link minimum
height, IK convergence, Piper joint limits, adjacent waypoint changes, fresh feedback,
arming lease, and execution budget. Those are robot capability constraints, not a
task-specific placement box or a fixed ClickBell path.

The configured maximum change between adjacent planned joint waypoints is `0.25 rad`.
The same `0.25 rad` ceiling applies when the executor compares each command with live
measured feedback. Settle tolerance is used only to decide whether a target has been
reached; it does not enlarge the command budget or change Piper's mechanical limits.

Saved observations from the centre and boundary of a test region can still be checked
offline without opening CAN or sending motion. The script below is retained as
commissioning analysis, not as an Agent MCP tool:

```bash
.venv/bin/python scripts/audit_click_bell_dynamic_region.py \
  --sample center=/path/to/center/frame \
  --sample left_edge=/path/to/left-edge/frame \
  --sample right_edge=/path/to/right-edge/frame \
  --camera cam_right_wrist \
  --arm right \
  --template /path/to/commissioned-click-bell-plan.json \
  --backend-options /path/to/private-piper-click-bell.json \
  --disable-workspace-check \
  --output /path/to/click-bell-region-audit.json
```

The report contains localization quality and IK admission for each sample and always
sets `motion_authorized: false`. Active Agent execution uses generic plans from fresh
observations instead of consuming this report.

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
settling tolerance for this commissioned path. The obsolete fixed-path Agent preview
and task-specific MCP motion adapter are not part of the active contract. Generic
`piper_move_joint` remains disabled for ClickBell; research uses tool-point
`piper_plan_pose`, segmented `piper_execute_plan`, selectable `piper_retreat_plan`,
`piper_hold`, and `piper_gripper`.

### Generic Agent experiment and recording

Start a fresh operator-present DeepSeek Flash experiment with:

```bash
cd /home/edemlab/autorax_ws/autorax
bash scripts/run_click_bell_experiment.sh
```

The wrapper creates a fresh session from the private commissioned backend under
`RAX_ARTIFACT_ROOT` and launches the normal Harness. Override it with
`RAX_CLICK_BELL_BACKEND_OPTIONS` when commissioning evidence changes. There is no
task-options file and no task-specific robot adapter.

When the Agent asks for visual confirmation, keep the first terminal running and use a
second terminal. Pass either the printed session directory or its `operator-input.json`:

```bash
cd /home/edemlab/autorax_ws/autorax
.venv/bin/python scripts/reply_operator.py \
  --session /home/edemlab/autorax_ws/artifacts/sessions/SESSION/operator-input.json \
  --result success \
  --note "operator observed the button move down"
```

Use `--result failure` or `--result unclear` when appropriate. This command only
answers the pending request and cannot move the robot.

The active robot tools are generic: `piper_observe`, `piper_pixel_to_position`,
`piper_plan_pose`, `piper_execute_plan`, `piper_retreat_plan`, `piper_hold`, and
`piper_gripper`. The Agent selects the live target, waypoint segment size, continuation
decision, and retreat depth. A local executor performs each bounded segment and holds
before returning control. In research mode the Agent may write and test a more capable
controller, then promote it through the skill-library lifecycle; execution mode runs
only frozen skills.

The Piper backend starts one session recorder before Agent work begins and stops it
when the robot service closes. It saves one MP4 per configured camera, aligned depth
PNGs, before/after stills, camera timestamps, and measured
joint/effort/speed/gripper samples under `<session>/recordings/<recording_id>/`. This
covers perception, planning delays, all motion segments, retreat, gripper actions, and
the operator-confirmation period as one experiment.

After the attempt, the Agent calls
`request_operator(kind="visual_confirmation")`; the reply begins with `success`,
`failure`, or `unclear`. `scripts/run_task_experiment.py` reads that durable reply and
passes it to the task manifest evaluator after the Harness exits. It also rejects a
run that used any `click_bell_*` robot tool, lacks one passed complete-session
recording, remained armed, or failed task evaluation. Force, stall, sound, or the
model's interpretation cannot mark success.
