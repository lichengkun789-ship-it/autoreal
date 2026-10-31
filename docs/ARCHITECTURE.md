# AutoRAX architecture

AutoRAX keeps the agent harness, robot backend, task definitions, and runtime
artifacts in separate ownership boundaries.

```text
selected agent harness (Claude Code or DeepSeek Harness)
    -> generated agent workspace
    -> MCP frontend
    -> registered RAX backend
    -> Piper SDK / pyrealsense2
    -> dual Piper hardware / configured RealSense cameras
```

Only `services/piper/` may import `piper_sdk`. Task packages call the stable RAX
runtime API and never import a robot backend. Agent skills explain how to choose and
interpret tools; they do not implement control. Session logs, frames, generated code,
and learned task memory live under `RAX_ARTIFACT_ROOT`.

Each session declares an Agent mode. `research` provides a writable, offline coding
sandbox; `execution` provides the same MCP tools with a read-only workspace. A
Git-backed library at `RAX_ARTIFACT_ROOT/skill-library/<robot>/` carries verified
executable procedures across sessions. Session work is isolated on a branch and can
reach the persistent library only through the validating `skill_library_merge` tool.

`harness/launcher.py` dispatches the operator-owned configuration to an adapter in
`harness/claude_cli/` or `harness/deepseek_cli/`. Both adapters generate the same MCP
contract and start the same single-owner robot frontend. Provider credentials and
vendor session homes remain outside the agent workspace.

## Current milestone

The Piper backend is read-only by default. It connects to the configured CAN
interfaces, reports measured dual-arm state, checks feedback freshness, and dry-runs
joint targets against joint limits and a per-call step cap. Optional `rgb-only` and
`rgbd` profiles add serial-bound RealSense observations without changing the motion
gate. Camera ownership and lifecycle live in `services/piper/camera.py`; MCP framing
and observation receipts live in `services/piper/backend.py`. Forward kinematics lives
in `services/piper/kinematics.py`; `services/piper/calibration.py` loads private fixed
or eye-in-hand extrinsics and performs read-only RGB-D point transforms.

Operator configuration can independently add the characterized `piper_move_joint`
primitive or generic plan-bound Cartesian execution. Joint-step admission is in
`services/piper/agent_motion.py`; SDK translation and the hold latch remain in
`services/piper/hold.py`; measured execution is shared with the operator
characterization runner in `services/piper/minimum_motion.py`. The tool is absent from
the MCP contract unless matching characterization evidence and an explicit move
allowlist are configured. Session arming lives outside the Agent workspace, and every
call writes a private receipt under the session root.

Cartesian admission and one-shot plan ownership live in
`services/piper/cartesian_motion.py`; reusable measured waypoint execution, hold, and
reverse recovery live in `services/piper/trajectory.py`. `piper_plan_pose` remains
read-only but can stage a lease-bound plan when this capability is enabled.
`piper_execute_plan` advances by a caller-selected number of waypoints and
`piper_retreat_plan` reverses a caller-selected number already executed. `piper_hold`
and the task-independent `piper_gripper` complete the Agent-facing Piper primitives.
Contact behavior is deliberately not claimed as impedance or force control.

Task definitions and generated Agent code still call only MCP. They cannot import the
Piper SDK, widen the configured envelope, enable motors, reset faults, or bypass the
single robot-service owner.

ClickBell no longer registers a Piper task adapter. Its goal, visual strategy, and
evaluator remain under `tasks/click_bell/` and `agent_skills/task/click-bell/`; the
Agent composes only generic Piper tools. The complete backend lifetime is recorded by
`services/piper/episode_recording.py`, independent of task phases. Durable operator
visual confirmation is evaluated by the task manifest after the Harness exits.

Task manifests also name the evaluator and optional task Skill. Experiment runners
evaluate semantic artifacts and task outcomes rather than requiring the Agent to
produce one exact tool-call sequence.

## Franka reference mapping

RAX `real/franka` is an architectural reference, not a hardware abstraction that can
be reused unchanged. AutoRAX keeps the same separation of concerns while giving Piper
implementations and safety assumptions their own modules.

| Franka responsibility | Piper location | Piper status |
| --- | --- | --- |
| Backend tools and receipts | `services/piper/backend.py` | State, RGB-D observation, joint dry-run, and calibrated pixel localization implemented |
| Calibration transforms | `services/piper/calibration.py` | Loading and validation implemented; calibration collection/solving is pending |
| Task perception and evaluation | `tasks/<task_id>/` | ClickBell localization and visual result evaluation implemented |
| Robot-specific task adapter | `services/piper/<task_id>/`, composed by `services/task_tools.py` | Extension point retained; ClickBell needs no adapter |
| Forward kinematics | `services/piper/kinematics.py` | Implemented against `piper_sdk==0.6.2` modified-DH geometry |
| Inverse kinematics | `services/piper/ik.py`, exposed by `piper_plan_pose` | Numerical IK and straight-path checks implemented; passed plans can be staged for configured Cartesian execution |
| Robot bridge/watchdog | `services/piper/sdk_reader.py`, `hold.py` | Feedback and restricted last-target hold implemented |
| Operator arming | controller lease plus evidence-bound motion controllers | Implemented for joint steps, generic Cartesian plans, and commissioned task paths |
| Cartesian trajectory execution | `ik.py`, `cartesian_motion.py`, `trajectory.py` | General lease-bound planning, segmented execution, measured hold, failure recovery, and selectable reverse retreat implemented |
| Gripper/contact | `gripper.py`, `gripper_motion.py`, `contact_motion.py` | General commissioned Agent gripper implemented; operator contact runner exists, but no generic impedance/force-control tool |

Franka's short-step pressing behavior depends on its impedance controller. Piper does
not inherit that behavior. The Agent may compose short generic tool-point segments and
select a retreat depth, while the local executor owns deterministic waypoint timing,
feedback checks, and holds. This is supervisory closed-loop exploration, not hard
real-time force control; generic contact termination remains a capability gap.
