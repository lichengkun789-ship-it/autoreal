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

Operator configuration can add one mutating tool, `piper_move_joint`. Admission is in
`services/piper/agent_motion.py`; SDK translation and the hold latch remain in
`services/piper/hold.py`; measured execution is shared with the operator
characterization runner in `services/piper/minimum_motion.py`. The tool is absent from
the MCP contract unless matching characterization evidence and an explicit move
allowlist are configured. Session arming lives outside the Agent workspace, and every
call writes a private receipt under the session root.

Task definitions and generated Agent code still call only MCP. They cannot import the
Piper SDK, widen the configured envelope, enable motors, reset faults, or bypass the
single robot-service owner.

Task-specific physical primitives follow the same ownership boundary. ClickBell's
Agent adapter is in `tasks/click_bell/tools.py`; evidence validation and Piper command
execution remain under `services/piper/`. `click_bell_guarded_press` replays one
commissioned plan, then writes a motion receipt and an episode recording through
`services/piper/episode_recording.py`. The separate confirmation tool accepts only a
durable operator reply and evaluates it with the robot-independent task evaluator.

## Franka reference mapping

RAX `real/franka` is an architectural reference, not a hardware abstraction that can
be reused unchanged. AutoRAX keeps the same separation of concerns while giving Piper
implementations and safety assumptions their own modules.

| Franka responsibility | Piper location | Piper status |
| --- | --- | --- |
| Backend tools and receipts | `services/piper/backend.py` | State, RGB-D observation, joint dry-run, and calibrated pixel localization implemented |
| Calibration transforms | `services/piper/calibration.py` | Loading and validation implemented; calibration collection/solving is pending |
| Task perception and adapters | `tasks/<task_id>/`, composed by `services/task_tools.py` | ClickBell localization, guarded-press adapter, and visual result evaluation implemented |
| Forward kinematics | `services/piper/kinematics.py` | Implemented against `piper_sdk==0.6.2` modified-DH geometry |
| Inverse kinematics | `services/piper/ik.py`, exposed by `piper_plan_pose` | Read-only numerical IK and straight-path joint checks implemented; motion use is not commissioned |
| Robot bridge/watchdog | `services/piper/sdk_reader.py`, `hold.py` | Feedback and restricted last-target hold implemented |
| Operator arming | controller lease plus evidence-bound motion controllers | Implemented for characterized joint steps and commissioned ClickBell paths |
| Cartesian servo and trajectory planning | `ik.py`, `contact_motion.py` | Read-only IK is general; execution is limited to one commissioned ClickBell path |
| Gripper/contact/press primitive | `gripper.py`, `contact_motion.py`, `click_bell_press.py` | Operator gripper tools and one task-scoped guarded press implemented; no general Agent gripper tool |

Franka's short-step pressing behavior depends on its impedance controller. Piper does
not inherit that contact behavior. Every Piper press path must separately verify speed,
contact termination, workspace, retreat, timeout, and operator recovery rules before
it can appear in the MCP contract. The current ClickBell tool is evidence-bound to one
such path; it is not a reusable Cartesian servo.
