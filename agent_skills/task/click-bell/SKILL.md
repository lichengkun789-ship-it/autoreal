---
name: click-bell
description: Explore and automate a tabletop ClickBell task by composing generic Piper perception, motion, gripper, hold, and operator tools.
---

# ClickBell

Treat this as a task program, not a robot backend. Use only generic Piper tools from
`MCP_SESSION.md`; there is no ClickBell motion command and no remembered bell pose.

## Explore from the live scene

1. Call `piper_status`, require fresh healthy feedback, and note the configured tool
   point, active plan progress, arming lease, and remaining budgets.
2. Call `piper_observe` for the overview and selected wrist camera. Locate the centre
   of the yellow press cap in the fresh full-resolution frame. In research mode, a
   tested image-processing program under `code/` may do this; otherwise use the vision
   model and verify the pixel against the returned image.
3. Convert several interior cap pixels with `piper_pixel_to_position`. Reject invalid
   depth and outliers. Use their median as the current cap surface point in the arm
   base frame. Never reuse a coordinate from a previous layout.
4. Close the selected gripper with `piper_gripper`. Plan a no-contact approach above
   the cap with `piper_plan_pose`; the position is the configured closed-gripper tool
   point, not the SDK flange.

Planning probes may stage a real plan. Call `piper_discard_plan` before trying a
different target when that staged plan has not been executed.

## Decide between every motion segment

Call `piper_execute_plan` with a caller-selected `waypoints` count. After each segment,
inspect its progress and reacquire status or images. The Agent, not this skill, chooses
the segment size. Continue only while the fresh evidence still supports the target.

When the current strategy cannot continue:

- call `piper_hold` first when the current command state is unclear;
- call `piper_retreat_plan` with the number of already executed waypoints that should
  be reversed; choose a partial retreat when a new view is enough, or omit `waypoints`
  for a full return;
- observe again and make a new plan from measured state;
- in research mode, revise or generate a controller under `code/click_bell/` when the
  failure is a missing decision rule rather than a robot fault. Do not edit a frozen
  execution-mode controller during the run.

This is supervisory closed-loop control: the Agent decides between bounded motion
segments. The Piper service performs each segment, monitors feedback, and holds; the
language model is not a hard-real-time servo.

## Press and evaluate

From a verified approach pose, plan a short downward tool-point motion to the cap.
Execute it in small observable segments. Stop advancing if the cap is no longer under
the tool or feedback becomes unhealthy. After the press attempt, retreat by a
caller-selected number of waypoints, open the gripper when appropriate, and capture a
fresh overview image.

Ask the operator for `visual_confirmation`; the operator reply is the task outcome
because this setup has neither sound nor tactile success sensing. A completed motion
is not proof that the bell was pressed.

After a successful research run, turn the observed policy into a resumable controller
whose steps each check preconditions, act, observe, and return evidence. Test it on
moved bell positions. Promote it to `skill-library/` only after the lifecycle and
independent evidence requirements in the continual-learning skill are met.
