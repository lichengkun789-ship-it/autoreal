# ClickBell guarded press commissioning

Use the live tools to execute exactly one admitted ClickBell task.

1. Call `status` and confirm `click_bell_set_gripper` and `click_bell_guarded_press`
   are enabled and armed. Confirm the gripper phase is `awaiting_close`.
2. Call `click_bell_set_gripper` exactly once with `{"action":"close"}`.
3. After it reports phase `closed`, call `click_bell_guarded_press` exactly once with
   no arguments. Confirm its response reports a passed retreat and keep its `press_id`.
4. Call `click_bell_set_gripper` exactly once with `{"action":"open"}`. Confirm it is
   bound to the same `press_id` and reports phase `opened`. Do not call another motion
   tool.
5. Call `request_operator` with kind `visual_confirmation`. Ask the human to inspect
   the bell and the saved before/after views, then reply with `success`, `failure`, or
   `unclear` followed by an optional note.
6. If the request is answered, call `click_bell_confirm_visual_result` exactly once
   with the returned `request_id` and the press `press_id`.
7. Report the recorded evaluation and all three phase recording paths: gripper close,
   press/retreat, and gripper open. Never claim success from force, sound, or your own
   visual guess.

If any gripper, motion, recording, or retreat step fails, stop immediately and ask the
operator for help.
