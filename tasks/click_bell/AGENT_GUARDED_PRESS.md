# ClickBell guarded press commissioning

Use the live tools to execute exactly one admitted ClickBell press.

1. Call `status` and confirm `click_bell_guarded_press` is enabled, armed, and has one
   call remaining.
2. Call `click_bell_guarded_press` exactly once with no arguments.
3. Confirm its response reports a passed retreat and keep its `press_id`. Do not call
   any other motion tool.
4. Call `request_operator` with kind `visual_confirmation`. Ask the human to inspect
   the bell and the saved before/after views, then reply with `success`, `failure`, or
   `unclear` followed by an optional note.
5. If the request is answered, call `click_bell_confirm_visual_result` exactly once
   with the returned `request_id` and the press `press_id`.
6. Report the recorded evaluation and artifact paths. Never claim success from force,
   sound, or your own visual guess.

If any motion or retreat step fails, stop immediately and ask the operator for help.
