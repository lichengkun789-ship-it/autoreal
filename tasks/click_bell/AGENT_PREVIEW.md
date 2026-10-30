This is an operator-supervised commissioning run, not a bell-press attempt.

1. Call `status` and require fresh, healthy Piper feedback.
2. Confirm `click_bell_approach_preview` is armed and has one remaining call.
3. Call `click_bell_approach_preview` exactly once with no arguments.
4. Call `status` again and report the motion receipt, retreat result, and final error.

Do not call generic joint motion, do not attempt contact, and do not claim that the
bell was pressed. Stop on any refusal or uncertain control state.
