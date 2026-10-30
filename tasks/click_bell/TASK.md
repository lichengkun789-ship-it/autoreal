# Task: click_bell

Locate the yellow press cap on the blue tabletop bell, approach it from above, press
once, and retreat to a clear pose. A successful run needs visual evidence that the
cap moved down or the bell body reacted; sound is not required.

Perception-only sessions must stop after localization. When the live contract includes
`click_bell_approach_preview`, the operator has admitted exactly one no-contact path:
call it at most once, inspect its receipt, and do not claim that the bell was pressed.

When the live contract includes `click_bell_guarded_press`, call it at most once. It
replays only an operator-commissioned path, attempts the reverse retreat, and records
all three cameras plus robot telemetry. A contact signal is not task success. After the
arm has retreated, call `request_operator` with kind `visual_confirmation` and ask the
operator to reply `success`, `failure`, or `unclear`. Pass the returned `request_id` and
the press tool's `press_id` to `click_bell_confirm_visual_result`. Never infer or invent
the operator's answer, and never repeat a press in the same arming lease.
