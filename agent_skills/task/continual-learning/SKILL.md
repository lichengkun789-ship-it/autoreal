---
name: continual-learning
description: Reuse and grow the persistent library of executable robot skills across sessions.
---

# Continual learning

`skill-library/` is the persistent executable memory for this robot. `memory/` belongs
only to the current experiment; `.agents/skills/` contains harness instructions.

## Start by reusing

Read `skill-library/INDEX.md`. Load only skills whose preconditions match the current
task and live observation. Prefer `verified` over `draft`; do not run a skill outside
its documented tested ranges. A reusable skill calls the robot only through
`api.runtime.Context` and reacquires the scene instead of replaying coordinates.

## Research mode

Explore with the live MCP tools, then write and test an end-to-end program under
`code/<project>/`. Once it works, package one stable sub-task under
`skill-library/skills/<slug>/` using
[the library format](references/library-format.md). Commit the library branch, then
call `skill_library_merge`. The host validates the package and merges it into the
persistent robot library. A draft may record failures; `verified` requires at least
two unattended successful runs with images.

## Execution mode

The workspace and skill library are read-only. Select a compatible frozen skill and
run its entry without editing it. Record the outcome in the session evidence. A failed
execution returns to a later research session for repair; never patch a skill during
its independent evaluation.

Skills never widen MCP schemas, robot limits, arming rules, or backend safety checks.
