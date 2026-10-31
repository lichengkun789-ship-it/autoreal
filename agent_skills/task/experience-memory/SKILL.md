---
name: experience-memory
description: Save successful {{ robot.name }} action tips and retrieve them for future similar actions with matching observable conditions within this task.
---

# Experience memory

Memory under `{{ vocab.memory }}/` belongs to this experiment session. It survives
resume and compaction but is not copied into later sessions. At start or after
compaction, read `{{ vocab.memory }}/PROGRESS.md` and retrieve only matching notes.
Open linked evidence when applicability or outcome is uncertain; do not load the
entire history. Cross-session executable procedures belong in `skill-library/`, as
described by [continual-learning](../continual-learning/SKILL.md).

## Record successful tips immediately

- As soon as an action produces a visibly successful, useful result, add or update a
  concise row in `{{ vocab.memory }}/TIPS.md`.
- Record the observable preconditions, task phase, tool and arguments or reusable pose
  relation, visible effect, and the saved frame paths. State what made it work so
  later attempts avoid repeating exploration or a superseded failure.
- Update the matching tip when the same maneuver is refined.
- The real scene drifts between attempts (objects are moved back by hand, lighting
  changes): record the conditions under which a tip held, not only its numbers.

## Distill

When a tip becomes an automated, reusable motion, contact strategy, recovery, or
completed procedure, package it in `skill-library/` with executable code and run
evidence. Keep attempt-specific notes in memory. Current perception and live tool
schemas take precedence over all notes and learned skills.

Read [references/format.md](references/format.md) only when creating or updating a
learned skill.
