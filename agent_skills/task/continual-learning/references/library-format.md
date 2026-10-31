# Skill library format

```text
skill-library/
  INDEX.md
  skills/<slug>/
    SKILL.md
    code/run.py
    code/<slug_with_underscores>/
    evidence/runs.jsonl
    evidence/images/*
```

`SKILL.md` frontmatter must contain `name`, `description`, `status`, `robot`, `entry`
and integer `version`; `entry` is `code/run.py`, and status is `draft`, `verified` or
`deprecated`. Optional `uses` lists other library skill slugs.

The entry exports `main(ctx, **params)` and returns `{"ok": bool, "measured": {...},
"why": str}`. It checks preconditions, uses fresh observations, and reaches hardware
only through `ctx`.

Each `evidence/runs.jsonl` row contains `session`, `date`, `commit`, `outcome`,
`unattended`, `conditions`, `checks`, and non-empty `images`. Image paths stay under
`evidence/images/`. `verified` requires two unattended successful rows.
