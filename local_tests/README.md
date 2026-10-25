# Regression tests

Run from the repository root in the robot Python environment:

```bash
$RAX_PYTHON -m pytest -q
```

The suite uses temporary artifact roots, the `mock` robot backend, fake hardware adapters and
fake model providers. It never moves hardware, opens cameras, calls a model or needs
credentials. Tests marked `hardware` are excluded by default; the archive tests skip
without `ffmpeg`.

DeepSeek Harness tests use a fake pinned package and fake credential. They validate
configuration, workspace generation, private-file handling, sandbox arguments, MCP
profile generation, resume identity, and usage accounting without npm, a provider, or
a robot connection.
